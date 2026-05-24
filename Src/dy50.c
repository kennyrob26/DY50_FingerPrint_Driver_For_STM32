/*
 * dy50.c
 *
 *  Created on: May 16, 2026
 *      Author: kenny
 */


#include "dy50.h"

#include "string.h"

#include "math.h"

uint8_t response;


DY50_AckCode_t  DY50_Init(DY50_Typedef_t *dy50, UART_HandleTypeDef *huart, GPIO_TypeDef *touch_gpio_port, uint16_t touch_gpio_pin)
{
	if((dy50 == NULL) || (huart == NULL) || (touch_gpio_port == NULL))
		return ACK_ERROR_INVALID_PARAMETER;

	if(dy50->status != DY50_STATUS_UNINITIALIZED)			//already initialized
		return ACK_OK;

	DY50_AckCode_t code_return = ACK_OK;
	DY50_Init_State_Machine init_status;

	while((init_status != DY50_INIT_COMPLETE) || (code_return != ACK_OK))
	{
		switch (init_status)
		{
			case DY50_INIT_DEFINE_PARAMS:
				dy50->huart = huart;
				dy50->buf_tx.packet.header      = DY50_HEADER;
				dy50->buf_tx.packet.chip_adress = DY50_ADDRESS;

				dy50->touch_gpio.port = touch_gpio_port;
				dy50->touch_gpio.pin  = touch_gpio_pin;

				dy50->status = DY50_STATUS_IDLE;  //For uses readsystem and verify password functions

				code_return = ACK_OK;
				init_status = DY50_INIT_READ_SYSTEM_PARAMS;
				break;

			case DY50_INIT_READ_SYSTEM_PARAMS:
				code_return = DY50_CMD_ReadSystemParams(dy50);
				if(code_return == ACK_OK)
					init_status = DY50_INIT_VERIFY_PASSWORD;

			case DY50_INIT_VERIFY_PASSWORD:
				code_return = DY50_CMD_VerifyPassword(dy50, DY50_PASSWORD);
				if(code_return == ACK_OK)
				{
					init_status = DY50_INIT_COMPLETE;
				}
				break;

			default:
				break;
		}
	}

	if(init_status == DY50_INIT_COMPLETE)
		dy50->status = DY50_STATUS_IDLE;
	else
		dy50->status = DY50_STATUS_UNINITIALIZED;

	return code_return;

}



inline static uint8_t DY50_PacketAckIsValid(DY50_packet_t *packet)
{
	if(packet->header == DY50_HEADER && packet->flag == 0x07)
		return 1;

	return 0;
}

inline static void DY50_CalcCheckSum(DY50_packet_t *packet, uint16_t payload_len)
{
	uint16_t checksum  = 0;
	checksum  = packet->flag;
	checksum += packet->length & 0x00FF;         //byte1
	checksum += (packet->length >> 8) & 0x00FF;  //byte2
	checksum += packet->code;					 //command

	if(payload_len <= (PAYLOAD_TX_SIZE - 2))
	{
		for(uint16_t i=0; i< payload_len; i++)
			checksum += packet->payload[i];
	}

	//Adiciona o checksum no fim do payload;
	packet->payload[payload_len]     = (checksum >> 8) & 0x00FF;
	packet->payload[payload_len + 1] = checksum & 0x00FF;
	packet->payload[payload_len + 2] = '\0'; 		//End string (case use strlen)
}



DY50_AckCode_t DY50_SendCommand(DY50_Typedef_t *dy50, uint8_t cmd, uint16_t tx_payload_len, uint16_t rx_payload_len)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	dy50->buf_tx.packet.header      = DY50_HEADER;
	dy50->buf_tx.packet.chip_adress = DY50_ADDRESS;
	dy50->buf_tx.packet.flag        = DY50_FLAG_COMMAND;
	dy50->buf_tx.packet.code        = cmd;

	const uint8_t code_len     = 1;
	const uint8_t checksum_len = 2;
	uint8_t packet_length = code_len + tx_payload_len + checksum_len;

	dy50->buf_tx.packet.length  = SWAP16(packet_length);

	DY50_CalcCheckSum(&dy50->buf_tx.packet, tx_payload_len);

	const uint8_t size_fix_bytes = 9; //header + address + flag + length
	uint16_t size_total_bytes = size_fix_bytes + packet_length;

	if(HAL_UART_Transmit(dy50->huart, dy50->buf_tx.raw, size_total_bytes, 100) != HAL_OK)
		return ACK_ERROR_UART_TX;

	if(HAL_UART_Receive(dy50->huart, dy50->buf_rx.raw, (ACK_PACKET_SIZE + rx_payload_len), 500) != HAL_OK)
		return ACK_ERROR_UART_RX;


	if(DY50_PacketAckIsValid(&dy50->buf_rx.packet))
	{
		return dy50->buf_rx.packet.code;
	}

	return ACK_ERROR_RESPONSE;

}

DY50_AckCode_t DY50_CMD_ReadSystemParams(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_AckCode_t ack_code;

	ack_code = DY50_SendCommand(dy50, DY50_CMD_READ_SYSTEM_PARAMS, PACKET_NOT_PAYLOAD, 16);

	if(ack_code == ACK_OK)
	{
		dy50->info.database_capacity = ((uint16_t)dy50->buf_rx.packet.payload[4] << 8);
		dy50->info.database_capacity |= ((uint16_t)dy50->buf_rx.packet.payload[5] & 0x00FF);

		dy50->info.security_level = dy50->buf_rx.packet.payload[7];

		dy50->info.packet_size = dy50->buf_rx.packet.payload[13]; //0->32, 1->64, 2->128, 3->256

		dy50->info.baund_rate  = dy50->buf_rx.packet.payload[15]; //baundrate x 9600

	}

	return ack_code;
}

DY50_AckCode_t DY50_CMD_VerifyPassword(DY50_Typedef_t *dy50, uint32_t password)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	dy50->buf_tx.packet.payload[0] = (uint8_t)((password >> 24) & 0x000000FF);
	dy50->buf_tx.packet.payload[1] = (uint8_t)((password >> 16) & 0x000000FF);
	dy50->buf_tx.packet.payload[2] = (uint8_t)((password >> 8)  & 0x000000FF);
	dy50->buf_tx.packet.payload[3] = (uint8_t)(password  & 0x000000FF);
	//dy50->buf_tx.packet.payload[4] = '\0'; //End string

	return DY50_SendCommand(dy50, DY50_CMD_VERIFY_PASSWORD, 4, PACKET_NOT_PAYLOAD);
}


DY50_AckCode_t DY50_SetIndexTable(DY50_Typedef_t *dy50, uint16_t index, uint8_t value)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if(index >= dy50->info.database_capacity)
		return ACK_ERROR_INVALID_PARAMETER;

	uint8_t buffer_index = (uint8_t)(index/8);

	uint8_t bit_index = (uint8_t) (index % 8);     //map 0 to 7 value
	bit_index = (uint8_t)(0x01 << bit_index);

	if(value == 1)
		dy50->info.table_index[buffer_index] |= bit_index;
	else if(value == 0)
		dy50->info.table_index[buffer_index] &= ~bit_index;

	return ACK_OK;

}

uint8_t DY50_GetBitIndex(uint8_t byte_value, uint8_t byte_size)
{

	for(uint8_t i=0; i<byte_size; i++)
	{
		if(((byte_value >> i) & 0x01) == 0)
			return i;
	}
	return 0xFF;  //Is FULL
}

int16_t DY50_FindFirstFreeID(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	uint8_t bytes  = (dy50->info.database_capacity / 8) + 1;

	uint8_t current_byte = 0;

	while(current_byte < bytes)
	{


		uint8_t current_byte_value = dy50->info.table_index[current_byte];


		if(current_byte_value == 0xFF)	//Byte is Full
			current_byte++;
		else
		{
			uint8_t bit_index;

			if(current_byte == bytes-1)
			{
				uint8_t last_byte_size = (dy50->info.database_capacity % 8) ;
				if(last_byte_size == 0)
					last_byte_size = 8;
				bit_index = DY50_GetBitIndex(current_byte_value, last_byte_size);
			}
			else
			{
				bit_index = DY50_GetBitIndex(current_byte_value, 8);
			}

			if(bit_index != 0xFF)
			{
				uint16_t index_result = (uint16_t)(current_byte * 8) + bit_index;

				if(index_result < dy50->info.database_capacity)
					return index_result;
			}
			current_byte++;
		}

	}

	return -1;   //Erro
}

DY50_AckCode_t DY50_CMD_ReadIndexTable(DY50_Typedef_t *dy50, uint8_t readIndexTable[], uint8_t size)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if((uint16_t)(size * 8) < dy50->info.database_capacity)   //Buffer is small
		return ACK_ERROR_INVALID_PARAMETER;

	DY50_AckCode_t code_ack;
	//Get page 0
	dy50->buf_tx.packet.payload[0] = 0x00;
	code_ack = DY50_SendCommand(dy50, DY50_CMD_READ_INDEX_TABLE, 1, 32);
	memcpy(readIndexTable, dy50->buf_rx.packet.payload, 32);

	//Get page 1 (if indexs>256)
	if((code_ack == ACK_OK) && (dy50->info.database_capacity > 256))
	{
		dy50->buf_tx.packet.payload[0] = 0x01;
		code_ack = DY50_SendCommand(dy50, DY50_CMD_READ_INDEX_TABLE, 1, 32);
		memcpy(&readIndexTable[32], dy50->buf_rx.packet.payload, 32);
	}

	return code_ack;
}


DY50_AckCode_t DY50_CMD_GetImage(DY50_Typedef_t *dy50)
{
	if(dy50 == NULL)
		return	ACK_ERROR_HANDLER_NOT_DEFINED;

	return DY50_SendCommand(dy50, DY50_CMD_GET_IMAGE, PACKET_NOT_PAYLOAD, PACKET_NOT_PAYLOAD);
}

DY50_AckCode_t DY50_CMD_GenChar(DY50_Typedef_t *dy50,  DY50_BufferId_t buffer_id)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if((buffer_id < DY50_BUFFER_ID_1) && (buffer_id > DY50_BUFFER_ID_4))
		return ACK_ERROR_INVALID_PARAMETER;

	dy50->buf_tx.packet.payload[0] = buffer_id;

	return(DY50_SendCommand(dy50, DY50_CMD_GEN_CHAR, 1, PACKET_NOT_PAYLOAD));
}

DY50_AckCode_t DY50_GenerateChar(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_AckCode_t code;

	code = DY50_CMD_GetImage(dy50);
	if(code == ACK_OK)
	{
		code = DY50_CMD_GenChar(dy50, buffer_id);
	}

	return code;
}

DY50_AckCode_t DY50_CMD_RegModel(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	return(DY50_SendCommand(dy50, DY50_CMD_REG_MODEL, PACKET_NOT_PAYLOAD, PACKET_NOT_PAYLOAD));
}

DY50_AckCode_t DY50_CMD_StoreChar(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id, uint16_t page_id)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if(page_id >= dy50->info.database_capacity)
		return ACK_ERROR_INVALID_PARAMETER;

	DY50_AckCode_t ack_code;

	dy50->buf_tx.packet.payload[0] = buffer_id;
	dy50->buf_tx.packet.payload[1] = (uint8_t)((page_id >> 8) & 0x00FF);
	dy50->buf_tx.packet.payload[2] = (uint8_t)(page_id & 0x00FF);

	ack_code = DY50_SendCommand(dy50, DY50_CMD_STORE_CHAR, 3, PACKET_NOT_PAYLOAD);

	if(ack_code == ACK_OK)
	{
		DY50_SetIndexTable(dy50, page_id, 1);
	}

	return ack_code;
}

static inline uint8_t checkIfBadImageError(DY50_AckCode_t error)
{
	switch (error) {
		case ACK_ERROR_NOT_FINGER:
 		case ACK_ERROR_IMAGE_IS_TOO_LIGHT:
		case ACK_ERROR_IMAGE_IS_TOO_BLURRY:
		case ACK_ERROR_IMAGE_IS_AMORPHOUS:
		case ACK_ERROR_IMAGE_IS_TOO_LITTLE:
			return 1; //Is image error
			break;
		default:
			return 0; //Is a fatal error
			break;
	}
}

DY50_FingerTouchState_t DY50_FingerIsOnTouch(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	uint8_t touch_state = HAL_GPIO_ReadPin(dy50->touch_gpio.port, dy50->touch_gpio.pin);

	if(touch_state == 0) //Exist fingfer
	{
		if(((HAL_GetTick() - dy50->enroll.debouncing_init_time) > 200))
		{
			return DY50_FINGER_IN_TOUCH_ACCEPTED;
		}
		else
		{

			return DY50_FINGER_IN_TOUCH_WAITING_TIME;
		}
	}

	return DY50_FINGER_NOT_IN_TOUCH;
}
DY50_AckCode_t DY50_Enroll(DY50_Typedef_t *dy50)
{
	if(dy50 == NULL)
		return	ACK_ERROR_HANDLER_NOT_DEFINED;

	DY50_BufferId_t buffer_id;
	DY50_AckCode_t ack_code;

	if(dy50->enroll.last_state == DY50_ENROLL_STATE_IDLE)
	{
		dy50->enroll.last_measure_time = HAL_GetTick(); //Reset time in first reading
		dy50->enroll.table_id = DY50_FindFirstFreeID(dy50);

		if(dy50->enroll.table_id < 0)
			return ACK_ERROR_TABLE_ID_IS_FULL;
	}

	if(dy50->enroll.last_state != DY50_ENROLL_STATE_COMPLETE)
	{
		dy50->enroll.last_state++;   //Next state
	}


	if((HAL_GetTick() - dy50->enroll.last_measure_time) < 8000)
	{
		switch (dy50->enroll.last_state)
		{
			case DY50_ENROLL_STATE_IDLE:
				return ACK_ERROR_IMPOSSIBLE_STATE; //ERROR: the IDLE state is not a possible case in this state machine
			case DY50_ENROLL_STATE_WRITE_BUFFER1:
				buffer_id = DY50_BUFFER_ID_1;
				break;
			case DY50_ENROLL_STATE_WRITE_BUFFER2:
				buffer_id = DY50_BUFFER_ID_2;
				break;
			case DY50_ENROLL_STATE_WRITE_BUFFER3:
				buffer_id = DY50_BUFFER_ID_3;
				break;
			case DY50_ENROLL_STATE_WRITE_BUFFER4:
				buffer_id = DY50_BUFFER_ID_4;
				break;
			case DY50_ENROLL_STATE_COMPLETE:
				return ACK_OK;
			default:
				break;
		}

		ack_code = DY50_GenerateChar(dy50, buffer_id);
		if(ack_code == ACK_OK)
		{
			dy50->enroll.last_measure_time = HAL_GetTick();
		}
		else if(checkIfBadImageError(ack_code))
		{
			dy50->enroll.last_state--;   //Repeat current state
			dy50->enroll.last_measure_time = HAL_GetTick();
		}
		else
		{
			dy50->enroll.last_state = DY50_ENROLL_STATE_IDLE;  //FATAL ERROR
		}
	}
	else
	{
		dy50->enroll.last_state = DY50_ENROLL_STATE_IDLE;  //ERROR case timeout
		ack_code = ACK_ERROR_TIMEOUT;
	}

	return ack_code;

}



void DY50_FingerTouchInterrupt(DY50_Typedef_t *dy50)
{
	if(dy50 == NULL)
		return;

	dy50->enroll.debouncing_init_time = HAL_GetTick();
	dy50->touch_flag = 1;
}

void DY50_EnrollHandler(DY50_Typedef_t *dy50)
{
	if(dy50 == NULL)
		return;

	if(dy50->touch_flag == 1)
	{
		DY50_FingerTouchState_t finger_state = DY50_FingerIsOnTouch(dy50);


		switch (finger_state) {
			case DY50_FINGER_IN_TOUCH_ACCEPTED:
				DY50_AckCode_t ackCodeEnroll = DY50_Enroll(dy50);

				if((ackCodeEnroll == ACK_OK) && (dy50->enroll.last_state == DY50_ENROLL_STATE_COMPLETE))
				{
					ackCodeEnroll = DY50_CMD_RegModel(dy50);
					if(ackCodeEnroll == ACK_OK)
					{
						ackCodeEnroll = DY50_CMD_StoreChar(dy50, DY50_BUFFER_ID_1, dy50->enroll.table_id);

						if(ackCodeEnroll != ACK_OK)
							dy50->enroll.last_state = DY50_ENROLL_STATE_IDLE;  //StoreChar Error
					}
					else
					{
					  dy50->enroll.last_state = DY50_ENROLL_STATE_IDLE;			//RegModel Error
					}
				}

				DY50_EnrolResponseCallBack(dy50, ackCodeEnroll);

				if(dy50->enroll.last_state == DY50_ENROLL_STATE_COMPLETE)
					dy50->enroll.last_state = DY50_ENROLL_STATE_IDLE;

				dy50->touch_flag = 0;

				break;
			case DY50_FINGER_IN_TOUCH_WAITING_TIME:
				//Waiting a debounce time
				break;
			case DY50_FINGER_NOT_IN_TOUCH:
				dy50->touch_flag = 0;
				break;
			default:
				break;
		}

	}
}

__weak void DY50_EnrolResponseCallBack(DY50_Typedef_t *dy50, DY50_AckCode_t ackCode)
{

	switch (ackCode) {
		case ACK_OK:
			switch (dy50->enroll.last_state)
			{
			  case DY50_ENROLL_STATE_WRITE_BUFFER1:
			  case DY50_ENROLL_STATE_WRITE_BUFFER2:
			  case DY50_ENROLL_STATE_WRITE_BUFFER3:
			  case DY50_ENROLL_STATE_WRITE_BUFFER4:

				  //Event case write buffer

				  break;

			  case DY50_ENROLL_STATE_COMPLETE:

				  //Event case write buffer and RegModel ok
				  break;

			  case DY50_ENROLL_STATE_IDLE:
				  //Fatal ERROR, write buffer ok, but RegModel Fail
				  break;
			  default:
				  break;

			}
			break;
		case ACK_ERROR_NOT_FINGER:
			//ERROR not finger in module
			break;
		case ACK_ERROR_IMAGE_IS_AMORPHOUS:
			//ERROR bad image (too amorphous)
			break;
		case ACK_ERROR_IMAGE_IS_TOO_LITTLE:
			//ERROR bad image (too little)
			break;
		default:
			break;
	}

}
















