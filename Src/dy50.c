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


DY50_Typedef_t *dy50_global; //Temporary for test


//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == dy50_global->huart)
	{
		//dy50_global->
		dy50_global->uart.dma_flag = 1;
		//HAL_UARTEx_ReceiveToIdle_DMA(dy50_global->huart, dy50_global->uart.buf_rx.raw, PAYLOAD_RX_SIZE + 9);
	}

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == dy50_global->huart)
    {
        __HAL_UART_CLEAR_FEFLAG(dy50_global->huart);
        __HAL_UART_CLEAR_NEFLAG(dy50_global->huart);
        __HAL_UART_CLEAR_OREFLAG(dy50_global->huart);

        //HAL_UARTEx_ReceiveToIdle_DMA(dy50_global->huart, dy50_global->uart.buf_rx.raw, PAYLOAD_RX_SIZE + 9);
    }
}



/**
 * @brief DY50 Initialization function
 *
 * @note This is an important function that should be called before all others
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param huart Is a pointer for UART Handler
 * @param touch_gpio_port Is a pointer for touch gpio port
 * @param touch_gpio_port Is the touch gpio pin
 *
 * @retval initialization state code
 */
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
				dy50->uart.buf_tx.packet.header      = DY50_HEADER;
				dy50->uart.buf_tx.packet.chip_adress = DY50_ADDRESS;

				dy50->touch_gpio.port = touch_gpio_port;
				dy50->touch_gpio.pin  = touch_gpio_pin;

				dy50->status = DY50_STATUS_IDLE;  //For uses readsystem and verify password functions

		        __HAL_UART_CLEAR_FEFLAG(dy50->huart);
		        __HAL_UART_CLEAR_NEFLAG(dy50->huart);
		        __HAL_UART_CLEAR_OREFLAG(dy50->huart);

				dy50_global = dy50;
				//HAL_UARTEx_ReceiveToIdle_DMA(dy50->huart, dy50->uart.buf_rx.raw, PAYLOAD_RX_SIZE);

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

/*
 * @brief Check if received packete is valid
 *
 * @note This function receive a DY50_packet_t and verify if is valid,
 * 		 According to the datasheet, a packet is valid if the response flag is 0x07
 *
 * @param packet is the response packet that will be verified
 *
 * @retval return 1 if packet is valid
 */
inline static uint8_t DY50_PacketAckIsValid(DY50_packet_t *packet)
{
	if(packet->header == DY50_HEADER && packet->flag == 0x07)
		return 1;

	return 0;
}

/*
 * @brief Calculate a checksum for package
 *
 * @note This function receive a DY50_packet_t and calculate your checksum
 * 	     The calculation is based on the Datasheet, the formula consists of
 * 	     summing all all the bytes after the packet flag
 *
 * @param packet is the response packet whose checksum we will calculate
 * @param payload_len is the payload size
 *
 * @retval none
 */
static inline void DY50_CalcCheckSum(DY50_packet_t *packet, uint16_t payload_len)
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


DY50_AckCode_t DY50_SendCommand_DMA(DY50_Typedef_t *dy50, DY50_Commands_t cmd, uint16_t tx_payload_len, uint16_t rx_payload_len)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	dy50->uart.buf_tx.packet.header      = DY50_HEADER;
	dy50->uart.buf_tx.packet.chip_adress = DY50_ADDRESS;
	dy50->uart.buf_tx.packet.flag        = DY50_FLAG_COMMAND;
	dy50->uart.buf_tx.packet.code        = cmd;

	const uint8_t code_len     = 1;
	const uint8_t checksum_len = 2;
	uint8_t packet_length = code_len + tx_payload_len + checksum_len;

	dy50->uart.buf_tx.packet.length  = SWAP16(packet_length);

	DY50_CalcCheckSum(&dy50->uart.buf_tx.packet, tx_payload_len);

	const uint8_t size_fix_bytes = 9; //header + address + flag + length
	uint16_t size_total_bytes = size_fix_bytes + packet_length;

	dy50->uart.dma_flag = 0;
	__HAL_UART_CLEAR_OREFLAG(dy50->huart);

	HAL_UART_Receive_DMA(dy50->huart, dy50->uart.buf_rx.raw, (ACK_PACKET_SIZE + rx_payload_len));

	if(HAL_UART_Transmit(dy50->huart, dy50->uart.buf_tx.raw, size_total_bytes, 100) != HAL_OK)
		return ACK_ERROR_UART_TX;

	return ACK_OK;
}

DY50_AckCode_t DY50_Wait_Command_Response(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if(dy50->uart.dma_flag == 1)
	{
		if(DY50_PacketAckIsValid(&dy50->uart.buf_rx.packet))
		{
			return dy50->uart.buf_rx.packet.code;
		}

		dy50->uart.dma_flag = 0;
		return ACK_ERROR_RESPONSE;
	}

	return ACK_WATING_RESPONSE;
}

DY50_AckCode_t DY50_Wait_Command_Response_Block(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_AckCode_t ack_response = DY50_Wait_Command_Response(dy50);
	while(ack_response == ACK_WATING_RESPONSE)
	{
		ack_response = DY50_Wait_Command_Response(dy50);
		//Block code waiting response
	}

	return ack_response;

}

/*
 * @brief The command sending function
 *
 * @note This function is the basis for sending all commands,
 * 		 an important point is tx_package and rx_package.
 *
 * 		 Use dy50 handler to get the response to a command:
 *
 * 		 dy50->buf_tx.packet -> contains all bytes of transmit
 * 		 dy50->buf_rx.packet -> contains all received bytes
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param cmd   The command to be sent
 * @param tx_payload_len It is the payload capacity of the transmission
 * @param rx_payload_len It is the payload capacity of the receiving
 *
 * @retval Returns whether the command was successful
 *
 */
DY50_AckCode_t DY50_SendCommand(DY50_Typedef_t *dy50, DY50_Commands_t cmd, uint16_t tx_payload_len, uint16_t rx_payload_len)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_SendCommand_DMA(dy50, cmd, tx_payload_len, rx_payload_len);
	return DY50_Wait_Command_Response_Block(dy50);

//	dy50->uart.buf_tx.packet.header      = DY50_HEADER;
//	dy50->uart.buf_tx.packet.chip_adress = DY50_ADDRESS;
//	dy50->uart.buf_tx.packet.flag        = DY50_FLAG_COMMAND;
//	dy50->uart.buf_tx.packet.code        = cmd;
//
//	const uint8_t code_len     = 1;
//	const uint8_t checksum_len = 2;
//	uint8_t packet_length = code_len + tx_payload_len + checksum_len;
//
//	dy50->uart.buf_tx.packet.length  = SWAP16(packet_length);
//
//	DY50_CalcCheckSum(&dy50->uart.buf_tx.packet, tx_payload_len);
//
//	const uint8_t size_fix_bytes = 9; //header + address + flag + length
//	uint16_t size_total_bytes = size_fix_bytes + packet_length;
//
//	if(HAL_UART_Transmit(dy50->huart, dy50->uart.buf_tx.raw, size_total_bytes, 100) != HAL_OK)
//		return ACK_ERROR_UART_TX;
//
//	if(HAL_UART_Receive(dy50->huart, dy50->uart.buf_rx.raw, (ACK_PACKET_SIZE + rx_payload_len), 500) != HAL_OK)
//		return ACK_ERROR_UART_RX;
//
//
//	if(DY50_PacketAckIsValid(&dy50->uart.buf_rx.packet))
//	{
//		return dy50->uart.buf_rx.packet.code;
//	}
//
//	return ACK_ERROR_RESPONSE;

}

/*
 * @brief Reads and Stores system parameters
 *
 * @note Use the "ReadSysPara" command to obtain relevant system information
 *
 * @param dy50  Is a pointer for dy50 handler
 *
 * @retval Returns whether it was possible to obtain the data
 */
DY50_AckCode_t DY50_CMD_ReadSystemParams(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_AckCode_t ack_code;

	ack_code = DY50_SendCommand(dy50, DY50_CMD_READ_SYSTEM_PARAMS, PACKET_NOT_PAYLOAD, 16);

	if(ack_code == ACK_OK)
	{
		dy50->info.database_capacity = ((uint16_t)dy50->uart.buf_rx.packet.payload[4] << 8);
		dy50->info.database_capacity |= ((uint16_t)dy50->uart.buf_rx.packet.payload[5] & 0x00FF);

		dy50->info.security_level = dy50->uart.buf_rx.packet.payload[7];

		dy50->info.packet_size = dy50->uart.buf_rx.packet.payload[13]; //0->32, 1->64, 2->128, 3->256

		dy50->info.baund_rate  = dy50->uart.buf_rx.packet.payload[15]; //baundrate x 9600

	}

	return ack_code;
}

/*
 * @brief Check password
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param password this is the password we will use
 *
 * @retval Returns whether the password was verified
 */
DY50_AckCode_t DY50_CMD_VerifyPassword(DY50_Typedef_t *dy50, uint32_t password)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	dy50->uart.buf_tx.packet.payload[0] = (uint8_t)((password >> 24) & 0x000000FF);
	dy50->uart.buf_tx.packet.payload[1] = (uint8_t)((password >> 16) & 0x000000FF);
	dy50->uart.buf_tx.packet.payload[2] = (uint8_t)((password >> 8)  & 0x000000FF);
	dy50->uart.buf_tx.packet.payload[3] = (uint8_t)(password  & 0x000000FF);

	return DY50_SendCommand(dy50, DY50_CMD_VERIFY_PASSWORD, 4, PACKET_NOT_PAYLOAD);
}


/*
 * @brief Estimates what the last filled ID was
 *
 * @note An important function that estimates the last known ID,
 *       allowing for optimized fingerprint searches in the database.
 *
 *       IMPORTANT: The returned value is not the last true ID,
 *       but rather an estimate, always rounded up
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param start_id is is the initial ID where we intend to begin the search
 * @param end_id is the final ID, meaning it limits how far we want to search for the last ID
 */
int16_t DY50_FindLastIndexFilled(DY50_Typedef_t *dy50, uint16_t start_id, uint16_t end_id)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return -1;

	uint8_t current_byte = start_id / 8;
	uint8_t end_byte   = end_id / 8;

	int8_t last_byte_id = -1;

	while(current_byte <= end_byte)
	{
		if(dy50->info.table_index[current_byte] != 0x00 )
		{
			last_byte_id = current_byte;
		}
		current_byte++;
	}

	if(last_byte_id == -1)
		return start_id;

	uint16_t estimated_id = ((last_byte_id + 1) * 8);

	if(estimated_id >= dy50->info.database_capacity)
		estimated_id = dy50->info.database_capacity;

	return estimated_id;
}

/*
 * @brief Sets an ID in Table Index
 *
 * @note Set an ID as marked in the Index table
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param index is the index we want to mark
 * @param value use 1 to select the ID and 0 to deselect the ID
 *
 * @retval Returns whether it was possible to set the value in the ID
 */
DY50_AckCode_t DY50_SetIndexTable(DY50_Typedef_t *dy50, uint16_t index, uint8_t value)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if((index >= dy50->info.database_capacity) || (value > 1))
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

/*
 * @brief Receives a byte and finds a first free bit (0)
 *
 * @param byte_value is a target byte
 * @param byte_size the number of valid bits in the byte
 *
 * @retval return the bit index, if it exists, or returns 0xFF if the byte is full
 */
static inline uint8_t DY50_GetBitIndex(uint8_t byte_value, uint8_t byte_size)
{

	for(uint8_t i=0; i<byte_size; i++)
	{
		if(((byte_value >> i) & 0x01) == 0)
			return i;
	}
	return 0xFF;  //Is FULL
}

/*
 * @brief Find the first free ID in Index Table
 *
 * @param dy50  Is a pointer for dy50 handler
 *
 * @retval returns the first index found, or -1 in case of error or full table
 */
int16_t DY50_FindFirstFreeIDInIndexTable(DY50_Typedef_t *dy50)
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


/*
 * @brief Reading the Index Table on dy50
 *
 * @note According to the data sheet, Index Table it contains all the IDs for DY50, the Table
 * 		 is divided into 2 pages, where each page has 32 bytes.
 * 		 The actual size of the table is defined for dy50->info.database_capacity
 *
 * 		 The readIndexTable[] received all ids, each bit represents one ID, where:
 *
 * 		 bit == 0 -> ID position is empty
 * 		 bit == 1 -> ID position is filled
 *
 * 	@param dy50  Is a pointer for dy50 handler
 * 	@param readIndexTable[] the array that receives all the IDs
 * 	@param size is a size of readIndexTable[], The size determines how much data will be returned
 *
 * 	@retval returns whether it was possible to read the index table
 *
 */
DY50_AckCode_t DY50_CMD_ReadIndexTable(DY50_Typedef_t *dy50, uint8_t readIndexTable[], uint8_t size)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	const uint8_t page_size = 32; //based on the datasheet, each page is 32 bytes
	const uint8_t tx_payload_size = 1;

	DY50_AckCode_t code_ack;

	//Get page 0
	dy50->uart.buf_tx.packet.payload[0] = 0x00;
	code_ack = DY50_SendCommand(dy50, DY50_CMD_READ_INDEX_TABLE, tx_payload_size, page_size);

	if(size >= page_size)
		memcpy(readIndexTable, dy50->uart.buf_rx.packet.payload, page_size);
	else
		memcpy(readIndexTable, dy50->uart.buf_rx.packet.payload, size);

	//Get page 1 (if indexs>256)
	if((code_ack == ACK_OK) && (dy50->info.database_capacity > 256))
	{
		if(size > page_size)
		{
			dy50->uart.buf_tx.packet.payload[0] = 0x01;
			code_ack = DY50_SendCommand(dy50, DY50_CMD_READ_INDEX_TABLE, tx_payload_size, page_size);

			uint8_t remaining_bytes = (size - 32);

			if(remaining_bytes >= page_size)
				memcpy(&readIndexTable[32], dy50->uart.buf_rx.packet.payload, page_size);
			else
				memcpy(&readIndexTable[32], dy50->uart.buf_rx.packet.payload, remaining_bytes);
		}
	}

	return code_ack;
}

/*
 * @brief Command Get Image (0x01)
 *
 * @note This command reads the fingerprint from sensor and generates the image.
 * 		 IMPORTANT: The image is storage in image buffer, then, the GenChar command must be used
 *
 * @param dy50  Is a pointer for dy50 handler
 *
 * @retval return whether it was possible to generate the image
 */
DY50_AckCode_t DY50_CMD_GetImage(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	return DY50_SendCommand(dy50, DY50_CMD_GET_IMAGE, PACKET_NOT_PAYLOAD, PACKET_NOT_PAYLOAD);
}

DY50_AckCode_t DY50_CMD_GenChar(DY50_Typedef_t *dy50,  DY50_BufferId_t buffer_id)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if((buffer_id < DY50_BUFFER_ID_1) && (buffer_id > DY50_BUFFER_ID_4))
		return ACK_ERROR_INVALID_PARAMETER;

	dy50->uart.buf_tx.packet.payload[0] = buffer_id;

	return(DY50_SendCommand(dy50, DY50_CMD_GEN_CHAR, 1, PACKET_NOT_PAYLOAD));
}

/*
 * @brief Command Generate Char (0x02)
 *
 * @note Generating Fingerprint image generation based on the original image
 * 		 IMPORTANT: The command Get Image is called before Generate Char
 *
 * @note The Generate Char stores in a temporary buffer, DY50 has 4 buffer:
 *	     1 - CharBuffer1
 *	     2 - CharBuffer2
 *	     3 - CharBuffer3
 *	     4 - CharBuffer4
*	     Each buffer stores a fingerprint image
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param buffer_id define in wich CharBuffer the fingerprint will be stored
 *
 * @retval returns whether the image was generated successfully
 */
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

/*
 * @brief Command RegModel (0x05)
 *
 * @note Generates a template based on CharBuffers, this command merges all Genchars to generate the template.
 * 		 IMPORTANT: All character buffers must be correctly populated, therefore, before using RegModel, use the Generate Char Command for each ID
 *
 * @param dy50  Is a pointer for dy50 handler
 *
 * @retval returns whether the ChaBuffers merges and template generation were sucessfull
 */
DY50_AckCode_t DY50_CMD_RegModel(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	return(DY50_SendCommand(dy50, DY50_CMD_REG_MODEL, PACKET_NOT_PAYLOAD, PACKET_NOT_PAYLOAD));
}

/*
 * @brief Command StoreChar (0x06)
 *
 * @note Stores the model generated by the RegModel command in the DY50 flash database
 *
 * @param buffer_id  the buffer where the template generated by RegModel is located (by default it is always in CharBuffer1)
 * @param page_id    This is the ID that the template will be stored under in the DY50 flash database
 *
 * @retval Returns whether it was possible to store the template in the DY50 flash database
 *
 * @warning If page_id already exists in Flash, it will be replaced without prior notice
 */
DY50_AckCode_t DY50_CMD_StoreChar(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id, uint16_t page_id)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if(page_id >= dy50->info.database_capacity)
		return ACK_ERROR_INVALID_PARAMETER;

	DY50_AckCode_t ack_code;

	dy50->uart.buf_tx.packet.payload[0] = buffer_id;
	dy50->uart.buf_tx.packet.payload[1] = (uint8_t)((page_id >> 8) & 0x00FF);
	dy50->uart.buf_tx.packet.payload[2] = (uint8_t)(page_id & 0x00FF);

	ack_code = DY50_SendCommand(dy50, DY50_CMD_STORE_CHAR, 3, PACKET_NOT_PAYLOAD);

	if(ack_code == ACK_OK)
	{
		DY50_SetIndexTable(dy50, page_id, 1);
	}

	return ack_code;
}

/*
 * @brief Check if the error message indicates a defective image
 *
 * @note In general, it means that it was possible to generate the image, but the quality was poor
 *
 * @param error receives an ack code
 *
 * @retval returns whether it's a bad image error
 *
 * @return 1 case bad image error or 0 case other error
 */
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


/*
 * @brief Check if there is a finger on the DY50 touch
 *
 * @note The function uses TOUCH_DEBOUNCE_TIME for define debounce time, this prevents the reading from being taken
 *       before the finger is correctly positioned. By default, the debounce time is set to 200ms, but you can change that value
 *
 * @param dy50  Is a pointer for dy50 handler
 *
 * @retval Returns whether there is a finger on the sensor
 *
 * @warning Please note that for the function to work correctly, the DY50's touch screen must be properly configured
 */
DY50_FingerTouchState_t DY50_FingerIsOnTouch(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	uint8_t touch_state = HAL_GPIO_ReadPin(dy50->touch_gpio.port, dy50->touch_gpio.pin);

	if(touch_state == 0) //Exist fingfer
	{
		if(((HAL_GetTick() - dy50->enroll.debouncing_init_time) > TOUCH_DEBOUNCE_TIME))
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


/*
 * @brif A complete Enroll Flux
 *
 * @note This code is based on the "Enroll Flow" for the datasheet, more details read the datasheet
 *
 * @warning The enroll has a maximum time limit between each reading, this time is defined for ENROLL_MAX_TIME_IDLE_BETWEEN_READING,
 * 		    If the time limit expires, all readings will be discarded and the registration will be reset
 *
 * @param dy50  Is a pointer for dy50 handler
 *
 * @retval Returns whether the enroll flow was successful
 *
 */
DY50_AckCode_t DY50_Enroll(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_BufferId_t buffer_id;
	DY50_AckCode_t ack_code;

	if(dy50->enroll.last_state == DY50_ENROLL_STATE_IDLE)
	{
		dy50->enroll.last_measure_time = HAL_GetTick(); //Reset time in first reading
		dy50->enroll.table_id = DY50_FindFirstFreeIDInIndexTable(dy50);

		if(dy50->enroll.table_id < 0)
			return ACK_ERROR_TABLE_ID_IS_FULL;
	}

	if(dy50->enroll.last_state != DY50_ENROLL_STATE_COMPLETE)
	{
		dy50->enroll.last_state++;   //Next state
	}


	if((HAL_GetTick() - dy50->enroll.last_measure_time) < ENROLL_MAX_TIME_IDLE_BETWEEN_READING)
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


/*
 * @brief Interrupt that monitors the touch of the dy50
 *
 * @note This function needs to be inside the callback function that handles the GPIO interrupt for the touch
 *
 * @param dy50  Is a pointer for dy50 handler
 *
 * @retval none
 */
void DY50_FingerTouchInterrupt(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return;

	if(dy50->touch_flag == 0)
	{
		dy50->enroll.debouncing_init_time = HAL_GetTick();
		dy50->touch_flag = 1;
	}
}

/*
 * @brief Non-blocking function that manages the Enroll flow
 *
 * @warning This function should be used in conjunction with the callback function of enroll
 *
 * @param dy50  Is a pointer for dy50 handler
 *
 * @retval none
 */
void DY50_EnrollHandler(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
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

/*
 * @brief Enroll Callback
 *
 * @note Weak function example, you must implement the strong function that overrides it
 *
 * @warning This function is called by the DY50 Enroll Handler
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param ackCode use to check the status of the last command
 */
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

/*
 * @brief Command Search (0x04)
 *
 * @note Search for a fingerprint in the DY50 flash database. Use this function to check if the fingerprint image
 *       we just read exists in the DY50 database.
 *
 * @param buffer_id is the CharBufferID where the image is located (usually it will be 1)
 * @param start_page_id  This refers to the page ID in the Flash database where the search begins
 * @param page_num Defines how many page IDs we will read after the start page
 *
 * @retval return if the fingerprint we were looking for was found
 *
 * @return 0x00 -> Successfull or 0x09 -> fingerprint not found
 */
DY50_AckCode_t DY50_CMD_Search(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id, uint16_t start_page_id, uint16_t page_num)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	dy50->uart.buf_tx.packet.payload[0] = buffer_id;
	dy50->uart.buf_tx.packet.payload[1] = (uint8_t)((start_page_id >> 8) & 0x00FF);
	dy50->uart.buf_tx.packet.payload[2] = (uint8_t)(start_page_id & 0x00FF);
	dy50->uart.buf_tx.packet.payload[3] = (uint8_t)((page_num >> 8) & 0x00FF);
	dy50->uart.buf_tx.packet.payload[4] = (uint8_t)(page_num & 0x00FF);

	const uint8_t payload_tx_len = 5,
		          payload_rx_len = 4;

	return DY50_SendCommand_DMA(dy50, Dy50_CMD_SEARCH, payload_tx_len, payload_rx_len);
}

/*
 * @brief Checks if the figerprint exists in the database
 *
 * @note It takes the obtained fingerprint and compares it with the fingerprints stored in the database
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param id_found this is the ID found by the search. If it is 0xFFFF, no match was found
 * @param math_score Returns a value from 0 to 255 indicating how closely the fingerprint matches the one found in the database
 *
 * @retval Returns whether the search was successful
 */
DY50_AckCode_t DY50_SearchFingerPrint(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_AckCode_t ack_code = ACK_OK;

	switch (dy50->search_state)
	{
		case DY50_SEARCH_STATE_IDLE:

			if((HAL_GetTick() - dy50->search_last_measuere_time) > SEARCH_MIN_TIME_BETWEEN_READING)
			{
				dy50->search_last_measuere_time = HAL_GetTick();
				dy50->search_state = DY50_SEARCH_STATE_CMD_SEARCH;
			}
			else
				ack_code = ACK_OK;	//There are no errors, just waiting
			break;

		case DY50_SEARCH_STATE_CMD_SEARCH:

			if(HAL_GPIO_ReadPin(dy50->touch_gpio.port, dy50->touch_gpio.pin) == 0)
			{
				int16_t last_id = DY50_FindLastIndexFilled(dy50, 0, (dy50->info.database_capacity - 1));

				if(last_id <= 0)
				{
					ack_code = ACK_ERROR_FINGERPRINT_NOT_FOUND;
				}
				else
				{
					ack_code = DY50_GenerateChar(dy50, DY50_BUFFER_ID_1);
					if(ack_code == ACK_OK)
					{
						ack_code = DY50_CMD_Search(dy50, DY50_BUFFER_ID_1, 0, last_id);
						if(ack_code == ACK_OK)
							dy50->search_state = DY50_SEARCH_STATE_WAITING_RESPONSE;
					}
					else
					{
						dy50->search_state = DY50_SEARCH_STATE_IDLE;
					}
				}
			}
			else
				ack_code = ACK_ERROR;

			break;

		case DY50_SEARCH_STATE_WAITING_RESPONSE:

			uint8_t callback_is_valid = 1;
			DY50_Search_Return_t search_return = {.id_found = 0xFFFF, .math_score = 0};
			dy50->search_state = DY50_SEARCH_STATE_COMPLETED;    //It only remains complete if it is not expecting a response

			ack_code = DY50_Wait_Command_Response(dy50);

			switch (ack_code)
			{
				case ACK_WATING_RESPONSE:

					dy50->search_state = DY50_SEARCH_STATE_WAITING_RESPONSE;
					callback_is_valid = 0;
					break;

				case ACK_OK:

					search_return.id_found =  ((uint16_t)dy50->uart.buf_rx.packet.payload[0]) << 8;
					search_return.id_found |= (((uint16_t)dy50->uart.buf_rx.packet.payload[1]) & 0x00FF);

					search_return.math_score = dy50->uart.buf_rx.packet.payload[3];

					callback_is_valid = 1;
					break;

				case ACK_ERROR_FINGERPRINT_NOT_FOUND:

					callback_is_valid = 1;

				default:
					callback_is_valid = 0;
					break;
			}

			if(callback_is_valid)
				DY50_SearchResponseCallBack(dy50, &search_return);

			break;
		default:
			break;
	}

	if(dy50->search_state == DY50_SEARCH_STATE_COMPLETED)
		dy50->search_state = DY50_SEARCH_STATE_IDLE;

	dy50->touch_flag = 0;	//We didn't use the flag, but it still needs to be reset to zero

	return ack_code;

}

__weak void DY50_SearchResponseCallBack(DY50_Typedef_t *dy50, const DY50_Search_Return_t *search_return)
{

}


void DY50_TaskHandler(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return;


	switch (dy50->status)
	{
		case DY50_STATUS_ENROLL_HANDLER:
			DY50_EnrollHandler(dy50);
			break;
		case DY50_STATUS_SEARCH_FINGERPRINT:
			DY50_SearchFingerPrint(dy50);
			break;
		default:
			dy50->touch_flag = 0;
			break;
	}

}









