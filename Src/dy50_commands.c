/*
 * dy50_commands.c
 *
 *  Created on: Jun 1, 2026
 *      Author: kenny
 */

#include "dy50_commands.h"

#include "string.h"

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

/*
 * @brief Send a command for DY50 and config response trigger
 *
 * @note This is a non-blocking solution for sending a command to DY50
 *
 * @warning This function does not return a response, only whether the command was sent,
 * 			for this purpose, use the SendCommandResponse_DMA function
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param cmd   The command to be sent
 * @param tx_payload_len It is the payload capacity of the transmission
 * @param rx_payload_len It is the payload capacity of the receiving
 *
 * @return whether the command was SENT successfully
 */
DY50_AckCode_t DY50_SendCommand_DMA(DY50_Typedef_t *dy50, DY50_Commands_t cmd, uint16_t tx_payload_len, uint16_t rx_payload_len)
{
//	if(dy50->status == DY50_STATUS_UNINITIALIZED)
//		return ACK_ERROR_DY50_UNINITIALIZED;
	if(dy50 == NULL)
		return ACK_ERROR_HANDLER_NOT_DEFINED;
	if(dy50->huart == NULL)
		return ACK_ERROR_UART_NOT_DEFINED;


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

	dy50->response_time = HAL_GetTick();

	return ACK_OK;
}

/*
 * @brief Awaiting a response from a command
 *
 * @note This is a non-blocking version for waiting for a command response (DMA only).
 * 		 The basis is the dma_flag, which is set when a touch interruption occurs.
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param timeout_response Set a maximum waiting time for the response
 *
 * @return whether command response is Valid
 */
DY50_AckCode_t DY50_WaitCommandResponse(DY50_Typedef_t *dy50, uint32_t timeout_response)
{
//	if(dy50->status == DY50_STATUS_UNINITIALIZED)
//		return ACK_ERROR_DY50_UNINITIALIZED;
	if(dy50 == NULL)
		return ACK_ERROR_HANDLER_NOT_DEFINED;
	if(dy50->huart == NULL)
		return ACK_ERROR_UART_NOT_DEFINED;


	if((HAL_GetTick() - dy50->response_time) > timeout_response)
		return ACK_ERROR_TIMEOUT;

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

/*
 * @brief Awaiting a response from a command (Block code)
 *
 * @note This is a blocking version for waiting for a command response,
 * 	     used when it is necessary to block the code until the response arrives
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param timeout_response Set a maximum waiting time for the response
 *
 * @return whether command response is Valid
 */
DY50_AckCode_t DY50_WaitCommandResponseBlock(DY50_Typedef_t *dy50, uint32_t timeout_response)
{
//	if(dy50->status == DY50_STATUS_UNINITIALIZED)
//		return ACK_ERROR_DY50_UNINITIALIZED;

	if(dy50 == NULL)
		return ACK_ERROR_HANDLER_NOT_DEFINED;
	if(dy50->huart == NULL)
		return ACK_ERROR_UART_NOT_DEFINED;


	DY50_AckCode_t ack_response = DY50_WaitCommandResponse(dy50, timeout_response);
	while(ack_response == ACK_WATING_RESPONSE)
	{
		ack_response = DY50_WaitCommandResponse(dy50, timeout_response);
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
 * @param timeout_response Set a maximum waiting time for the response
 *
 * @returns whether the command was successful
 *
 */
DY50_AckCode_t DY50_SendCommand(DY50_Typedef_t *dy50, DY50_Commands_t cmd, uint16_t tx_payload_len, uint16_t rx_payload_len, uint32_t timeout_response)
{
//	if(dy50->status == DY50_STATUS_UNINITIALIZED)
//		return ACK_ERROR_DY50_UNINITIALIZED;
	if(dy50 == NULL)
		return ACK_ERROR_HANDLER_NOT_DEFINED;
	if(dy50->huart == NULL)
		return ACK_ERROR_UART_NOT_DEFINED;

	DY50_SendCommand_DMA(dy50, cmd, tx_payload_len, rx_payload_len);
	return DY50_WaitCommandResponseBlock(dy50, 1000);
}

/*
 * @brief Sending a command and waiting for a response (without blocking the code)
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param cmd   The command to be sent
 * @param tx_payload_len It is the payload capacity of the transmission
 * @param rx_payload_len It is the payload capacity of the receiving
 * @param timeout_response Set a maximum waiting time for the response
 */
DY50_AckCode_t DY50_SendCommandResponse_DMA(DY50_Typedef_t *dy50, DY50_Commands_t cmd, uint16_t tx_payload_len, uint16_t rx_payload_len, uint32_t timeout_response)
{
//	if(dy50->status == DY50_STATUS_UNINITIALIZED)
//		return ACK_ERROR_DY50_UNINITIALIZED;

	if(dy50 == NULL)
		return ACK_ERROR_HANDLER_NOT_DEFINED;
	if(dy50->huart == NULL)
		return ACK_ERROR_UART_NOT_DEFINED;


	DY50_AckCode_t ack_code;

	switch (dy50->cmd_dma_status)
	{
		case DY50_CMD_DMA_STATUS_IDLE:
		case DY50_CMD_DMA_STATUS_SEND:

			dy50->uart.dma_flag = 0;

			ack_code = DY50_SendCommand_DMA(dy50, cmd, tx_payload_len, rx_payload_len);

			if(ack_code == ACK_OK)
			{
				dy50->cmd_dma_status = DY50_CMD_DMA_STATUS_WAITING;
				ack_code = ACK_WATING_RESPONSE;
			}
			else
				dy50->cmd_dma_status = DY50_CMD_DMA_STATUS_IDLE;
			break;

		case DY50_CMD_DMA_STATUS_WAITING:

			ack_code = DY50_WaitCommandResponse(dy50, timeout_response);

			switch (ack_code)
			{
				case ACK_WATING_RESPONSE:
					dy50->cmd_dma_status = DY50_CMD_DMA_STATUS_WAITING;
					break;
				case ACK_OK:
					dy50->cmd_dma_status = DY50_CMD_DMA_STATUS_IDLE;
					break;
				default:
					dy50->cmd_dma_status = DY50_CMD_DMA_STATUS_IDLE;
					break;
			}

			break;
		default:
			break;
	}

	return ack_code;
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
//	if(dy50->status == DY50_STATUS_UNINITIALIZED)
//		return ACK_ERROR_DY50_UNINITIALIZED;

	if(dy50 == NULL)
		return ACK_ERROR_HANDLER_NOT_DEFINED;
	if(dy50->huart == NULL)
		return ACK_ERROR_UART_NOT_DEFINED;


	DY50_AckCode_t ack_code;

	ack_code = DY50_SendCommand(dy50, DY50_CMD_READ_SYSTEM_PARAMS, PACKET_NOT_PAYLOAD, 16, 100);

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
//	if(dy50->status == DY50_STATUS_UNINITIALIZED)
//		return ACK_ERROR_DY50_UNINITIALIZED;

	if(dy50 == NULL)
		return ACK_ERROR_HANDLER_NOT_DEFINED;
	if(dy50->huart == NULL)
		return ACK_ERROR_UART_NOT_DEFINED;

	dy50->uart.buf_tx.packet.payload[0] = (uint8_t)((password >> 24) & 0x000000FF);
	dy50->uart.buf_tx.packet.payload[1] = (uint8_t)((password >> 16) & 0x000000FF);
	dy50->uart.buf_tx.packet.payload[2] = (uint8_t)((password >> 8)  & 0x000000FF);
	dy50->uart.buf_tx.packet.payload[3] = (uint8_t)(password  & 0x000000FF);

	return DY50_SendCommand(dy50, DY50_CMD_VERIFY_PASSWORD, 4, PACKET_NOT_PAYLOAD, 100);
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
 * 	@returns whether it was possible to read the index table
 *
 */
DY50_AckCode_t DY50_CMD_ReadIndexTable(DY50_Typedef_t *dy50, uint8_t readIndexTable[], uint8_t size)
{
//	if(dy50->status == DY50_STATUS_UNINITIALIZED)
//		return ACK_ERROR_DY50_UNINITIALIZED;

	if(dy50 == NULL)
		return ACK_ERROR_HANDLER_NOT_DEFINED;
	if(dy50->huart == NULL)
		return ACK_ERROR_UART_NOT_DEFINED;

	const uint8_t page_size = 32; //based on the datasheet, each page is 32 bytes
	const uint8_t tx_payload_size = 1;

	DY50_AckCode_t code_ack;

	//Get page 0
	dy50->uart.buf_tx.packet.payload[0] = 0x00;
	code_ack = DY50_SendCommand(dy50, DY50_CMD_READ_INDEX_TABLE, tx_payload_size, page_size, 200);

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
			code_ack = DY50_SendCommand(dy50, DY50_CMD_READ_INDEX_TABLE, tx_payload_size, page_size, 200);

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
 * @return whether it was possible to generate the image
 */
DY50_AckCode_t DY50_CMD_GetImage(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	return DY50_SendCommand(dy50, DY50_CMD_GET_IMAGE, PACKET_NOT_PAYLOAD, PACKET_NOT_PAYLOAD, 500);
}

/*
 * @brief Command Get Image (0x01) (DMA without blocking version)
 *
 * @note This command reads the fingerprint from sensor and generates the image.
 * 		 IMPORTANT: The image is storage in image buffer, then, the GenChar command must be used
 *
 * @param dy50  Is a pointer for dy50 handler
 *
 * @return whether it was possible to generate the image
 */
DY50_AckCode_t DY50_CMD_GetImageDMA(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	return DY50_SendCommandResponse_DMA(dy50, DY50_CMD_GET_IMAGE, PACKET_NOT_PAYLOAD, PACKET_NOT_PAYLOAD, 500);
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
DY50_AckCode_t DY50_CMD_GenChar(DY50_Typedef_t *dy50,  DY50_BufferId_t buffer_id)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if((buffer_id < DY50_BUFFER_ID_1) && (buffer_id > DY50_BUFFER_ID_4))
		return ACK_ERROR_INVALID_PARAMETER;

	dy50->uart.buf_tx.packet.payload[0] = buffer_id;

	return(DY50_SendCommand(dy50, DY50_CMD_GEN_CHAR, 1, PACKET_NOT_PAYLOAD, 500));
}

/*
 * @brief Command Generate Char (0x02) (DMA without blocking version)
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
 * @returns whether the image was generated successfully
 */
DY50_AckCode_t DY50_CMD_GenCharDMA(DY50_Typedef_t *dy50,  DY50_BufferId_t buffer_id)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if((buffer_id < DY50_BUFFER_ID_1) && (buffer_id > DY50_BUFFER_ID_4))
		return ACK_ERROR_INVALID_PARAMETER;

	dy50->uart.buf_tx.packet.payload[0] = buffer_id;

	return(DY50_SendCommandResponse_DMA(dy50, DY50_CMD_GEN_CHAR, 1, PACKET_NOT_PAYLOAD, 500));
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

	return(DY50_SendCommand(dy50, DY50_CMD_REG_MODEL, PACKET_NOT_PAYLOAD, PACKET_NOT_PAYLOAD, 500));
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
 * @return if the fingerprint we were looking for was found
 *
 * @retval 0x00 -> Successfull or 0x09 -> fingerprint not found
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

	//return DY50_SendCommand_DMA(dy50, Dy50_CMD_SEARCH, payload_tx_len, payload_rx_len);
	return DY50_SendCommandResponse_DMA(dy50, Dy50_CMD_SEARCH, payload_tx_len, payload_rx_len, 500);
}
