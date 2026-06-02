/*
 * dy50_commands.h
 *
 *  Created on: Jun 1, 2026
 *      Author: kenny
 */

#ifndef DY50_FINGERPRINT_DRIVER_FOR_STM32_INC_DY50_COMMANDS_H_
#define DY50_FINGERPRINT_DRIVER_FOR_STM32_INC_DY50_COMMANDS_H_

#include "dy50_types.h"

#define DY50_HEADER  (uint16_t) SWAP16(0xEF01)
#define DY50_ADDRESS (uint32_t) SWAP32(0xFFFFFFFF)

#define DY50_PASSWORD	0x00000000

#define PACKET_NOT_PAYLOAD 0
#define ACK_PACKET_SIZE 12

extern DY50_Typedef_t *dy50_global;


DY50_AckCode_t DY50_SendCommand_DMA(DY50_Typedef_t *dy50, DY50_Commands_t cmd, uint16_t tx_payload_len, uint16_t rx_payload_len);
DY50_AckCode_t DY50_WaitCommandResponse(DY50_Typedef_t *dy50, uint32_t timeout_response);
DY50_AckCode_t DY50_WaitCommandResponseBlock(DY50_Typedef_t *dy50, uint32_t timeout_response);
DY50_AckCode_t DY50_SendCommand(DY50_Typedef_t *dy50, DY50_Commands_t cmd, uint16_t tx_payload_len, uint16_t rx_payload_len, uint32_t timeout_response);
DY50_AckCode_t DY50_SendCommandResponse_DMA(DY50_Typedef_t *dy50, DY50_Commands_t cmd, uint16_t tx_payload_len, uint16_t rx_payload_len, uint32_t timeout_response);
DY50_AckCode_t DY50_CMD_ReadSystemParams(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_CMD_VerifyPassword(DY50_Typedef_t *dy50, uint32_t password);
DY50_AckCode_t DY50_CMD_ReadIndexTable(DY50_Typedef_t *dy50, uint8_t readIndexTable[], uint8_t size);
DY50_AckCode_t DY50_CMD_GetImage(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_CMD_GetImageDMA(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_CMD_GenChar(DY50_Typedef_t *dy50,  DY50_BufferId_t buffer_id);
DY50_AckCode_t DY50_CMD_GenCharDMA(DY50_Typedef_t *dy50,  DY50_BufferId_t buffer_id);
DY50_AckCode_t DY50_CMD_RegModel(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_CMD_Search(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id, uint16_t start_page_id, uint16_t page_num);

#endif /* DY50_FINGERPRINT_DRIVER_FOR_STM32_INC_DY50_COMMANDS_H_ */
