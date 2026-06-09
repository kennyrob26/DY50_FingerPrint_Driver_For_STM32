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


DY50_AckCode_t DY50_SendCommand_DMA(DY50_Typedef_t *dy50, DY50_Commands_t cmd);
DY50_AckCode_t DY50_Async_WaitCommandResponse(DY50_Typedef_t *dy50, uint32_t timeout_response);
DY50_AckCode_t DY50_Sync_WaitCommandResponse(DY50_Typedef_t *dy50, uint32_t timeout_response);
DY50_AckCode_t DY50_Sync_SendCommand_Wait_Response(DY50_Typedef_t *dy50, DY50_Commands_t cmd, uint32_t timeout_response);
DY50_AckCode_t DY50_Async_SendCommand_Wait_Response(DY50_Typedef_t *dy50, DY50_Commands_t cmd, uint32_t timeout_response);
DY50_AckCode_t DY50_Sync_CMD_ReadSystemParams(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Sync_CMD_VerifyPassword(DY50_Typedef_t *dy50, uint32_t password);
DY50_AckCode_t DY50_Sync_CMD_ReadIndexTable(DY50_Typedef_t *dy50, uint8_t readIndexTable[], uint8_t size);
DY50_AckCode_t DY50_Sync_CMD_GetImage(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Async_CMD_GetImage(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Sync_CMD_GenChar(DY50_Typedef_t *dy50,  DY50_BufferId_t buffer_id);
DY50_AckCode_t DY50_Async_CMD_GenChar(DY50_Typedef_t *dy50,  DY50_BufferId_t buffer_id);
DY50_AckCode_t DY50_SetIndexTable(DY50_Typedef_t *dy50, uint16_t index, uint8_t value);
DY50_AckCode_t DY50_Sync_CMD_StoreChar(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id, uint16_t page_id);
DY50_AckCode_t DY50_Async_CMD_StoreChar(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id, uint16_t page_id);
DY50_AckCode_t DY50_Sync_CMD_RegModel(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Async_CMD_RegModel(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Async_CMD_Search(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id, uint16_t start_page_id, uint16_t page_num);
DY50_AckCode_t DY50_Sync_CMD_Search(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id, uint16_t start_page_id, uint16_t page_num);
uint8_t DY50_Mutex_Acquire(DY50_Typedef_t *dy50, DY50_Mutex_Status_t owner);
void DY50_Mutex_Release(DY50_Typedef_t *dy50, DY50_Mutex_Status_t owner);
DY50_AckCode_t DY50_Async_CMD_DeletChar(DY50_Typedef_t *dy50, uint16_t start_id, uint16_t num_of_templates);
DY50_AckCode_t DY50_Sync_CMD_DeletChar(DY50_Typedef_t *dy50, uint16_t start_id, uint16_t num_of_templates);
DY50_AckCode_t DY50_Async_CMD_Empty(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Sync_CMD_Empty(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Async_CMD_GetRandomCode(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Sync_CMD_GetRandomCode(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Async_CMD_LoadChar(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id, uint16_t page_id);
DY50_AckCode_t DY50_Sync_CMD_LoadChar(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id, uint16_t page_id);
DY50_AckCode_t DY50_Async_CMD_ValidTemplateNum(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Sync_CMD_ValidTemplateNum(DY50_Typedef_t *dy50);
#endif /* DY50_FINGERPRINT_DRIVER_FOR_STM32_INC_DY50_COMMANDS_H_ */
