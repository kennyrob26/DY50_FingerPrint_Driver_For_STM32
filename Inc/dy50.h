/*
 * dy50.h
 *
 *  Created on: May 16, 2026
 *      Author: kenny
 */

#ifndef DY50_DRIVER_FOR_STM32_INC_DY50_H_
#define DY50_DRIVER_FOR_STM32_INC_DY50_H_

#include "dy50_types.h"
#include "dy50_commands.h"


#define TOUCH_DEBOUNCE_TIME 200

#define ENROLL_MAX_TIME_IDLE_BETWEEN_READING 8000
#define SEARCH_MIN_TIME_BETWEEN_READING 200

DY50_AckCode_t DY50_Init(DY50_Typedef_t *dy50, UART_HandleTypeDef *huart, GPIO_TypeDef *touch_gpio_port, uint16_t touch_gpio_pin);
DY50_AckCode_t  DY50_ReInit(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_SetIndexTable(DY50_Typedef_t *dy50, uint16_t index, uint8_t value);
int16_t DY50_FindFirstFreeIDInIndexTable(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_GenerateChar(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id);
DY50_AckCode_t DY50_GenerateCharDMA(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id);
uint8_t DY50_ExistFingerOnTouch(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Enroll(DY50_Typedef_t *dy50);

void DY50_FingerTouchInterrupt(DY50_Typedef_t *dy50);

DY50_AckCode_t DY50_EnrollHandler(DY50_Typedef_t *dy50);
void DY50_EnrolResponseCallBack(DY50_Typedef_t *dy50, DY50_AckCode_t ackCode);


DY50_AckCode_t DY50_SearchFingerPrint(DY50_Typedef_t *dy50);
void DY50_SearchResponseCallBack(DY50_Typedef_t *dy50, const DY50_Search_Return_t *search_return);

DY50_AckCode_t DY50_DeleteTemplates(DY50_Typedef_t *dy50, uint16_t start_id, uint16_t num_of_templates);
DY50_AckCode_t DY50_DeleteTemplateId(DY50_Typedef_t *dy50, uint16_t id);
DY50_AckCode_t DY50_DeleteTemplateHandler(DY50_Typedef_t *dy50);

void DY50_TaskHandler(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_ChageStatus(DY50_Typedef_t *dy50, DY50_Status_t new_status);
#endif /* DY50_DRIVER_FOR_STM32_INC_DY50_H_ */


















