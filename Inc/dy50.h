/*
 * dy50.h
 *
 *  Created on: May 16, 2026
 *      Author: kenny
 */

#ifndef DY50_DRIVER_FOR_STM32_INC_DY50_H_
#define DY50_DRIVER_FOR_STM32_INC_DY50_H_

#include "stm32g4xx_hal.h"

#define SWAP16(x) (((x) >> 8) | ((x) << 8))
#define SWAP32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))

#define PAYLOAD_TX_SIZE 32

#define DY50_HEADER  (uint16_t) SWAP16(0xEF01)
#define DY50_ADDRESS (uint32_t) SWAP32(0xFFFFFFFF)

#define PACKET_NOT_PAYLOAD 0
#define ACK_PACKET_SIZE 12



typedef struct __attribute__((packed))
{
	uint16_t header;
	uint32_t chip_adress;
	uint8_t flag;
	uint16_t length;
	uint8_t code;
	uint8_t payload[PAYLOAD_TX_SIZE];
}DY50_packet_t;

typedef union
{
	DY50_packet_t packet;
	uint8_t raw[sizeof(DY50_packet_t)];
}DY50_Buffer_t;

typedef struct
{
	GPIO_TypeDef *port;
	uint16_t pin;
}DY50_Gpio_t;



typedef enum
{
	ACK_OK 				      	  = 0x00,
	ACK_ERROR_RECEIVING       	  = 0x01,
	ACK_ERROR_NOT_FINGER      	  = 0x02,
	ACK_ERROR_GET_FINGERPRINT 	  = 0x03,
	ACK_ERROR_IMAGE_IS_TOO_LIGHT  = 0x04, //0x04 a 0x07
	ACK_ERROR_IMAGE_IS_TOO_BLURRY = 0x05,
	ACK_ERROR_IMAGE_IS_AMORPHOUS  = 0x06,
	ACK_ERROR_IMAGE_IS_TOO_LITTLE = 0x07,

	ACK_ERROR_IMPOSSIBLE_STATE    = 246,
	ACK_ERROR_MAX_ATTEMPS         = 247,
	ACK_ERROR_TIMEOUT             = 248,
	ACK_ERROR_INVALID_PARAMETER   = 249,
	ACK_ERROR_HANDLER_NOT_DEFINED = 250,
	ACK_ERROR_UART_NOT_DEFINED    = 251,
	ACK_ERROR_UART_TX             = 252,
	ACK_ERROR_UART_RX             = 253,
	ACK_ERROR_RESPONSE            = 255,
}DY50_AckCode_t;

typedef enum
{
	DY50_CMD_GET_IMAGE 			=  0x01,
	DY50_CMD_GEN_CHAR  			=  0x02,
	DY50_CMD_REG_MODEL 			=  0x05,
	DY50_CMD_VERIFY_PASSWORD 	=  0x13,
	DY50_CMD_READ_SYSTEM_PARAMS =  0x0F,
}DY50_Commands_t;

typedef enum
{
	DY50_FLAG_COMMAND     = 0x01,
	DY50_FLAG_DATA_PACKET = 0x02,
	DY50_FLAG_END_PACKET  = 0x08
}DY50_Flag_t;

typedef enum
{
	DY50_BUFFER_ID_1  = 0x01,
	DY50_BUFFER_ID_2  = 0x02,
	DY50_BUFFER_ID_3  = 0x03,
	DY50_BUFFER_ID_4  = 0x04
}DY50_BufferId_t;


typedef enum
{
	DY50_ENROLL_STATE_IDLE          = 0,
	DY50_ENROLL_STATE_WRITE_BUFFER1 = 1,
	DY50_ENROLL_STATE_WRITE_BUFFER2 = 2,
	DY50_ENROLL_STATE_WRITE_BUFFER3 = 3,
	DY50_ENROLL_STATE_WRITE_BUFFER4 = 4,
	DY50_ENROLL_STATE_COMPLETE      = 5,

	DY50_ENROLL_ERROR_TIMEOUT      = 100,
}DY50_EnrollState_t;

typedef enum
{
	DY50_FINGER_IN_TOUCH_ACCEPTED     = 0,
	DY50_FINGER_IN_TOUCH_WAITING_TIME = 1,
	DY50_FINGER_NOT_IN_TOUCH          = 2
}DY50_FingerTouchState_t;


typedef struct
{
	DY50_EnrollState_t last_state;
	uint32_t last_measure_time;
	uint32_t debouncing_init_time;
}DY50_Enroll_t;

typedef struct
{
	uint16_t database_capacity;
	uint8_t  security_level;
	uint8_t packet_size;
	uint8_t  baund_rate;    //9600 x baundrate conf
}DY50_Info_t;

typedef struct
{
	UART_HandleTypeDef *huart;
	DY50_Info_t info;
	DY50_Gpio_t touch_gpio;
	DY50_Buffer_t buf_tx;
	DY50_Buffer_t buf_rx;
	DY50_Enroll_t enroll;
	uint8_t touch_flag;
}DY50_Typedef_t;

void DY50_Init(DY50_Typedef_t *dy50, UART_HandleTypeDef *huart, GPIO_TypeDef *touch_gpio_port, uint16_t touch_gpio_pin);
DY50_AckCode_t DY50_SendCommand(DY50_Typedef_t *dy50, uint8_t cmd, uint16_t tx_payload_len, uint16_t rx_payload_len);
DY50_AckCode_t DY50_CMD_ReadSystemParams(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_CMD_VerifyPassword(DY50_Typedef_t *dy50, uint32_t password);
DY50_AckCode_t DY50_CMD_GetImage(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_CMD_GenChar(DY50_Typedef_t *dy50,  DY50_BufferId_t buffer_id);
DY50_AckCode_t DY50_GenerateChar(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id);
DY50_AckCode_t DY50_CMD_RegModel(DY50_Typedef_t *dy50);
uint8_t DY50_ExistFingerOnTouch(DY50_Typedef_t *dy50);
DY50_AckCode_t DY50_Enroll(DY50_Typedef_t *dy50);

void DY50_FingerTouchInterrupt(DY50_Typedef_t *dy50);
void DY50_EnrollHandler(DY50_Typedef_t *dy50);
void DY50_EnrolResponseCallBack(DY50_Typedef_t *dy50, DY50_AckCode_t ackCode);
#endif /* DY50_DRIVER_FOR_STM32_INC_DY50_H_ */


















