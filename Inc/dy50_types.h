/*
 * dy50_types.h
 *
 *  Created on: Jun 1, 2026
 *      Author: kenny
 */

#ifndef DY50_FINGERPRINT_DRIVER_FOR_STM32_INC_DY50_TYPES_H_
#define DY50_FINGERPRINT_DRIVER_FOR_STM32_INC_DY50_TYPES_H_

#include "stm32g4xx_hal.h"

#define SWAP16(x) (((x) >> 8) | ((x) << 8))
#define SWAP32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))

#define PAYLOAD_TX_SIZE 34
#define PAYLOAD_RX_SIZE 34

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
	DY50_INIT_DEFINE_PARAMS      = 0,
	DY50_INIT_READ_SYSTEM_PARAMS = 1,
	DY50_INIT_VERIFY_PASSWORD    = 2,
	DY50_INIT_READ_INDEX_TABLE   = 3,
	DY50_INIT_COMPLETE           = 4,
	DY50_INIT_ERROR              = 5,
}DY50_Init_State_Machine;

typedef enum
{
	ACK_OK 				      	  = 0x00,
	ACK_ERROR_RECEIVING       	  = 0x01,
	ACK_ERROR_NOT_FINGER      	  = 0x02,
	ACK_ERROR_GET_FINGERPRINT 	  = 0x03,
	ACK_ERROR_IMAGE_IS_TOO_LIGHT  = 0x04,
	ACK_ERROR_IMAGE_IS_TOO_BLURRY = 0x05,
	ACK_ERROR_IMAGE_IS_AMORPHOUS  = 0x06,
	ACK_ERROR_IMAGE_IS_TOO_LITTLE = 0x07,
	ACK_ERROR_UNMATCHED           = 0x08,		//In Match command
	ACK_ERROR_DELETE_FAIL         = 0x10,

	ACK_ERROR_FINGERPRINT_NOT_FOUND = 0x09,		//In Search Command

	ACK_ERROR_MUTEX_IS_LOCK       = 241,

	ACK_WATING_RESPONSE           = 242,

	ACK_ERROR                     = 243,
	ACK_ERROR_DY50_UNINITIALIZED  = 244,
	ACK_ERROR_TABLE_ID_IS_FULL    = 245,
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
	DY50_CMD_MATCH              =  0x03,
	Dy50_CMD_SEARCH             =  0x04,
	DY50_CMD_REG_MODEL 			=  0x05,
	DY50_CMD_STORE_CHAR         =  0x06,
	DY50_CMD_LOAD_CHAR          =  0x07,
	DY50_CMD_DELETE_CHAR        =  0x0C,
	DY50_CMD_EMPTY			    =  0x0D,
	DY50_CMD_READ_SYSTEM_PARAMS =  0x0F,
	DY50_CMD_VERIFY_PASSWORD 	=  0x13,
	DY50_CMD_GET_RANDOM_CODE    =  0x14,
	DY50_CMD_VALID_TEMPLATE_NUM =  0x1D,
	DY50_CMD_READ_INDEX_TABLE   =  0x1F
}DY50_Commands_t;

typedef enum
{
	DY50_COMMAND_ASYNC,
	DY50_COMMAND_SYNC
}DY50_Command_Type_t;

typedef enum
{
	DY50_FLAG_COMMAND     = 0x01,
	DY50_FLAG_DATA_PACKET = 0x02,
	DY50_FLAG_END_PACKET  = 0x08
}DY50_Flag_t;

typedef enum
{
	DY50_CMD_DMA_STATUS_IDLE,
	DY50_CMD_DMA_STATUS_SEND,
	DY50_CMD_DMA_STATUS_WAITING,
}DY50_Command_DMA_Status_t;

typedef enum
{
	DY50_GENCHAR_STATUS_IDLE,
	DY50_GENCHAR_STATUS_GET_IMAGE,
	DY50_GENCHAR_STATUS_GEN_CHAR,
}DY50_GenerateChar_Status_t;

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
	DY50_ENROLL_STATE_WAITING_RESPONSE  = 6,

	DY50_ENROLL_ERROR_TIMEOUT      = 100,
}DY50_EnrollState_t;

typedef enum
{
	DY50_SEARCH_STATE_IDLE,
	DY50_SEARCH_STATE_CMD_GENCHAR,
	DY50_SEARCH_STATE_CMD_SEARCH,
	DY50_SEARCH_STATE_WAITING_RESPONSE,
	DY50_SEARCH_STATE_COMPLETED,

}DY50_SearchState_t;

typedef enum
{
	DY50_MATCH_STATE_IDLE         = 0,
	DY50_MATCH_STATE_CMD_GENCHAR  = 1,
	DY50_MATCH_STATE_CMD_LOADCHAR = 2,
	DY50_MATCH_STATE_CMD_MATCH    = 3,
}DY50_MatchStateHandler_t;

typedef enum
{
	DY50_FINGER_IN_TOUCH_ACCEPTED     = 0,
	DY50_FINGER_IN_TOUCH_WAITING_TIME = 1,
	DY50_FINGER_NOT_IN_TOUCH          = 2
}DY50_FingerTouchState_t;

typedef struct
{
	DY50_Gpio_t gpio;
	uint8_t flag;
}DY50_Touch_Info_t;

typedef struct
{
	uint8_t dma_flag;
	DY50_Buffer_t buf_tx;
	uint8_t tx_payload_len;
	DY50_Buffer_t buf_rx;
	uint8_t rx_payload_len;
}DY50_UART_Info_t;


typedef enum
{
	DY50_ENROLL_HANDLER_STATE_IDLE,
	DY50_ENROLL_HANDLER_CMD_ENROLL,
	DY50_ENROLL_HANDLER_CMD_REG_MODEL,
	DY50_ENROLL_HANDLER_CMD_STORE_CHAR,
	DY50_ENROLL_HANDLER_STATE_INITIALIZED,
}DY50_Enroll_Handler_State_t;

typedef struct
{
	DY50_EnrollState_t last_state;
	uint32_t last_measure_time;
	uint32_t debouncing_init_time;
	int16_t table_id;
}DY50_Enroll_t;


typedef struct
{
	//DY50_SearchState_t state;
	uint32_t last_measuere_time;
}DY50_Search_t;

typedef struct
{
	uint16_t id_found;
	uint8_t math_score;
}DY50_Search_Return_t;

typedef struct
{
	uint16_t target_id;
	uint8_t math_score;
}DY50_Match_Return_t;

typedef struct
{
	DY50_EnrollState_t last_state;
}DY50_Enroll_Return_t;

typedef union
{
	DY50_Search_Return_t search;
	DY50_Enroll_Return_t enroll;
	DY50_Match_Return_t match;
}DY50_Event_Data_t;

typedef struct
{
	DY50_AckCode_t ack_code;
	DY50_Event_Data_t data;
}DY50_Event_Info_t;

typedef enum
{
	DY50_DELETE_HANDLER_STATE_IDLE        = 0,
	DY50_DELETE_HANDLER_STATE_INITIALIZED,
}DY50_DeleteHandler_State_t;

typedef struct
{
	//DY50_DeleteHandler_State_t state;
	uint16_t id;
	uint16_t num_of_templates;
}DY50_Delete_t;


typedef struct
{
	uint16_t database_capacity;
	uint8_t  security_level;
	uint8_t packet_size;
	uint8_t  baund_rate;    //9600 x baundrate conf
	uint8_t table_index[64];
	uint16_t last_index_filled;
}DY50_Info_t;

typedef enum
{
	DY50_STATUS_UNINITIALIZED      = 0,
	DY50_STATUS_IDLE               = 1,
	DY50_STATUS_ENROLL_HANDLER     = 2,
	DY50_STATUS_SEARCH_FINGERPRINT = 3,
	DY50_STATUS_DELETE_TEMPLATE    = 4,
	DY50_STATUS_MATCH_HANDLER      = 5,
}DY50_Status_t;

typedef enum
{
	DY50_MUTEX_IS_FREE     = 0x00,
	DY50_MUTEX_ENROLL_LOCK = 0x01,
	DY50_MUTEX_SEARCH_LOCK = 0x02,
	DY50_MUTEX_DELETE_LOCK = 0x03,
	DY50_MUTEX_MATCH_LOCK  = 0x04,
}DY50_Mutex_Status_t;


typedef union
{
	DY50_EnrollState_t enroll_state;
	DY50_SearchState_t search_state;
	DY50_DeleteHandler_State_t delete_state;
	DY50_MatchStateHandler_t match_state;
}DY50_Handler_State_t;

typedef struct
{
	UART_HandleTypeDef *huart;
	DY50_Status_t status;
	DY50_Mutex_Status_t mutex;
	DY50_Info_t info;

	DY50_Event_Info_t event;

	DY50_Touch_Info_t touch;

	DY50_UART_Info_t uart;

	uint32_t response_time;

	DY50_Command_DMA_Status_t cmd_dma_status;

	DY50_Handler_State_t handler;

	DY50_GenerateChar_Status_t genchar_status;

	DY50_Enroll_t enroll;
	DY50_Search_t search;
	DY50_Delete_t delete;

	uint8_t math_target_id;

}DY50_Typedef_t;


#endif /* DY50_FINGERPRINT_DRIVER_FOR_STM32_INC_DY50_TYPES_H_ */
