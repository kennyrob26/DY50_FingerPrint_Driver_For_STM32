/*
 * dy50.c
 *
 *  Created on: May 16, 2026
 *      Author: kenny
 */


#include "dy50.h"

#include "math.h"

uint8_t response;


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
 * @return initialization state code
 */
DY50_AckCode_t  DY50_Init(DY50_Typedef_t *dy50, UART_HandleTypeDef *huart, GPIO_TypeDef *touch_gpio_port, uint16_t touch_gpio_pin)
{
	if((dy50 == NULL) || (huart == NULL) || (touch_gpio_port == NULL))
		return ACK_ERROR_INVALID_PARAMETER;

	if(dy50->status != DY50_STATUS_UNINITIALIZED)			//already initialized
		return ACK_OK;


	DY50_AckCode_t code_return = ACK_OK;
	DY50_Init_State_Machine init_status = DY50_INIT_DEFINE_PARAMS;

	uint32_t init_start_time = HAL_GetTick();

	while((init_status != DY50_INIT_COMPLETE))
	{
		if(((HAL_GetTick() - init_start_time) > 2000) || (code_return != ACK_OK))
			break;

		switch (init_status)
		{
			case DY50_INIT_DEFINE_PARAMS:
				dy50->huart = huart;
				dy50->uart.buf_tx.packet.header      = DY50_HEADER;
				dy50->uart.buf_tx.packet.chip_adress = DY50_ADDRESS;

				dy50->touch.gpio.port = touch_gpio_port;
				dy50->touch.gpio.pin  = touch_gpio_pin;

				dy50->status = DY50_STATUS_UNINITIALIZED;  //For uses readsystem and verify password functions

		        __HAL_UART_CLEAR_FEFLAG(dy50->huart);
		        __HAL_UART_CLEAR_NEFLAG(dy50->huart);
		        __HAL_UART_CLEAR_OREFLAG(dy50->huart);

				dy50_global = dy50;

				code_return = ACK_OK;
				init_status = DY50_INIT_READ_SYSTEM_PARAMS;
				break;

			case DY50_INIT_READ_SYSTEM_PARAMS:

				code_return = DY50_Sync_CMD_ReadSystemParams(dy50);

				if(code_return == ACK_OK)
					init_status = DY50_INIT_VERIFY_PASSWORD;
				break;

			case DY50_INIT_VERIFY_PASSWORD:

				code_return = DY50_Sync_CMD_VerifyPassword(dy50, DY50_PASSWORD);

				if(code_return == ACK_OK)
					init_status = DY50_INIT_READ_INDEX_TABLE;
				break;
			case DY50_INIT_READ_INDEX_TABLE:

				code_return = DY50_Sync_CMD_ReadIndexTable(dy50, dy50->info.table_index, sizeof(dy50->info.table_index));
				if(code_return == ACK_OK)
					init_status = DY50_INIT_COMPLETE;
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

DY50_AckCode_t  DY50_ReInit(DY50_Typedef_t *dy50)
{
	if((dy50 == NULL) || (dy50->huart == NULL) || (dy50->touch.gpio.port == NULL))
		return ACK_ERROR;

	HAL_UART_DMAStop(dy50->huart); //Pause DMA Receive

    volatile uint32_t clear_rdr_register;
	while ((dy50->huart->Instance->ISR & UART_FLAG_RXNE) == UART_FLAG_RXNE)
	{
		clear_rdr_register = dy50->huart->Instance->RDR; // consumes the data to clear the register
	}

	return (DY50_Init(dy50, dy50->huart, dy50->touch.gpio.port, dy50->touch.gpio.pin));
}




/*
 * @brief Receives a byte and finds a first free bit (0)
 *
 * @param byte_value is a target byte
 * @param byte_size the number of valid bits in the byte
 *
 * @return the bit index, if it exists, or returns 0xFF if the byte is full
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
 * @returns the first index found, or -1 in case of error or full table
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
 * @brief Generate Char function
 *
 * @note This function uses the GetImage function to read a fingerprint from the sensor,
 *       and then uses GenChar to generate an image based on the reading obtained with GetImage
 * @note The Generate Char stores in a temporary buffer, DY50 has 4 buffer:
 *	     1 - CharBuffer1
 *	     2 - CharBuffer2
 *	     3 - CharBuffer3
 *	     4 - CharBuffer4
*	     Each buffer stores a fingerprint image
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param buffer_id Defines in which buffet the generated image will be stored
 * @returns whether the image was generated successfully
 */
DY50_AckCode_t DY50_GenerateChar(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_AckCode_t code;

	code = DY50_Sync_CMD_GetImage(dy50);
	if(code == ACK_OK)
	{
		code = DY50_Sync_CMD_GenChar(dy50, buffer_id);
	}

	return code;
}

/*
 * @brief Generate Char function (DMA without blocking version)
 *
 * @note This function uses the GetImage function to read a fingerprint from the sensor,
 *       and then uses GenChar to generate an image based on the reading obtained with GetImage
 * @note The Generate Char stores in a temporary buffer, DY50 has 4 buffer:
 *	     1 - CharBuffer1
 *	     2 - CharBuffer2
 *	     3 - CharBuffer3
 *	     4 - CharBuffer4
*	     Each buffer stores a fingerprint image
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param buffer_id Defines in which buffet the generated image will be stored
 * @returns whether the image was generated successfully
 */
DY50_AckCode_t DY50_GenerateCharDMA(DY50_Typedef_t *dy50, DY50_BufferId_t buffer_id)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_AckCode_t code = ACK_WATING_RESPONSE;

	switch (dy50->genchar_status)
	{
		case DY50_GENCHAR_STATUS_IDLE:
			dy50->genchar_status = DY50_GENCHAR_STATUS_GET_IMAGE;
		case DY50_GENCHAR_STATUS_GET_IMAGE:

			code = DY50_Async_CMD_GetImage(dy50);

			if(code == ACK_OK)
			{
				dy50->genchar_status = DY50_GENCHAR_STATUS_GEN_CHAR;
				code = ACK_WATING_RESPONSE;
			}
			else if(code != ACK_WATING_RESPONSE)
			{
				dy50->genchar_status = DY50_GENCHAR_STATUS_IDLE;
			}

			break;

		case DY50_GENCHAR_STATUS_GEN_CHAR:

			code = DY50_Async_CMD_GenChar(dy50, buffer_id);

			if(code == ACK_OK)
			{
				dy50->genchar_status = DY50_GENCHAR_STATUS_IDLE;
			}
			else if(code != ACK_WATING_RESPONSE)
			{
				dy50->genchar_status = DY50_GENCHAR_STATUS_IDLE;
			}

			break;
		default:
			dy50->genchar_status = DY50_GENCHAR_STATUS_IDLE;
			break;
	}

	return code;
}





/*
 * @brief Check if the error message indicates a defective image
 *
 * @note In general, it means that it was possible to generate the image, but the quality was poor
 *
 * @param error receives an ack code
 *
 * @returns whether it's a bad image error
 *
 * @retval 1 case bad image error or 0 case other error
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
 * @returns whether there is a finger on the sensor
 *
 * @warning Please note that for the function to work correctly, the DY50's touch screen must be properly configured
 */
DY50_FingerTouchState_t DY50_FingerIsOnTouch(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	uint8_t touch_state = HAL_GPIO_ReadPin(dy50->touch.gpio.port, dy50->touch.gpio.pin);

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
 * @returns whether the enroll flow was successful
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

		//ack_code = DY50_GenerateChar(dy50, buffer_id);
		ack_code = DY50_GenerateCharDMA(dy50, buffer_id);
		if(ack_code == ACK_OK)
		{
			dy50->enroll.last_measure_time = HAL_GetTick();
		}
		else if(ack_code == ACK_WATING_RESPONSE)
		{
			dy50->enroll.last_state--;   //Repeat current state, waiting response
		}
		else if( checkIfBadImageError(ack_code))
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

	if(dy50->touch.flag == 0)
	{
		dy50->enroll.debouncing_init_time = HAL_GetTick();
		dy50->touch.flag = 1;
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
//DY50_AckCode_t DY50_EnrollHandler(DY50_Typedef_t *dy50)
//{
//	if(dy50->status == DY50_STATUS_UNINITIALIZED)
//		return ACK_ERROR_DY50_UNINITIALIZED;
//
//	DY50_AckCode_t ack_code = ACK_OK;
//
//	if(dy50->touch.flag == 1)
//	{
//		DY50_FingerTouchState_t finger_state;
//		if(dy50->enroll.handler_state == DY50_ENROLL_HANDLER_STATE_IDLE)
//		{
//			if(DY50_Mutex_Acquire(dy50, DY50_MUTEX_ENROLL_LOCK))
//			{
//				finger_state = DY50_FingerIsOnTouch(dy50);
//				if(finger_state == DY50_FINGER_IN_TOUCH_ACCEPTED)
//				{
//					dy50->enroll.handler_state = DY50_ENROLL_HANDLER_STATE_INITIALIZED;
//					ack_code = ACK_OK;
//				}
//			}
//			else
//			{
//				dy50->enroll.handler_state = DY50_ENROLL_HANDLER_STATE_IDLE;
//				dy50->touch.flag = 0;
//				return ACK_ERROR_MUTEX_IS_LOCK;
//			}
//		}
//		else
//		{
//			finger_state = HAL_GPIO_ReadPin(dy50->touch.gpio.port, dy50->touch.gpio.pin);
//		}
//
//		switch (finger_state) {
//			case DY50_FINGER_IN_TOUCH_ACCEPTED:
//				ack_code = DY50_Enroll(dy50);
//
//				if(ack_code != ACK_WATING_RESPONSE)
//				{
//
//					if((ack_code == ACK_OK) && (dy50->enroll.last_state == DY50_ENROLL_STATE_COMPLETE))
//					{
//						ack_code = DY50_Sync_CMD_RegModel(dy50);
//						if(ack_code == ACK_OK)
//						{
//							ack_code = DY50_Sync_CMD_StoreChar(dy50, DY50_BUFFER_ID_1, dy50->enroll.table_id);
//
//							if(ack_code != ACK_OK)
//							{
//								dy50->enroll.last_state = DY50_ENROLL_STATE_IDLE;  //StoreChar Error
//								dy50->enroll.handler_state = DY50_ENROLL_HANDLER_STATE_IDLE;
//							}
//						}
//						else
//						{
//						  dy50->enroll.last_state = DY50_ENROLL_STATE_IDLE;			//RegModel Error
//						  dy50->enroll.handler_state = DY50_ENROLL_HANDLER_STATE_IDLE;
//						}
//					}
//
//					//DY50_EnrolResponseCallBack(dy50, ack_code);
//					dy50->event.ack_code = ack_code;
//					dy50->event.data.enroll.last_state = dy50->enroll.last_state;
//					DY50_EventCallback(dy50, DY50_STATUS_ENROLL_HANDLER);
//
//					if(dy50->enroll.last_state == DY50_ENROLL_STATE_COMPLETE)
//					{
//						dy50->enroll.last_state = DY50_ENROLL_STATE_IDLE;
//						dy50->enroll.handler_state = DY50_ENROLL_HANDLER_STATE_IDLE;
//					}
//
//					if(dy50->enroll.handler_state == DY50_ENROLL_HANDLER_STATE_IDLE)
//					{
//						DY50_Mutex_Release(dy50, DY50_MUTEX_ENROLL_LOCK);
//					}
//
//					dy50->touch.flag = 0;
//				}
//
//				break;
//			case DY50_FINGER_IN_TOUCH_WAITING_TIME:
//				ack_code = ACK_WATING_RESPONSE;
//				break;
//			case DY50_FINGER_NOT_IN_TOUCH:
//
//				DY50_Mutex_Release(dy50, DY50_MUTEX_ENROLL_LOCK);
//				dy50->touch.flag = 0;
//				dy50->enroll.handler_state = DY50_ENROLL_HANDLER_STATE_IDLE;
//
//				ack_code = ACK_ERROR_NOT_FINGER;
//				break;
//			default:
//				break;
//		}
//
//	}
//
//	if(dy50->mutex == DY50_MUTEX_ENROLL_LOCK)
//	{
//		if((HAL_GetTick() - dy50->enroll.last_measure_time) > ENROLL_MAX_TIME_IDLE_BETWEEN_READING)
//		{
//			dy50->enroll.last_state = DY50_ENROLL_STATE_IDLE;
//			dy50->enroll.handler_state = DY50_ENROLL_HANDLER_STATE_IDLE;
//			DY50_Mutex_Release(dy50, DY50_MUTEX_ENROLL_LOCK);
//		}
//	}
//
//
//	return ack_code;
//}



static inline void DY50_EnrollError(DY50_Typedef_t *dy50)
{
	dy50->enroll.last_state = DY50_ENROLL_STATE_IDLE;
	dy50->enroll.handler_state = DY50_ENROLL_HANDLER_STATE_IDLE;
	dy50->touch.flag = 0;
	DY50_Mutex_Release(dy50, DY50_MUTEX_ENROLL_LOCK);
}


DY50_AckCode_t DY50_EnrollHandler(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_AckCode_t ack_code = ACK_OK;

	DY50_FingerTouchState_t finger_state;

	uint8_t callback_is_valid = 0;

	if(dy50->touch.flag == 1)
	{

		uint8_t finger_in_touch = HAL_GPIO_ReadPin(dy50->touch.gpio.port, dy50->touch.gpio.pin);
		if(finger_in_touch != 0)   // 0 is finger in touch, 1 not finger in touch
		{
			DY50_EnrollError(dy50);

			ack_code = ACK_ERROR_NOT_FINGER;
			dy50->touch.flag = 0;
			return ack_code;
		}


		switch (dy50->enroll.handler_state)
		{
			case DY50_ENROLL_HANDLER_STATE_IDLE:

				if(DY50_Mutex_Acquire(dy50, DY50_MUTEX_ENROLL_LOCK))
				{
					dy50->enroll.last_measure_time = HAL_GetTick();

					dy50->enroll.handler_state = DY50_ENROLL_HANDLER_CMD_ENROLL;
					dy50->enroll.last_state = DY50_ENROLL_STATE_IDLE;
					ack_code = ACK_OK;
				}
				else
				{
					DY50_EnrollError(dy50);
					return ACK_ERROR_MUTEX_IS_LOCK;
				}

				callback_is_valid = 0;

				break;

			case DY50_ENROLL_HANDLER_CMD_ENROLL:

				ack_code = DY50_Enroll(dy50);

				if(ack_code != ACK_WATING_RESPONSE)
				{
					if(ack_code == ACK_OK)
					{
						if(dy50->enroll.last_state == DY50_ENROLL_STATE_COMPLETE)
						{
							dy50->enroll.handler_state = DY50_ENROLL_HANDLER_CMD_REG_MODEL;
						}
						else
						{
							callback_is_valid = 1;
							dy50->touch.flag = 0;
						}
					}
					else
					{
						DY50_EnrollError(dy50);
					}
				}
				else
					callback_is_valid = 0;

				break;

			case DY50_ENROLL_HANDLER_CMD_REG_MODEL:

				ack_code = DY50_Async_CMD_RegModel(dy50);

				if(ack_code == ACK_OK)
				{
					dy50->enroll.handler_state = DY50_ENROLL_HANDLER_CMD_STORE_CHAR;
				}
				else if(ack_code != ACK_WATING_RESPONSE)
				{
					DY50_EnrollError(dy50);
				}
				//Else waiting response

				break;

			case DY50_ENROLL_HANDLER_CMD_STORE_CHAR:

				ack_code = DY50_Async_CMD_StoreChar(dy50, DY50_BUFFER_ID_1, dy50->enroll.table_id);

				if(ack_code == ACK_OK)
				{
					callback_is_valid = 1;

					dy50->enroll.handler_state = DY50_ENROLL_HANDLER_STATE_IDLE;
					DY50_Mutex_Release(dy50, DY50_MUTEX_ENROLL_LOCK);
				}
				else if(ack_code != ACK_WATING_RESPONSE)
				{
					DY50_EnrollError(dy50);
				}
				break;

			default:
				break;
		}

		}


	if(callback_is_valid == 1)
	{
		dy50->event.ack_code = ack_code;
		dy50->event.data.enroll.last_state = dy50->enroll.last_state;
		DY50_EventCallback(dy50, DY50_STATUS_ENROLL_HANDLER);
	}


	if(dy50->mutex == DY50_MUTEX_ENROLL_LOCK)
	{
		if((HAL_GetTick() - dy50->enroll.last_measure_time) > ENROLL_MAX_TIME_IDLE_BETWEEN_READING)
		{
			DY50_EnrollError(dy50);
		}
	}


	return ack_code;
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
 *
 * @returns extimed last id
 */
static inline int16_t DY50_FindLastIndexFilled(DY50_Typedef_t *dy50, uint16_t start_id, uint16_t end_id)
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
 * @brief Checks if the figerprint exists in the database
 *
 * @note It takes the obtained fingerprint and compares it with the fingerprints stored in the database
 *
 * @param dy50  Is a pointer for dy50 handler
 * @param id_found this is the ID found by the search. If it is 0xFFFF, no match was found
 * @param math_score Returns a value from 0 to 255 indicating how closely the fingerprint matches the one found in the database
 *
 * @returns whether the search was successful
 */
DY50_AckCode_t DY50_SearchFingerPrint(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_AckCode_t ack_code = ACK_OK;

	int16_t last_id;

	if(HAL_GPIO_ReadPin(dy50->touch.gpio.port, dy50->touch.gpio.pin) == 0)
	{
		switch (dy50->search.state)
		{
			case DY50_SEARCH_STATE_IDLE:

				if((HAL_GetTick() - dy50->search.last_measuere_time) > SEARCH_MIN_TIME_BETWEEN_READING)
				{
					if(DY50_Mutex_Acquire(dy50, DY50_MUTEX_SEARCH_LOCK))
					{
						dy50->search.last_measuere_time = HAL_GetTick();

						last_id = DY50_FindLastIndexFilled(dy50, 0, (dy50->info.database_capacity - 1));

						if(last_id <= 0)
						{
							ack_code = ACK_ERROR_FINGERPRINT_NOT_FOUND;
						}
						else
						{
							dy50->search.state = DY50_SEARCH_STATE_CMD_GENCHAR;
						}
					}
					else
					{
						dy50->search.state = DY50_SEARCH_STATE_IDLE;
						ack_code = ACK_ERROR_MUTEX_IS_LOCK;
					}

				}
				else
					ack_code = ACK_OK;	//There are no errors, just waiting
				break;

			case DY50_SEARCH_STATE_CMD_GENCHAR:

				ack_code = DY50_GenerateCharDMA(dy50, DY50_BUFFER_ID_1);

				if(ack_code == ACK_OK)
				{
					dy50->search.state = DY50_SEARCH_STATE_CMD_SEARCH;
				}
				else if(ack_code != ACK_WATING_RESPONSE)
				{
					dy50->search.state = DY50_SEARCH_STATE_IDLE;
					DY50_Mutex_Release(dy50, DY50_MUTEX_SEARCH_LOCK);
				}
				break;

			case DY50_SEARCH_STATE_CMD_SEARCH:

				last_id = DY50_FindLastIndexFilled(dy50, 0, (dy50->info.database_capacity - 1));
				ack_code = DY50_Async_CMD_Search(dy50, DY50_BUFFER_ID_1, 0, last_id);
				DY50_Search_Return_t search_return = {.id_found = 0xFFFF, .math_score = 0};

				uint8_t callback_is_valid = 0;

				if(ack_code == ACK_OK)
				{
					search_return.id_found =  ((uint16_t)dy50->uart.buf_rx.packet.payload[0]) << 8;
					search_return.id_found |= (((uint16_t)dy50->uart.buf_rx.packet.payload[1]) & 0x00FF);

					search_return.math_score = dy50->uart.buf_rx.packet.payload[3];

					callback_is_valid = 1;
				}
				else if(ack_code == ACK_ERROR_FINGERPRINT_NOT_FOUND)
				{
					callback_is_valid = 1;
				}
				else if(ack_code != ACK_WATING_RESPONSE)
				{
					callback_is_valid = 0;
					dy50->search.state = DY50_SEARCH_STATE_IDLE;
					DY50_Mutex_Release(dy50, DY50_MUTEX_SEARCH_LOCK);
				}

				if(callback_is_valid)
				{
					//DY50_SearchResponseCallBack(dy50, &search_return);

					dy50->event.data.search.id_found = search_return.id_found;
					dy50->event.data.search.math_score = search_return.math_score;
					DY50_EventCallback(dy50, DY50_STATUS_SEARCH_FINGERPRINT);

					DY50_Mutex_Release(dy50, DY50_MUTEX_SEARCH_LOCK);
				}

				break;
			default:
				break;
		}

	}
	else
	{
		dy50->search.state = DY50_SEARCH_STATE_IDLE;
		DY50_Mutex_Release(dy50, DY50_MUTEX_SEARCH_LOCK);
		ack_code = ACK_ERROR;
	}

	if(dy50->search.state == DY50_SEARCH_STATE_COMPLETED)
		dy50->search.state = DY50_SEARCH_STATE_IDLE;

	dy50->touch.flag = 0;	//We didn't use the flag, but it still needs to be reset to zero

	return ack_code;

}


DY50_AckCode_t DY50_DeleteTemplates(DY50_Typedef_t *dy50, uint16_t start_id, uint16_t num_of_templates)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if(num_of_templates == 0 || ((start_id + num_of_templates) > dy50->info.database_capacity))
		return ACK_ERROR_INVALID_PARAMETER;

	DY50_AckCode_t ack_code = DY50_ChageStatus(dy50, DY50_STATUS_DELETE_TEMPLATE);

	if(ack_code == ACK_OK)
	{
		dy50->delete.id = start_id;
		dy50->delete.num_of_templates = num_of_templates;
	}

	return ack_code;
}

DY50_AckCode_t DY50_DeleteTemplateId(DY50_Typedef_t *dy50, uint16_t id)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if(id >= dy50->info.database_capacity)
		return ACK_ERROR_INVALID_PARAMETER;

	return DY50_DeleteTemplates(dy50, id, 1);

}

DY50_AckCode_t Dy50_DeleteAllTemplates(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_AckCode_t ack_code = DY50_ChageStatus(dy50, DY50_STATUS_DELETE_TEMPLATE);

	if(ack_code == ACK_OK)
	{
		dy50->delete.id = 0;
		dy50->delete.num_of_templates = (dy50->info.database_capacity - 1);
	}

	return ack_code;
}


DY50_AckCode_t DY50_DeleteTemplateHandler(DY50_Typedef_t *dy50)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	if((dy50->delete.id >= dy50->info.database_capacity) || dy50->delete.num_of_templates == 0)
	{
		dy50->status = DY50_STATUS_IDLE;
		return ACK_ERROR_INVALID_PARAMETER;
	}

	DY50_AckCode_t ack_code;

	if(DY50_Mutex_Acquire(dy50, DY50_MUTEX_DELETE_LOCK))
	{
		if((dy50->delete.id == 0) && (dy50->delete.num_of_templates == (dy50->info.database_capacity - 1)))
		{
			ack_code = DY50_Async_CMD_Empty(dy50);
			if(ack_code != ACK_WATING_RESPONSE)
			{
				if(ack_code == ACK_OK)
					DY50_Sync_CMD_ReadIndexTable(dy50, dy50->info.table_index, sizeof(dy50->info.table_index));
				dy50->status = DY50_STATUS_IDLE;
				DY50_Mutex_Release(dy50, DY50_MUTEX_DELETE_LOCK);
			}
		}
		else
		{
			ack_code = DY50_Async_CMD_DeletChar(dy50, dy50->delete.id, dy50->delete.num_of_templates);

			if(ack_code != ACK_WATING_RESPONSE)
			{
				if(ack_code == ACK_OK)
					DY50_SetIndexTable(dy50, dy50->delete.id, 0);
				dy50->status = DY50_STATUS_IDLE;
				DY50_Mutex_Release(dy50, DY50_MUTEX_DELETE_LOCK);
			}
		}

	}
	else
	{
		ack_code = ACK_ERROR_MUTEX_IS_LOCK;
	}

	return ack_code;
}


/*
 * @brief The main Handler of the DY50
 *
 * @note This is a main Handler, a state machine that controls which handler will be executed
 *
 * @param dy50  Is a pointer for dy50 handler
 *
 * @retval none
 */
void DY50_TaskHandler(DY50_Typedef_t *dy50)
{
	switch (dy50->status)
	{
		case DY50_STATUS_UNINITIALIZED:
			DY50_ReInit(dy50);
			break;
		case DY50_STATUS_ENROLL_HANDLER:
			DY50_EnrollHandler(dy50);
			break;
		case DY50_STATUS_SEARCH_FINGERPRINT:
			DY50_SearchFingerPrint(dy50);
			break;
		case DY50_STATUS_DELETE_TEMPLATE:
			DY50_DeleteTemplateHandler(dy50);
			break;
		default:
			dy50->touch.flag = 0;
			break;
	}

}

DY50_AckCode_t DY50_ChageStatus(DY50_Typedef_t *dy50, DY50_Status_t new_status)
{
	if(dy50->status == DY50_STATUS_UNINITIALIZED)
		return ACK_ERROR_DY50_UNINITIALIZED;

	DY50_AckCode_t ack_code;

	if(dy50->mutex == DY50_MUTEX_IS_FREE)
	{
		dy50->status = new_status;
		ack_code = ACK_OK;
	}
	else
	{
		ack_code = ACK_ERROR_MUTEX_IS_LOCK;
	}

	return ack_code;
}

__weak void DY50_EventCallback(DY50_Typedef_t *dy50, DY50_Status_t event)
{
	if(dy50->event.ack_code != ACK_OK)
	{
		switch (dy50->event.ack_code)
		{
		case ACK_ERROR_NOT_FINGER:
			//ERROR not finger in module
			break;
		case ACK_ERROR_IMAGE_IS_TOO_LIGHT:
			//ERROR bad image (too LIGHT)
			break;
		case ACK_ERROR_IMAGE_IS_TOO_BLURRY:
			//ERROR bad image (too blurry)
			break;
		case ACK_ERROR_IMAGE_IS_AMORPHOUS:
			//ERROR bad image (amorphous)
			break;
		case ACK_ERROR_IMAGE_IS_TOO_LITTLE:
			//ERROR bad image (too little)
			break;

		//The idea is for you to address any errors you deem necessary

		default:
			//Other erros
			break;
		}
	}
	else if(dy50->event.ack_code == ACK_OK)
	{

		switch (event)
		{
			case DY50_STATUS_ENROLL_HANDLER:

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

			case DY50_STATUS_SEARCH_FINGERPRINT:

				uint16_t fingerprint_id = dy50->event.data.search.id_found;
				uint16_t math_score = dy50->event.data.search.math_score;

				if(fingerprint_id != 0xFFFF && math_score > 0)
				{
					//fingerprint exist
				}
				else
				{
					//fingerprint not exist
				}
				break;

			case DY50_STATUS_DELETE_TEMPLATE:
				//Delete successfully
				break;
			default:
				break;
		}
	}
}






