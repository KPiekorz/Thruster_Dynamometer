/*
 * thruster_control_module.c
 *
 *  Created on: Nov 14, 2020
 *      Author: kpiek
 */

#include "thruster_control_module.h"
#include "tenso.h"
#include "task_eth.h"
#include "main.h"
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* typdef enum */

typedef enum
{
	TENSOMETER_START = 1,
	TENSOMETER_STOP = 2,
	TENSOMETER_OFFSET = 3,
	TENSOMETER_CALIBRATION = 4,
	TENSOMETER_GET_VALUE = 5,

} tensometer_control_mode_t;

/* typedef */

typedef void (*control_task)(void *p);

typedef struct
{
	const char * task_name;
	control_task task_ptr;
	uint16_t task_size;
	uint16_t task_priority;

} thruster_control_task_t;

/* global variable */

#define QUEUE_WAIT 	10
#define QUEUE_SIZE	10
QueueHandle_t receive_command_queue;
QueueHandle_t sent_data_queue;
QueueHandle_t tensometer_queue;

#define TIMER_INIT_PERIOD	1
TimerHandle_t tensometer_timer;

#define THRUSTE_UART_PARAMETER_LEN	2
#define THRUSTER_UART_DATA_LEN 10
uint8_t thruster_uart_data[THRUSTER_UART_DATA_LEN];
extern UART_HandleTypeDef huart3;

uint8_t tansmit_data[10];

uint16_t tenso_global_value = 0;

/* interupt handlers */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		thruster_message_t message;
		memset(&message, 0, sizeof(thruster_message_t));

		message.thruster_module_id = thruster_uart_data[0];
		message.payload_size = thruster_uart_data[1];

		memcpy(message.payload, &thruster_uart_data[2], thruster_uart_data[1]);

		xQueueSendFromISR(receive_command_queue, &message, &xHigherPriorityTaskWoken);

		HAL_UART_Receive_IT(&huart3, thruster_uart_data, THRUSTER_UART_DATA_LEN);

		/* Now the buffer is empty we can switch context if necessary. */
		if (xHigherPriorityTaskWoken) {
			/* Actual macro used here is port specific. */
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

/* static helper function */

void thrsuterControlModule_TensometerTimerCallback(TimerHandle_t timer_handle)
{
	thruster_message_t tenso_data_message;
	// get  tensometer mesurement
	tenso_global_value = HX711_Value_Gram();
	tansmit_data[0] = tenso_global_value&0xFF;
	tansmit_data[1] = tenso_global_value>>8;

    HAL_UART_Transmit_IT(&huart3, tansmit_data, 2);
	// create tenso data message
}

/* control task */

static void thrsuterControlModule_TaskReceiveCommand(void *p)
{
	receive_command_queue = xQueueCreate(QUEUE_SIZE, sizeof(thruster_message_t));
	HAL_UART_Receive_IT(&huart3, thruster_uart_data, THRUSTER_UART_DATA_LEN);

	while(1)
	{
		thruster_message_t command_message;
		xQueueReceive( receive_command_queue, &command_message, (TickType_t)portMAX_DELAY);

		switch ((thruster_module_id_t) command_message.thruster_module_id)
		{
			case THRUSTER_TENSOMETER_ID:
				xQueueSend( tensometer_queue, (void *)&command_message, (TickType_t)QUEUE_WAIT );
			break;
			default:
			break;
		}

	}
}

static void thrsuterControlModule_TaskSendData(void *p)
{
	sent_data_queue = xQueueCreate(QUEUE_SIZE, sizeof(thruster_message_t));

	while(1)
	{
		thruster_message_t data_message;
		xQueueReceive( sent_data_queue, &data_message, (TickType_t)portMAX_DELAY);

		// send directly using  uart
	}
}

static void thrsuterControlModule_TaskGetTensometerMeasurement(void *p)
{
	tensometer_queue = xQueueCreate(QUEUE_SIZE, sizeof(thruster_message_t));

	tensometer_timer = xTimerCreate("Tensometer timer", pdMS_TO_TICKS(TIMER_INIT_PERIOD), pdTRUE, (void *) 0, thrsuterControlModule_TensometerTimerCallback);

	while(1)
	{
		thruster_message_t tensometer_message;
		xQueueReceive( tensometer_queue, &tensometer_message, (TickType_t)portMAX_DELAY);

		switch ((tensometer_control_mode_t) tensometer_message.payload[0])
		{
			case TENSOMETER_START:
			{
				uint16_t hz = (uint16_t)(tensometer_message.payload[2]<<8) | tensometer_message.payload[1];
				// change timer period
				xTimerChangePeriod(tensometer_timer, pdMS_TO_TICKS(1000 / (float)hz), 0);
				// start tensometer timer
				xTimerStart(tensometer_timer, 0);
			}
			break;
			case TENSOMETER_STOP:
				// stop timer
				xTimerStop(tensometer_timer, 0);
			break;
			case TENSOMETER_OFFSET:
				HX711_Tare(10);
			break;
			case TENSOMETER_CALIBRATION:
			{
				uint16_t weight = (uint16_t)(tensometer_message.payload[2]<<8) | tensometer_message.payload[1];

				HX711_Calibration(weight, HX711_Average_Value(10));
			}
			break;
			case TENSOMETER_GET_VALUE:
			{
				tenso_global_value = HX711_Value_Gram();
				tansmit_data[0] = tenso_global_value&0xFF;
				tansmit_data[1] = tenso_global_value>>8;
			    HAL_UART_Transmit_IT(&huart3, tansmit_data, 2);
			}
			break;
			default:
			break;
		}

	}
}

/* task definition */

const thruster_control_task_t thruster_task[] =
{
		{"Receive command task", 		thrsuterControlModule_TaskReceiveCommand,		 			 1000, 		 3},
		{"Send command task", 			thrsuterControlModule_TaskSendData, 		 				 1000, 		 3},
		{"Get tenso measurement", 		thrsuterControlModule_TaskGetTensometerMeasurement, 		 1000,		 3},
};

#define GetTaskArrayDim()	(sizeof(thruster_task)/sizeof(thruster_control_task_t))

/* global function */

void ThrusterControlModule_InitSystem(void)
{
	TaskHandle_t xHandle = NULL;

	for (uint8_t i = 0; i < GetTaskArrayDim(); i++)
	{
		xTaskCreate(thruster_task[i].task_ptr,
					thruster_task[i].task_name,
					thruster_task[i].task_size,
					NULL,
					thruster_task[i].task_priority,
					&xHandle);
	}
}
