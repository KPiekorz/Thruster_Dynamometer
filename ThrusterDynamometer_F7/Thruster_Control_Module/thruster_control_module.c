/*
 * thruster_control_module.c
 *
 *  Created on: Nov 14, 2020
 *      Author: kpiek
 */

#include "thruster_control_module.h"
#include "tenso.h"
#include "task_eth.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* typdef enum */

typedef enum
{
	TENSOMETER_START,
	TENSOMETER_STOP,
	TENSOMETER_OFFSET,
	TENSOMETER_CALIBRATION,

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

/* static helper function */

void thrsuterControlModule_TensometerTimerCallback(TimerHandle_t timer_handle)
{
	thruster_message_t tenso_data_message;
	// get  tensometer mesurement
	uint16_t tenso_value = HX711_Value_Gram();

	// create tenso data message

	xQueueSend( sent_data_queue, (void *)&tenso_data_message, (TickType_t)QUEUE_WAIT );
}

/* control task */

static void thrsuterControlModule_TaskReceiveCommand(void *p)
{
	receive_command_queue = xQueueCreate(QUEUE_SIZE, sizeof(thruster_message_t));

	while(1)
	{
		thruster_message_t command_message;
		xQueueReceive( receive_command_queue, &command_message, (TickType_t)portMAX_DELAY);

		switch ((thruster_module_id_t) command_message.thruster_module_id)
		{
			case THRUSTER_TENSOMETER_ID:
				xQueueSend( tensometer_queue, (void *)command_message, (TickType_t)QUEUE_WAIT );
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

		// send directly using ethernet connection
		TaskEth_SendData();
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

		switch ((tensometer_control_mode_t) )
		{
			case TENSOMETER_START:
				// change timer period
				 xTimerChangePeriod(tensometer_timer, ticks, 0);
				// start tensometer timer
				xTimerStart(tensometer_timer, 0);
			break;
			case TENSOMETER_STOP:
				// stop timer
				xTimerStop(tensometer_timer, 0);
			break;
			case TENSOMETER_OFFSET:
				HX711_Tare(10);
			break;

			case TENSOMETER_CALIBRATION:
				HX711_Calibration(weight, HX711_Average_Value(10));
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
					thruster_task[i].task_ptr,
					thruster_task[i].task_size,
					NULL,
					thruster_task[i].task_priority,
					&xHandle );
	}
}

void ThrusterControlModule_ForwardReceivedCommandMessage(const uint8_t * command_data, uint16_t command_data_size)
{

	// create command message
	thruster_message_t msg;

	xQueueSend( receive_command_queue, (void *)&msg, (TickType_t)QUEUE_WAIT );
}
