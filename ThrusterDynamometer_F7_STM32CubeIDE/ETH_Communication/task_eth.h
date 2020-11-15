/*
 * task_eth.h
 *
 *  Created on: Aug 18, 2020
 *      Author: kpiek
 */

#ifndef INC_TASK_ETH_H_
#define INC_TASK_ETH_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "projdefs.h"
#include <stdbool.h>


void TaskEth_Receive(void * pvParameters);

bool TaskEth_SendData(const void * eth_data, uint16_t eth_data_length);

#endif /* INC_TASK_ETH_H_ */
