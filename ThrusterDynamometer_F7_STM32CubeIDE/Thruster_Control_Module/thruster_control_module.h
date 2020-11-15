/*
 * thruster_control_module.h
 *
 *  Created on: Nov 14, 2020
 *      Author: kpiek
 */

#ifndef THRUSTER_CONTROL_MODULE_H_
#define THRUSTER_CONTROL_MODULE_H_

#include <stdint.h>

#define THRUSTER_PAYLOAD_SIZE	30

typedef enum
{
	THRUSTER_TENSOMETER_ID = 1,

} thruster_module_id_t;

typedef struct
{
	thruster_module_id_t thruster_module_id;
	uint16_t payload_size;
	uint8_t	payload[THRUSTER_PAYLOAD_SIZE];
} thruster_message_t;

void ThrusterControlModule_InitSystem(void);

void ThrusterControlModule_ForwardReceivedCommandMessage(const uint8_t * command_data, uint16_t command_data_size);

#endif /* THRUSTER_CONTROL_MODULE_H_ */
