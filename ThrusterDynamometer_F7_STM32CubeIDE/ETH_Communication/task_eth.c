/*
 * task_eth.c
 *
 *  Created on: Aug 18, 2020
 *      Author: kpiek
 */

#include "task_eth.h"

#include "main.h"

#include "lwip.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include <stdbool.h>
#include <stdint.h>

#include "thruster_control_module.h"

#define TASK_ETH_CONNECTION_PORT	4242

const uint8_t gupik_IPv4_address[4] = { 169 , 254 , 0 , 20 };

static struct netconn *newconn;

static bool taskEth_TcpListen(struct netconn **conn);
static bool taskEth_AcceptConnection(struct netconn *conn);
static bool taskEth_ReceiveData();
static void taskEth_ProcessReceivedData(const uint8_t * eth_data, uint16_t eth_data_len);

/* eth data message */
static uint8_t eth_data_message[MAX_ETH_DATA_LENGTH] = {0};

void TaskEth_Receive(void * pvParameters)
{
	struct netconn *conn = NULL;

	if (taskEth_TcpListen(&conn) == FALSE)
	{
		Panic();
	}

	for(;;)
	{

		if (taskEth_AcceptConnection(conn) == FALSE)
		{
			Panic();
		}

		if (taskEth_ReceiveData() == FALSE)
		{
			Panic();
		}

	}
	netconn_close(conn);
	netconn_delete(conn);
}

static bool taskEth_TcpListen(struct netconn **conn)
{
	(*conn) = netconn_new(NETCONN_TCP);

	if ((*conn) != NULL)
	{
		if (netconn_bind((*conn), (const ip_addr_t *)gupik_IPv4_address, TASK_ETH_CONNECTION_PORT) == ERR_OK)
		{
			netconn_listen((*conn));
			return TRUE;
		}
		netconn_close((*conn));
		netconn_delete((*conn));
	}

	return FALSE;
}
static bool taskEth_AcceptConnection(struct netconn *conn)
{
	// block task until new connection was received
	err_enum_t accept_err = netconn_accept(conn, &newconn);
	if (accept_err == ERR_OK)
	{
		return TRUE;
	}
	return FALSE;
}

static bool taskEth_ReceiveData()
{
	struct netbuf *buf;
	void *data;
	uint16_t len;
	err_enum_t recv_err;

	while ((recv_err = netconn_recv(newconn, &buf)) == ERR_OK)
	{
		// here have to make fuction to proccess rcv buf
		do
		{
			// put buf in data
			netbuf_data(buf, &data, &len);

			if (len <= MAX_ETH_DATA_LENGTH)
			{
				memset(eth_data_message, 0, sizeof(eth_data_message));
				memcpy(eth_data_message, data, len);

				taskEth_ProcessReceivedData(eth_data_message, len);
			}
		}
		while (netbuf_next(buf) >= 0);
		netbuf_delete(buf);
	}
	netconn_close(newconn);
	netconn_delete(newconn);
	if(recv_err == ERR_CLSD)
	{
		return TRUE;
	}
	return FALSE;
}

static void taskEth_ProcessReceivedData(const uint8_t * eth_data, uint16_t eth_data_len)
{
	ThrusterControlModule_ForwardReceivedCommandMessage(eth_data, eth_data_len);
}

bool TaskEth_SendData(const void * eth_data, uint16_t eth_data_length)
{
	err_t send_error = netconn_write(newconn, eth_data, eth_data_length, NETCONN_COPY);
	if (send_error != ERR_OK)
	{
		return FALSE;
	}
	return TRUE;
}
