/**
 ******************************************************************************
 * @addtogroup TauLabsModules Tau Labs Modules
 * @{
 * @addtogroup GeigerModule Serial port Geiger counter interface
 * @{
 *
 * @file       Geiger.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2015
 * @author     Edouard Lafargue, ed@lafargue.name, Copyright (C) 2015
 * @brief      Reads from Geiger counter over serial port and populate 'Radiation' UAVO
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

// ****************

#include "openpilot.h"

#include "radiation.h"

#include "modulesettings.h"
#include "pios_thread.h"

// ****************
// Private functions

static void geigerTask(void *parameters);
int parse_geiger_stream (uint8_t c, char *rx_buffer, RadiationData *RadData);

// ****************
// Private constants

#if defined(PIOS_GEIGER_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_GEIGER_STACK_SIZE
#else
#define STACK_SIZE_BYTES 384
#endif

#define TASK_PRIORITY                   PIOS_THREAD_PRIO_LOW

#define BRIDGE_BUF_LEN 10

// ****************
// Private variables

static struct pios_thread *geigerTaskHandle;

static char * geiger_buf;

static uint32_t usart_port;

static bool module_enabled = false;



/**
 * Initialise the module
 * \return -1 if initialisation failed
 * \return 0 on success
 */

static int32_t geigerStart(void)
{
	if (module_enabled) {
		// Start tasks
		geigerTaskHandle = PIOS_Thread_Create(geigerTask, "Geiger", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
		TaskMonitorAdd(TASKINFO_RUNNING_GEIGER, geigerTaskHandle);
		return 0;
	}

	return -1;
}

/**
 * Initialise the module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
static int32_t geigerInitialize(void)
{
	// TODO: Get from settings object
	usart_port = PIOS_COM_BRIDGE;

	/* Only run the module if we have a VCP port and a selected USART port */
	if (!usart_port) {
		module_enabled = false;
		return 0;
	}

#ifdef MODULE_Geiger_BUILTIN
	module_enabled = true;
#else
	uint8_t module_state[MODULESETTINGS_ADMINSTATE_NUMELEM];
	ModuleSettingsAdminStateGet(module_state);
	if (module_state[MODULESETTINGS_ADMINSTATE_GEIGER] == MODULESETTINGS_ADMINSTATE_ENABLED) {
		module_enabled = true;
	} else {
		module_enabled = false;
	}
#endif

	if (module_enabled) {
		geiger_buf = PIOS_malloc(BRIDGE_BUF_LEN);
		PIOS_Assert(geiger_buf);

		RadiationInitialize();
	}

	return 0;
}
MODULE_INITCALL(geigerInitialize, geigerStart)

/**
 * Main task. It does not return.
 */

static void geigerTask(void *parameters)
{

	RadiationData radiation;

	RadiationGet(&radiation);

	while (1) {
		uint8_t c;

		while(PIOS_COM_ReceiveBuffer(usart_port, &c, 1, 500)) {
			int res;
			/* TODO : Parse Geiger value, populate Geiger UAVObject */
			res = parse_geiger_stream (c,geiger_buf, &radiation);
			res++;
		}
	}
}

int parse_geiger_stream (uint8_t c, char *rx_buffer, RadiationData *RadData) {
	return 0;
}




/**
 * @}
 * @}
 */

