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

#define GEIGER_BUF_LEN 16

#define GEIGER_TIMEOUT_MS 4000  // Raise the alarm if Geiger stops updating for more than 4 seconds

#define	PARSER_OVERRUN	-2 // message buffer overrun before completing the message
#define	PARSER_ERROR	-1 // message unparsable by this parser
#define PARSER_INCOMPLETE	0 // parser needs more data to complete the message
#define PARSER_COMPLETE	1 // parser has received a complete message and finished processing


// ****************
// Private variables

static struct pios_thread *geigerTaskHandle;

static char * geiger_buf;

static uint32_t usart_port;

static bool module_enabled = false;

static uint32_t timeOfLastUpdateMs;

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
	usart_port = PIOS_COM_GEIGER;

	/* Only run the module if we have the Geiger port selected in the configuration */
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
		geiger_buf = PIOS_malloc(GEIGER_BUF_LEN);
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

	radiation.CPM = 999999;
	radiation.Status = RADIATION_STATUS_INITIALIZING;
	RadiationSet(&radiation);

	// We want to monitor the Geiger counter and update
	// the UAVO in case it stops talking back to us
	uint32_t timeNowMs = PIOS_Thread_Systime();
	timeOfLastUpdateMs = timeNowMs;

	// Main loop, never exits
	while (1) {
		uint8_t c;

		while(PIOS_COM_ReceiveBuffer(usart_port, &c, 1, 500)) {
			int ret;
			/* Parse Geiger value, populate Geiger UAVObject */
			ret = parse_geiger_stream (c,geiger_buf, &radiation);
			if (ret == PARSER_ERROR || ret == PARSER_OVERRUN) {
				radiation.Status = RADIATION_STATUS_ERROR;
				RadiationSet(&radiation);
			}

			if (ret == PARSER_COMPLETE) {
				timeOfLastUpdateMs = PIOS_Thread_Systime();
			}
		}

		// Now check for communication timeout:
		timeNowMs = PIOS_Thread_Systime();
		if ((timeNowMs - timeOfLastUpdateMs) >= GEIGER_TIMEOUT_MS) {
			// we have not received any valid Geiger counter sentence for too long.
			radiation.Status = RADIATION_STATUS_ERROR;
			RadiationSet(&radiation);
		}
	}
}

/**
 * Parse the serial data stream from a Medcom "GL" module.
 *
 *  Using the proven GPS NMEA module parser as the reference for
 *  this one, no point re-inventing the wheel
 *
 * A typical output string is: CPM:1:45:V
 *   "1" is the number of readings
 *   45 is the CPM reading
 *   V can be "V" or "X" depending on whether reading is valid or not
 *
 *   Max message length: CPM:1:999999:V\r\n (16 characters)
 */
int parse_geiger_stream (uint8_t c, char *rx_buffer, RadiationData *RadData) {

	static uint8_t rx_count = 0;
	static bool start_flag = false;
	static bool found_cr = false;

	// detect start while acquiring stream
	if (!start_flag && (c == 'C')) // Start of "CPM" line found
	{
		start_flag = true;
		found_cr = false;
		rx_count = 0;
	}
	else
	if (!start_flag)
		return PARSER_ERROR;

	if (rx_count >= GEIGER_BUF_LEN)
	{
		// The buffer is already full and we haven't found a valid CPM reading.
		// Flush the buffer.
		start_flag = false;
		found_cr = false;
		rx_count = 0;
		return PARSER_OVERRUN;
	}
	else
	{
		geiger_buf[rx_count] = c;
		rx_count++;
	}

	// look for ending '\r\n' sequence
	if (!found_cr && (c == '\r') )
		found_cr = true;
	else
	if (found_cr && (c != '\n') )
		found_cr = false;  // false end flag
	else
	if (found_cr && (c == '\n') )
	{

		// prepare to parse next sentence
		start_flag = false;
		found_cr = false;

		// Make sure our buffer is null-terminated
		// (know the last 2 characters are \r\n, we don't care about those
		geiger_buf[rx_count-2] = 0;

		// Our rxBuffer must look like this now:
		//   [0]           = 'C'
		//   ...           = zero or more bytes of sentence payload
		//   [end_pos - 1] = '\r'
		//   [end_pos]     = '\n'
		//
		// Now we can parse the reading.
		// will do a few sanity checks, then get the CPM reading

		// Check that we start with "CPM:1:"

		// Due to the way strcmp works, we can use it to check that
		// geiger_buf starts with "CPM:1:".
		if (strncmp(geiger_buf, "CPM:1:", 6) != 0) {
			rx_count = 0;
			return PARSER_INCOMPLETE;
		}

		// We're good now: get the valid flag
		bool valid = (geiger_buf[rx_count-3] == 'V');
		// Parse the CPM reading into a 32bit unsigned int
		uint32_t cpm = strtoul(&geiger_buf[6],NULL,10);

		RadData->CPM = cpm;
		RadData->Status = (valid) ? RADIATION_STATUS_VALID : RADIATION_STATUS_INVALID;
		RadiationSet(RadData);

		rx_count = 0;
		return PARSER_COMPLETE;

	}
	return PARSER_INCOMPLETE;

}




/**
 * @}
 * @}
 */

