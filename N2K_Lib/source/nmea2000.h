/* Copyright (c) 2009 Philipp Drewes
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
// ----------------------------------------------------------------------------


#ifndef NMEA2000_H
#define NMEA2000_H

/**
*Title:    NMEA 2000 library
*Author:   Philipp Drewes <philipp.drewes@tu-harburg.de> 
*File:     $Id: phili $
*Software: AVR-GCC 3.3
*Hardware: any AVR connected to an MCP2515 via the SPI-Interface, tested on ATMEGA128 at 4 Mhz
*Usage:    see Doxygen manual
*/

/** 
 *  @defgroup pdrewes_nmea2000 NMEA2000 software stack 
 *  @code #include <nmea2000.h> @endcode
 * 
 *  @brief NMEA2000 library to build NMEA2k compatible devices
 * 
 * This library implements an NMEA2000 protocol stack for the AVR microcontrollers.
 * This library requires the mcp2515 library files written by Fabian Greif.
 *
 * The NMEA2000 stack is mainly controlled by the functions @code nmea2000_init() @endcode,
 * @code nmea2000_iso_address_claim() @endcode and @code nmea2000_get_message() @endcode.
 * The nmea2000_init() function is used to initialize the stack. nmea2000_iso_address_claim() is used to start the
 * address claioming process. nmea2000_get_message() must be called continously in the
 * in the main loop. It then manages the reassembly of received messages as well as the the
 * transmission of reply messages to incomming requests.
 *
 */

/**@{*/


#include <inttypes.h>


/** EEPROM Address where to store device ID must be defined */
#define NMEA2000_EEPROM_DEVICE_ID_ADDRESS 0x0000

#ifndef NMEA2000_EEPROM_DEVICE_ID_ADDRESS
	#error "EEPROM Device ID Address not defined"
#endif



/** PGNs required for network management*/

#define PGN_ACKNOWLEDGE				0x0E800
#define PGN_REQUEST					0x0EA00
#define PGN_ADDRESSCLAIM			0x0EE00
#define PGN_PRODUCTINFORMATION		0x1F014

/** Message based PGNs */
#define PGN_DISTANCELOG	0x1F513	// 128275
#define PGN_SPEED		0x1F503	// 128259

#define DEFAULT_SOURCE_ADDRESS		13
#define PRIORITY_LEVEL				 6



/** The NMEA2000 Transmit Message
* This struct defines the structure for an NMEA2000 message that is transmitted over the CAN bus.
* The structure includes the parameter group number of the message, the message priority (0..5), the
* destination address, the payload size and a pointer to the payload. NMEA2000 messages can have a
* maximum payload size of 223 bytes. When messages shall be broadcast to all ECUs the destination must be set 
* to 0xFF.
*
* \@see nmea2000_transmit_message()
*/

typedef struct{	
	uint32_t PGN;				/** The PGN of the message to be transmitted*/
	uint8_t priority;			/** The priority of the message */ 
	uint8_t destination;		/** The destination of the ECU to receive this message */
	uint8_t payload_size;		/** The size of the payload (given in bytes) */
	uint8_t *payload;			/** Pointer to the payload to be transmitted */
} tNMEA2000TransmitMessage;


struct sNMEA2000ReceiveBuffer;

typedef struct sNMEA2000ReceiveBuffer{	
	struct sNMEA2000ReceiveBuffer *next;	/** "pointer" to "next" receive container with the "same" message  */
	uint8_t identifier;						/** index of the "NMEA2000ReceiveMessage" array element related to a specific buffer */
	uint8_t processed_bytes;
	uint8_t payload[223];					/** memory to store payload */
	uint8_t	source;							/** source address of the transmitting ECU. Allows to distinguish messages with the same PGN from different ECUs*/
} tNMEA2000ReceiveBuffer;


/* function pointer to the function to execute calling "nmea2000_process_complete_list" */
typedef void (*funcp_exec_recbuf)(tNMEA2000ReceiveBuffer *buffer);				


typedef struct{
	uint32_t PGN;										/** The PGN of the message */
	uint8_t  payload_size;							/** Pointer to the payload to be transmitted */
	tNMEA2000ReceiveBuffer *receivebuffer_list;		/** "Pointer" to receive buffer list */
	funcp_exec_recbuf func;										/** function pointer für den handle */			
} tNMEA2000ReceiveMessage;

/*
** function prototypes
*/

/**
*  @brief   Initialize NMEA Protocoll stack
*  @param   none
*  @return  none
*
* During initialization of the NMEA2000 protocol stack
*/

extern uint8_t nmea2000_init (void);

/**
*  @brief   Returns the current source address of the ECU
*  @param   none
*  @return  current ECU source address
*
* During initialization of the NMEA2000 protocol stack the last source address of 
* this device is fetched from EEPROM. Source addresses may hange with time as
* additional devices are attached to the bus which might try to claim the same
* source address as this device and a higher NAME value.
*/

extern uint8_t nmea2000_getDeviceID(void);

/**
*  @brief   Request a device to transmit a specific message via the CAN bus
*  @param  destination_address  address of the ECU that the request is send to
*  @param  requestedPGN ParameterGroupNumber (PGN) of requested message
*  @return  none
*
* This function forces the device with the address "destination_address" to transmit
* a specific message via the CAN bus. The desired message is given by the requestedPGN
* parameter.
*/

extern void nmea2000_iso_request (uint8_t destination_address, uint32_t requestedPGN);

/**
*  @brief Begin address claim procedure
*  @param  destination_address  address of the ECU that the message is send to
*  @return  none
*
* This function starts the address claim procedure. Many of the payload fields of this message are filled
* with values given in file @code nmea2000_defs.h @endcode
*/

extern void nmea2000_iso_address_claim(uint8_t destination_address);

extern void nmea2000_transmit_message (tNMEA2000TransmitMessage *message_container);




extern void nmea2000_get_message(void);

extern void nmea2000_process_complete_list(void);

/**@}*/



#endif
