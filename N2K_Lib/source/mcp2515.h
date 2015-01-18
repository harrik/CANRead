#ifndef	MCP2515_H
#define	MCP2515_H

// ----------------------------------------------------------------------------
/* Copyright (c) 2007 Fabian Greif
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

#include <inttypes.h>
#include <avr/pgmspace.h>

#include "mcp2515_defs.h"
#include "utils.h"

// ----------------------------------------------------------------------------

typedef struct {
	uint32_t id;				//!< ID der Nachricht (29 Bit)
	uint8_t length;				//!< Anzahl der Datenbytes
	uint8_t data[8];			//!< Die Daten der CAN Nachricht
	
} tCAN;


/**
 * \ingroup	    can_interface
 * \name		Bits des Filters fuer den MCP2515 umformatieren
 *
 * \code
 *  prog_uint8_t can_filter[] =
 *  {
 *  	MCP2515_FILTER_EXTENDED(0),	// Filter 0
 *  	MCP2515_FILTER_EXTENDED(0),	// Filter 1
 *  	
 *  	MCP2515_FILTER_EXTENDED(0),	// Filter 2
 *  	MCP2515_FILTER_EXTENDED(0),	// Filter 3
 *  	MCP2515_FILTER_EXTENDED(0),	// Filter 4
 *  	MCP2515_FILTER_EXTENDED(0),	// Filter 5
 *  	
 *  	MCP2515_FILTER_EXTENDED(0),	// Maske 0
 *  	MCP2515_FILTER_EXTENDED(0),	// Maske 1
 *  };
 * \endcode
 *
 * \see			can_static_filter()
 *
 * \~german
 * \warning		Dieses Makro sollte nur Werte verwendet die schon zur
 *				Compile-Zeit bekannt sind. Der Code sollte ansonsten zwar trotzdem
 *				funktionieren, wird danner aber sehr groß.
 *
 * \~english
 * \warning		Do not use this Makro for Variables, only for static values
 *				known at compile-time.
 */
//@{
#if defined(__DOXYGEN__)

	#define	MCP2515_FILTER_EXTENDED(id)
	#define	MCP2515_FILTER(id)
	
#else

	#define MCP2515_FILTER_EXTENDED(id)	\
			(uint8_t)  ((uint32_t) (id) >> 21), \
			(uint8_t)((((uint32_t) (id) >> 13) & 0xe0) | (1<<3) | \
				(((uint32_t) (id) >> 16) & 0x3)), \
			(uint8_t)  ((uint32_t) (id) >> 8), \
			(uint8_t)  ((uint32_t) (id))
	
	#define	MCP2515_FILTER(id) \
			(uint8_t)((uint32_t) id >> 3), \
			(uint8_t)((uint32_t) id << 5), \
			0, \
			0
#endif
//@}



// ----------------------------------------------------------------------------
/**
 * \ingroup	can_interface
 * \brief	Datenstruktur zur Aufnahme von CAN-Filtern
 *
 * \code
 *  rtr | Funtion
 * -----|------
 *  00  | alle Nachrichten unabhaengig vom RTR-Bit
 *  01  | ungültig
 *  10  | empfange nur nicht RTR-Nachrichten
 *  11  | empfange nur Nachrichten mit gesetzem RTR-Bit
 * \endcode
 *
 * \b ACHTUNG:
 * Funktioniert nur mit dem AT90CAN, beim MCP2515 wird der Parameter ignoriert. 
 *
 * \code
 *  ext | Funtion
 * -----|------
 *  00  | alle Nachrichten
 *  01  | ungueltig
 *  10  | empfange nur Standard-Nachrichten
 *  11  | empfange nur Extended-Nachrichten
 * \endcode
 *
 * \warning	Filter sind beim SJA1000 nur begrenzt nutzbar, man sollte ihn nur
 * 			in Systemen mit entweder Standard- oder Extended-Frames einsetzten,
 * 			aber nicht beidem zusammen.
 */


typedef struct
{
	uint32_t id;				//!< ID der Nachricht (11 oder 29 Bit)
	uint32_t mask;				//!< Maske
	struct {
		uint8_t rtr : 2;		//!< Remote Request Frame
		uint8_t extended : 2;	//!< extended ID
	} flags;
} can_filter_t;



// ----------------------------------------------------------------------------
/**
 * \ingroup can_interface
 * \brief   Inhalt der Fehler-Register
 */
typedef struct {
	uint8_t rx;				//!< Empfangs-Register
	uint8_t tx;				//!< Sende-Register
} can_error_register_t;


// ----------------------------------------------------------------------------
extern void mcp2515_write_register( uint8_t adress, uint8_t data );

// ----------------------------------------------------------------------------
extern uint8_t mcp2515_read_register(uint8_t adress);

// ----------------------------------------------------------------------------
extern void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data);

// -------------------------------------------------------------------------
extern __attribute__ ((gnu_inline)) inline void mcp2515_change_operation_mode(uint8_t mode)
{
	mcp2515_bit_modify(CANCTRL, 0xe0, mode);
	while ((mcp2515_read_register(CANSTAT) & 0xe0) != (mode & 0xe0))
		;
}

// ----------------------------------------------------------------------------
extern uint8_t mcp2515_read_status(uint8_t type);

// ----------------------------------------------------------------------------
extern bool mcp2515_init(void);

// ----------------------------------------------------------------------------
// check if there are any new messages waiting
extern uint8_t mcp2515_check_message(void);

// ----------------------------------------------------------------------------
// check if there is a free buffer to send messages
extern bool mcp2515_check_free_buffer(void);

// ----------------------------------------------------------------------------
extern uint8_t mcp2515_get_message(tCAN *message);

// ----------------------------------------------------------------------------
extern uint8_t mcp2515_send_message(const tCAN *message);


// ----------------------------------------------------------------------------
/**
 * \ingroup	can_interface
 * \brief	Setzt die Werte für alle Filter
 *
 * \code
 * // Filter und Masken-Tabelle anlegen
 * prog_char can_filter[] = {
 * 	MCP2515_FILTER_EXTENDED(0),	// Filter 0
 * 	MCP2515_FILTER_EXTENDED(0),	// Filter 1
 * 	
 * 	MCP2515_FILTER_EXTENDED(0),	// Filter 2
 * 	MCP2515_FILTER_EXTENDED(0),	// Filter 3
 * 	MCP2515_FILTER_EXTENDED(0),	// Filter 4
 * 	MCP2515_FILTER_EXTENDED(0),	// Filter 5
 * 	
 * 	MCP2515_FILTER_EXTENDED(0),	// Maske 0
 * 	MCP2515_FILTER_EXTENDED(0),	// Maske 1
 * };
 * 
 * ...
 *
 * // Filter und Masken-Tabelle laden
 * can_static_filter(can_filter);
 * \endcode
 *
 * \param	*filter_array	Array im Flash des AVRs mit den Initialisierungs-
 *							werten für die Filter des MCP2515
 * 
 * \see		MCP2515_FILTER_EXTENDED()
 * \see		MCP2515_FILTER()
 * \warning	Wird nur vom MCP2515 unterstuetzt.
 */

extern void mcp2515_static_filter(const prog_uint8_t *filter);



/**
 * \ingroup	can_interface
 *  @brief   set or change the settings of one of the MCP2515s filters
 *
 * The MCP2515 has 6 message acceptance filters. Two of these filters (filter 0 and filter 1) are assotiated with receive
 * buffer the remaining 4 filters (2, 3, 4 and 5) are connected to receive buffer 1. Each receive buffer additionally uses one mask register.
 * The mask register for receive buffer 0 is set when modifying filter 0 while the mask for receive buffer 1 is set with filter 2. *
 *
 * @param	number	Position des Filters
 * @param	filter	zu setzender Filter
 *
 * @return	 false falls ein Fehler auftrat, true ansonsten
 *
/*/

extern bool mcp2515_set_filter(uint8_t number, const can_filter_t *filter);


void mcp2515_read_error_register(can_error_register_t *error);


#endif	// MCP2515_H
