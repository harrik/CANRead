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


#include <avr/io.h>
#include <util/delay.h>

#include "mcp2515.h"
#include "mcp2515_defs.h"

#include "utils.h"
#include "defaults.h"
#include "spi.h"

#include <stdio.h>

// -------------------------------------------------------------------------
void mcp2515_write_register( uint8_t adress, uint8_t data )
{
	RESET(MCP2515_CS);
	
	spi_putc(SPI_WRITE);
	spi_putc(adress);
	spi_putc(data);
	
	SET(MCP2515_CS);
}

// -------------------------------------------------------------------------
uint8_t mcp2515_read_register(uint8_t adress)
{
	uint8_t data;
	
	RESET(MCP2515_CS);
	
	spi_putc(SPI_READ);
	spi_putc(adress);
	
	data = spi_putc(0xff);	
	
	SET(MCP2515_CS);
	
	return data;
}

// -------------------------------------------------------------------------
void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
	RESET(MCP2515_CS);
	
	spi_putc(SPI_BIT_MODIFY);
	spi_putc(adress);
	spi_putc(mask);
	spi_putc(data);
	
	SET(MCP2515_CS);
}

// ----------------------------------------------------------------------------
uint8_t mcp2515_read_status(uint8_t type)
{
	uint8_t data;
	
	RESET(MCP2515_CS);
	spi_putc(type);
	data = spi_putc(0xff);	
	SET(MCP2515_CS);
	
	return data;
}

// -------------------------------------------------------------------------
bool mcp2515_init(void)
{
	SET(MCP2515_CS);
	SET_OUTPUT(MCP2515_CS);
	
	SET_INPUT(MCP2515_INT);
	SET(MCP2515_INT);
	
	// active SPI master interface
	spi_init();

	// reset MCP2515 by software reset.
	// After this he is in configuration mode.
	RESET(MCP2515_CS);
	spi_putc(SPI_RESET);
	SET(MCP2515_CS);

	// wait a little bit until the MCP2515 has restarted
	_delay_ms(50);

	// load CNF1..3 Register
	RESET(MCP2515_CS);
	spi_putc(SPI_WRITE);
	spi_putc(CNF3);
	
	// Bitrate 250 kbps using a crystal at 4 MHz
	spi_putc((0<<PHSEG22)|(1<<PHSEG21)|(0<<PHSEG20));									// CNF3
	spi_putc((1<<BTLMODE)|(0<<SAM)|(0<<PHSEG12)|(1<<PHSEG11)|(0<<PHSEG10) );			// CNF2
	spi_putc((0<<SJW1)|(0<<SJW0)|(0<<BRP4)|(0<<BRP3)|(0<<BRP2)|(0<<BRP1)|(0<<BRP0));	// CNF1

	// activate interrupts both receive interrupts
	spi_putc((1<<RX1IE)|(1<<RX0IE));	// CANINTE
	SET(MCP2515_CS);


	// test if we could read back the value in => is the chip accessible?
	if (mcp2515_read_register(CNF3) != ((0<<PHSEG22)|(1<<PHSEG21)|(0<<PHSEG20))) {
		return FALSE;
	}

	// activate the RXnBF Pins to signal message reception
	// this project uses the RXnBF pins only to visualize data reception via two LEDs.
	// the pins are not connected to the Mega128. Buffer status is read through the SPI-Interface.
	mcp2515_write_register(BFPCTRL, (1<<B1BFE) | (1<<B0BFE) | (1<<B1BFM) | (1<<B0BFM) );
	
	// set TXnRTS as inputs
	mcp2515_write_register(TXRTSCTRL, 0);
	
	// except only messages with extended identifiers, that meet the filter criteria
	// receive buffer 0 uses message roll over to move messages to buffer 1 if necessary

	// RXB1 | RXB0
	//       1 |       1 : receive any message (no filtering)
	//       1 |       0 : valid messages fullfilling filter criteria having extended identifierts 
	//       0 |       1 : valid messages fullfilling filter criteria having standard. identifierts 	
	//       0 |       0 : valid messages fullfilling filter criteria having arbitary identifiers

	mcp2515_write_register(RXB0CTRL, (1<<RXM1)|(0<<RXM0)|(1<<BUKT));

	// except only messages with extended identifiers, that meet the filter criteria
	mcp2515_write_register(RXB1CTRL, (1<<RXM1)|(0<<RXM0));	

	// reset device to normal mode
	mcp2515_write_register(CANCTRL, 0);

	return TRUE;
}

// ----------------------------------------------------------------------------
// check if there are any new messages waiting

uint8_t mcp2515_check_message(void) {
	return (!IS_SET(MCP2515_INT));
}

// ----------------------------------------------------------------------------
// check if there is a free buffer to send messages

bool mcp2515_check_free_buffer(void)
{
	uint8_t status = mcp2515_read_status(SPI_READ_STATUS);
	
	if ((status & 0x54) == 0x54) {
		// all buffers used
		return FALSE;
	}
	
	return TRUE;
}


uint8_t mcp2515_read_id(uint32_t *id)
{
	uint8_t first;
	uint8_t tmp;
	
	first = spi_putc(0xff);
	tmp   = spi_putc(0xff);
	
	if (tmp & (1 << IDE)) {
		spi_start(0xff);
		
		*((uint16_t *) id + 1)  = (uint16_t) first << 5;
		*((uint8_t *)  id + 1)  = spi_wait();
		spi_start(0xff);
		
		*((uint8_t *)  id + 2) |= (tmp >> 3) & 0x1C;
		*((uint8_t *)  id + 2) |=  tmp & 0x03;
		
		*((uint8_t *)  id)      = spi_wait();
		
		return TRUE;
	}
	else {
		spi_start(0xff);
		
		*((uint8_t *)  id + 3) = 0;
		*((uint8_t *)  id + 2) = 0;
		
		*((uint16_t *) id) = (uint16_t) first << 3;
		
		spi_wait();
		spi_start(0xff);
		
		*((uint8_t *) id) |= tmp >> 5;
		
		spi_wait();
		
		return FALSE;
	}
}


// ----------------------------------------------------------------------------
uint8_t mcp2515_get_message(tCAN *message)
{

		uint8_t addr;
		
	#ifdef	RXnBF_FUNKTION
			if (!IS_SET(MCP2515_RX0BF))
				addr = SPI_READ_RX;
			else if (!IS_SET(MCP2515_RX1BF))
				addr = SPI_READ_RX | 0x04;
			else
				return 0;
	#else
			// read status
			uint8_t status = mcp2515_read_status(SPI_RX_STATUS);
			
			if (_bit_is_set(status,6)) {
				// message in buffer 0
				addr = SPI_READ_RX;
			}
			else if (_bit_is_set(status,7)) {
				// message in buffer 1
				addr = SPI_READ_RX | 0x04;
			}
			else {
				// Error: no message available
				return 0;
			}
	#endif
		
		RESET(MCP2515_CS);
		spi_putc(addr);
		
		// CAN ID auslesen und ueberpruefen
		mcp2515_read_id(&message->id);

		// read DLC
		uint8_t length = spi_putc(0xff);
		
		length &= 0x0f;
		message->length = length;
		// read data
		for (uint8_t i=0;i<length;i++) {
			message->data[i] = spi_putc(0xff);
		}
		SET(MCP2515_CS);
		
		// clear interrupt flag
	#ifdef RXnBF_FUNKTION
		if (!IS_SET(MCP2515_RX0BF))
	#else
		if (_bit_is_set(status, 6))
	#endif
			mcp2515_bit_modify(CANINTF, (1<<RX0IF), 0);
		else
			mcp2515_bit_modify(CANINTF, (1<<RX1IF), 0);
		
//		CAN_INDICATE_RX_TRAFFIC_FUNCTION;
		
	#ifdef RXnBF_FUNKTION
			return 1;
	#else
			return (status & 0x07) + 1;
	#endif	

}

// ----------------------------------------------------------------------------

#ifdef USE_SOFTWARE_SPI

static uint8_t usi_interface_spi_temp;

static void spi_start(uint8_t data) {
	usi_interface_spi_temp = spi_putc(data);
}

static uint8_t spi_wait(void) {
	return usi_interface_spi_temp;
}

#else

static void spi_start(uint8_t data) {

	SPDR = data;
}

static uint8_t spi_wait(void) {
	// warten bis der vorherige Werte geschrieben wurde
	while(!(SPSR & (1<<SPIF)))
		;
	
	return SPDR;
}

#endif


void mcp2515_write_id(const uint32_t *id)
{
	uint8_t tmp;

	spi_start(*((uint16_t *) id + 1) >> 5);

	// naechsten Werte berechnen
	tmp  = (*((uint8_t *) id + 2) << 3) & 0xe0;
	tmp |= (1 << IDE);
	tmp |= (*((uint8_t *) id + 2)) & 0x03;
		
	// warten bis der vorherige Werte geschrieben wurde
	spi_wait();
		
	// restliche Werte schreiben
	spi_putc(tmp);
	spi_putc(*((uint8_t *) id + 1));
	spi_putc(*((uint8_t *) id));
}



uint8_t mcp2515_send_message(const tCAN *message)
{
	// Status des MCP2515 auslesen
	uint8_t status = mcp2515_read_status(SPI_READ_STATUS);
	
	/* Statusbyte:
	 *
	 * Bit	Funktion
	 *  2	TXB0CNTRL.TXREQ
	 *  4	TXB1CNTRL.TXREQ
	 *  6	TXB2CNTRL.TXREQ
	 */
	uint8_t address;
	if (_bit_is_clear(status, 2)) {
		address = 0x00;
	}
// Message transmission is now performed with only one transmission buffer.
// Otherwise correct sequencing of can frames during fast-packet-transmission can not
// be guaranteed

/*	else if (_bit_is_clear(status, 4)) {
		address = 0x02;
	} 
	else if (_bit_is_clear(status, 6)) {
		address = 0x04;
	}*/
	else {
		// Alle Puffer sind belegt,
		// Nachricht kann nicht verschickt werden
		return 0;
	}

	
	RESET(MCP2515_CS);
	spi_putc(SPI_WRITE_TX | address);
	mcp2515_write_id(&message->id);
	uint8_t length = message->length & 0x0f;
	
	// Nachrichten Laenge einstellen
	spi_putc(length);
		
	// Daten
	for (uint8_t i=0;i<length;i++) {
		spi_putc(message->data[i]);
	}

	SET(MCP2515_CS);
	
	_delay_us(1);
	
	// CAN Nachricht verschicken
	// die letzten drei Bit im RTS Kommando geben an welcher
	// Puffer gesendet werden soll.
	RESET(MCP2515_CS);
	address = (address == 0) ? 1 : address;
	spi_putc(SPI_RTS | address);
	SET(MCP2515_CS);
	
//	CAN_INDICATE_TX_TRAFFIC_FUNCTION;
	
	return address;
}




// ---------------------------------------------------------------------------------
void mcp2515_static_filter(const prog_uint8_t *filter)
{
	// change to configuration mode
	mcp2515_bit_modify(CANCTRL, 0xe0, (1<<REQOP2));
	// wait until change is complete
	while ((mcp2515_read_register(CANSTAT) & 0xe0) != (1<<REQOP2))
		;

	// only messages with extended data format are accepted as all NMEA2000 messages are of the extended type. 
	// buffer 0 supports roll over
//	mcp2515_write_register(RXB0CTRL, (0<<RXM1)|(0<<RXM0)|(1<<BUKT));
//	mcp2515_write_register(RXB1CTRL, (0<<RXM1)|(0<<RXM0));

//	mcp2515_write_register(RXB0CTRL, (1<<BUKT));
//	mcp2515_write_register(RXB1CTRL, 0);

    
	uint8_t i, j;
	for (i = 0; i < 0x30; i += 0x10)
	{
		RESET(MCP2515_CS);
		spi_putc(SPI_WRITE);
		spi_putc(i);
		
		for (j = 0; j < 12; j++) 
		{
			if (i == 0x20 && j >= 0x08)
				break;
			
			spi_putc(pgm_read_byte(filter++));
		}
		SET(MCP2515_CS);
	}
	
	mcp2515_bit_modify(CANCTRL, 0xe0, 0);
}


bool mcp2515_set_filter(uint8_t number, const can_filter_t *filter)
{
	uint8_t mask_address = 0;
	uint8_t mode = mcp2515_read_register(CANSTAT);

	// the mcp2515 supports 5 different message filters
	if (number > 5)
		return FALSE;
	
	// change to configuration mode
	mcp2515_change_operation_mode( (1<<REQOP2) );
	
	// set filter mask
	if (number == 0)
	{
		mask_address = RXM0SIDH;
		
		if (filter->flags.extended == 0x3) {
			// only extended identifier
			mcp2515_write_register(RXB0CTRL, (1<<RXM1) | (1<<BUKT));
		}
		else if (filter->flags.extended == 0x2) {
			// only standard identifier
			mcp2515_write_register(RXB0CTRL, (1<<RXM0) | (1<<BUKT));
		}
		else {
			// receive all messages
			mcp2515_write_register(RXB0CTRL, 0 | (1<<BUKT));
		}
	}
	else if (number == 2)
	{
		mask_address = RXM1SIDH;
		
		if (filter->flags.extended == 0x3) {
			// only extended identifier
			mcp2515_write_register(RXB1CTRL, (1<<RXM1));
		}
		else if (filter->flags.extended == 0x2) {
			// only standard identifier
			mcp2515_write_register(RXB1CTRL, (1<<RXM0));
		}
		else {
			mcp2515_write_register(RXB1CTRL, 0);
		}
	}
	
	if (mask_address)
	{
		RESET(MCP2515_CS);
		spi_putc(SPI_WRITE);
		spi_putc(mask_address);
		mcp2515_write_id(&filter->mask);
		SET(MCP2515_CS);
		
		_delay_us(1);
	}
	
	// write filter
	uint8_t filter_address;
	if (number >= 3) {
		number -= 3;
		filter_address = RXF3SIDH;
	}
	else {
		filter_address = RXF0SIDH;
	}
	
	RESET(MCP2515_CS);
	spi_putc(SPI_WRITE);
	spi_putc(filter_address | (number * 4));
	mcp2515_write_id(&filter->id);
	SET(MCP2515_CS);
	
	_delay_us(1);
	
	// restore previous mode
	mcp2515_change_operation_mode( mode );
	
	return TRUE;
}


// ----------------------------------------------------------------------------
void mcp2515_read_error_register(can_error_register_t *error){
	
	error->tx = mcp2515_read_register(TEC);
	error->rx = mcp2515_read_register(REC);
}

