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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <util/delay.h>
#include <stdio.h>
#include <inttypes.h>

#include "uart.h"
#include "utils.h"
#include "mcp2515.h"
#include "defaults.h"
#include "nmea2000.h"

#include "mtc.h"

// ----------------------------------------------------------------------------

#define	PRINT(string, ...)		printf_P(PSTR(string), ##__VA_ARGS__)

static int putchar__(char c, FILE *stream) {
	uart1_putc(c);
	return 0;
}

static FILE mystdout = FDEV_SETUP_STREAM(putchar__, 0, _FDEV_SETUP_WRITE);


static void toggle_led(void){

	TOGGLE(LED_yellow);
}



// ----------------------------------------------------------------------------
// main program

// convert NMEA2000 messages into NMEA0183 messages (GLL and GGA strings)
// PGN 129029

int main(void){
	
	// Initialisiere die UART Schnittstelle
	uart1_init(UART_BAUD_SELECT(4800UL, F_CPU));

	// Umleiten der Standardausgabe => ab jetzt koennen wir printf() verwenden
		stdout = &mystdout;


	SET_OUTPUT(LED_yellow);
	SET_OUTPUT(LED_green);

	mt_Init();

	// Aktiviere Interrupts
	sei();

	nmea2000_init();

	mt_timeradd(toggle_led,MTC_200ms,MT_T_CONTINUOUS);

	nmea2000_iso_address_claim(0xFF);
	mt_TWait(MTC_250ms);

	while (1) {
		mt_Robin();
		nmea2000_get_message();
		nmea2000_process_complete_list();
	}
	
	return 0;
}

