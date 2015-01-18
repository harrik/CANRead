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


#include <stdio.h>
#include <avr/pgmspace.h>


#include "nmea0183.h"
#include "nmea2000.h"
#include "uart.h"

#include <string.h>
#include "utils.h"
#include "defaults.h"


//$GPGLL,4916.45,N,12311.12,W,225444,A,*1D
//
//Where:
//     GLL          Geographic position, Latitude and Longitude
//     4916.46,N    Latitude 49 deg. 16.45 min. North
//     12311.12,W   Longitude 123 deg. 11.12 min. West
//     225444       Fix taken at 22:54:44 UTC
//     A            Data Active or V (void)
//     *iD          checksum data

typedef struct{
	int8_t lat_deg;
	uint8_t lat_min_predecimalplace;
	uint16_t lat_min_decimalplace;
	int16_t lon_deg;
	uint8_t lon_min_predecimalplace;
	uint8_t lon_min_decimalplace;
	uint8_t hour;
	uint8_t min;	
	uint8_t sec;
	char active_void;
} tnmea0183;

// compute checksum of nmea0183 data stream

static char make_checksum(char *string){
	uint8_t checksum = 0;
	++string;
	while (*string != '*' && *string != '\0')
	   checksum ^= *(uint8_t*)string++;
	
	return checksum;
}


//$GPGGA,191410,4735.5634,N,00739.3538,E,1,04,4.4,351.5,M,48.0,M,,*45
//       ^      ^           ^            ^ ^  ^   ^       ^     
//       |      |           |            | |  |   |       |    
//       |      |           |            | |  |   |       Höhe Geoid minus
//       |      |           |            | |  |   |       Höhe Ellipsoid (WGS84)
//       |      |           |            | |  |   |       in Metern (48.0,M)
//       |      |           |            | |  |   |        |      |           |            | |  |   Höhe über Meer (über Geoid)
//       |      |           |            | |  |   in Metern (351.5,M)
//       |      |           |            | |  |
//       |      |           |            | |  HDOP (horizontal dilution
//       |      |           |            | |  of precision) Genauigkeit
//       |      |           |            | |
//       |      |           |            | Anzahl der erfassten Satelliten
//       |      |           |            |
//       |      |           |            Qualität der Messung
//       |      |           |            (0 = ungültig)
//       |      |           |            (1 = GPS)
//       |      |           |            (2 = DGPS)
//       |      |           |            (6 = geschätzt nur NMEA-0183 2.3)
//       |      |           |
//       |      |           Längengrad
//       |      |
//       |      Breitengrad
//       |
//       Uhrzeit


//	$GPGLL,4735.5634,N,00739.3538,E,191410,A,A*4A



void nmea0183_PGN129025_to_GLL_GGA(tNMEA2000ReceiveBuffer *buf){

	tnmea0183 nmea0183_dataset;

	char nmea0183_out[50];
	char nmea0183_out2[6];


	uint32_t tmp_u32;

	int32_t tmp_i32;

	// use part of date value to check if dataset is valid.
	// all NMEA2000 bytes are set to 0xFF when invalid data has been received
	nmea0183_dataset.active_void = (*(buf->payload+2) == 0xFF ? 'V':'A');

	//	setup timevalue
	// time data is given in units of10^-4 sec
	// convert to plain seconds first

	tmp_u32 = *((uint32_t*)(buf->payload+3)) /10000UL;

	nmea0183_dataset.sec = tmp_u32 % 60;

	tmp_u32 = tmp_u32 /60;

	nmea0183_dataset.min = tmp_u32 % 60;

	nmea0183_dataset.hour = tmp_u32 / 60;



	tmp_i32 = *((int64_t*)(buf->payload+07)) / 100000000000ULL;

	nmea0183_dataset.lat_deg = (int8_t) (tmp_i32 / 100000ULL);

	tmp_i32 = (tmp_i32 % 100000ULL) *60 ;

	nmea0183_dataset.lat_min_predecimalplace = (uint8_t) (tmp_i32 / 100000UL);

	nmea0183_dataset.lat_min_decimalplace = (uint16_t) (tmp_i32 % 100000UL);

	tmp_i32 = *((int64_t*)(buf->payload+15)) / 100000000000ULL;

	nmea0183_dataset.lon_deg = (int8_t) (tmp_i32 / 100000ULL);

	tmp_i32 = (tmp_i32 % 100000ULL) *60 ;

	nmea0183_dataset.lon_min_predecimalplace = (uint8_t) (tmp_i32 / 100000UL);

	nmea0183_dataset.lon_min_decimalplace = (uint16_t) (tmp_i32 % 100000UL);


//	printf_P(PSTR("$GPGGA,191410,4735.5634,N,00739.3538,E,1,04,4.4,351.5,M,48.0,M,,*%02x\n"),);

	sprintf_P(nmea0183_out,PSTR("$GPGLL,%02u%02u.%04u,%c,%03u%02u.%04u,%c,%02u%02u%02u,%c,*"), \
				nmea0183_dataset.lat_deg > 0 ? nmea0183_dataset.lat_deg : - nmea0183_dataset.lat_deg, \
				nmea0183_dataset.lat_min_predecimalplace, nmea0183_dataset.lat_min_decimalplace, \
				nmea0183_dataset.lat_deg > 0 ? 'N':'S', \
				nmea0183_dataset.lon_deg > 0 ? nmea0183_dataset.lon_deg : - nmea0183_dataset.lon_deg, \
				nmea0183_dataset.lon_min_predecimalplace, nmea0183_dataset.lon_min_decimalplace, \
				nmea0183_dataset.lon_deg > 0 ?'E':'W', \
				nmea0183_dataset.hour,nmea0183_dataset.min, nmea0183_dataset.sec, \
				nmea0183_dataset.active_void, 0);

	sprintf(nmea0183_out2, "%02X\n",make_checksum(nmea0183_out));

	strcat(nmea0183_out,nmea0183_out2);

	uart1_puts(nmea0183_out);	

	TOGGLE(LED_green);

}






void nmea0183_init(void){



}

