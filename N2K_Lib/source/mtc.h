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

#ifndef MTC_H
#define MTC_H

/* Definition der MT-Zeitverzögerungen für mt_Wait und SDC*/
/* Je nach Lage des MT-Timers kann die Zeit auch um max. 1 Tick kürzer sein */
/* Diese Tabelle muß je nach gewählter Timer-Tick-Dauer angepasst werden ! */

#define MTC_5ms		    1
#define MTC_10ms	    2
#define MTC_15ms	    3
#define MTC_20ms	    4
#define MTC_25ms	    5
#define MTC_30ms	    6
#define MTC_35ms	    7
#define MTC_40ms	    8
#define MTC_45ms	    9
#define MTC_50ms	   10
#define MTC_100ms	   20
#define MTC_200ms	   40
#define MTC_250ms	   50
#define MTC_500ms	  100
#define MTC_1s        200
#define MTC_1s5       300 
#define MTC_2s		  400 
#define MTC_2s5	 	  500
#define MTC_3s	      600 
#define MTC_5s	     1000
#define MTC_7s	  	 1400
#define MTC_10s		 2000
#define MTC_11s		 2200
#define MTC_12s		 2400
#define MTC_15s		 3000
#define MTC_20s		 4000
#define MTC_30s		 6000
#define MTC_40s		 8000
#define MTC_45s		 9050
#define MTC_50s		10000
#define MTC_1m		12000
#define MTC_1m5		18000

/* Down-Counter-Definitions */
/* The fast Down-Counters (FDC) are dekrementiert with the interrupt timing interval*/
/*	  Maximum value is 255 Timerticks								   */
#define _NUM_FDC	1		/* Number of fast Down-Counters */
   
/* The slow Down-Counters (SDC) are decremented in cycles of one second */
/*	  Maximum value is 255 Sekunden, about 4 min.						 */
#define _NUM_SDC	2		/* Number of slow Down-Counters */
   
/* Indecees of the  fast Down-Counters, ranging from 0 to _NUM_SDC-1 */
#define _FDC_MTintern	0	/* MT-internal FDC building the time basis for the SDC */
   
/* Indecees of the slow Down-Counters, ranging from 0 to _NUM_SDC-1 */
#define _SDC_ABL	0		/* Wartezeit für Prozess-Ablauf */
#define _SDC_MWSP	1		/* Wartezeit für Abspeicherung */
   
/* Indecees of the Events von 0 bis _NUM_EVT-1 */
#define _NUM_EVT					2		/* Number of Events */
#define _EVT_SDC					0		/* SDC Event*/
#define _EVT_MCP2515_INT			1		/* MCP2515 Interrupt*/


#define MT_T_ONESHOT	0x01
#define MT_T_CONTINUOUS	0x00

#define t_res	uint16_t

typedef void (*funcp)(void);


/* Globale Variablen */

/* Schnelle Down-Counter, werden durch den Timerinterrupt dekremmentiert, falls >0 */
extern volatile uint8_t FDC[_NUM_FDC];   

/* Langsame Down-Counter, werden durch den _SDC_MTintern dekremmentiert, falls >0 */
extern volatile uint8_t SDC[_NUM_SDC];

/* Event-Flags, werden durch ein Ereignis gesetzt, zeigen ein Ereignis an */
extern volatile uint8_t Event[_NUM_EVT];

/*** Makros *******/
#define mt_SetFDC(Idx,Wert)       FDC[Idx] = Wert
#define mt_SetSDC(Idx,Wert)       SDC[Idx] = Wert
#define mt_ResFDC(Idx)            FDC[Idx] = 0
#define mt_ResSDC(Idx)            SDC[Idx] = 0
#define mt_TestFDC(Idx)           FDC[Idx]
#define mt_TestSDC(Idx)           SDC[Idx]

#define mt_Event(Idx)             Event[Idx] = 1
#define mt_ResEvent(Idx)          Event[Idx] = 0
#define mt_TestEvent(Idx)         Event[Idx]


/*** Prototypen ***/
extern uint8_t mt_timeradd( funcp func, t_res delay, uint8_t flags);
extern uint8_t mt_timerremove( funcp func );
extern void mt_timerinit(void);
extern void mt_Robin(void);
extern void mt_TWait(uint16_t  t);
extern void mt_EWait(uint8_t  e);
extern void mt_ETWait(uint8_t  e, uint16_t t);
extern uint8_t mt_ETimeout(void);

/*  Funktion: Initialisiert den Multitasking-Kernel                     */
extern void mt_Init(void);

#endif
