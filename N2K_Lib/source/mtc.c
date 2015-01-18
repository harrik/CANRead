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

// Idee zum Scheduler nach Peter Dannegger

/**
 * \brief Multi-Tasking Core 
 * Multi-Tasking Core for ATMEL microcontrollers */

#define MTK16  1
#include <avr/interrupt.h>
#include "mtc.h"
#include "utils.h"

#define MT_MAXTIMERS	10
#define MT_T_FREE		254
#define MT_T_LAST		255


typedef struct{
  uint8_t next;				// next timer in list or end mark
  t_res ticks;				// total timer ticks, necessary when reinsering continous timers ; set to 0 for one-shot timers
  t_res ticks_delta;		// timer ticks delta to previous entry
  funcp func;				// pointer to function been exectuted
} t_ctrl_struct;

/* Prototypen */

/* Globale Variablen */

volatile uint8_t FDC[_NUM_FDC];   /* Fast Down-Counters, are deceremnted through the Timerinterrupt, if >0 */
volatile uint8_t SDC[_NUM_SDC];   /* Slow Down-Counters, are decremented by _SDC_MTintern, if >0 */
volatile uint8_t Event[_NUM_EVT]; /* Event-Flags, are set by Event and signal the occurrence of Events*/

volatile static uint16_t mMtkTimer;     // Timer für Zeitverzögerung
static uint8_t mWaitIdx;      // Index der Warte-Ursache

static t_ctrl_struct mt_timerCtrlLst[MT_MAXTIMERS];
static volatile t_res mt_timerdelay;				// count down until next service
static uint8_t mt_timerfirst;				// point to first entry


// is continously called from "mt_Robin" and manages :
// - executing functions related to elapsed timers
// - checking for reinsertion continous timers
// - updating mt_timerdelay with ticks_delta after elapsing

void mt_timertick(void){
	t_ctrl_struct *p;					// for faster access

//	cli();
	ENTER_CRITICAL_SECTION
	while( mt_timerdelay == 0 ){				// serve if delay = 0
	    if( mt_timerfirst == MT_T_LAST )		// no function to serve
			break;
		p = &mt_timerCtrlLst[mt_timerfirst];
		mt_timerfirst = p->next;										// point to next
		p->next = MT_T_FREE;											// mark free
		mt_timerdelay = mt_timerCtrlLst[mt_timerfirst].ticks_delta;		// next delay delta

		if (p->ticks)													// one-shot timer?
    		mt_timeradd(p->func, p->ticks, 0);
	
		p->func();														// execute function
  	}
	LEAVE_CRITICAL_SECTION
//	sei();

}

// add a new timer to the scheduler system
// the function call requires:
// - a function pointer to the function to be executed
// - the delaytime in _MTZ_XXX
// - and an information about "one shot timer" or "continous" timer


uint8_t mt_timeradd( funcp func, t_res delay, uint8_t flags ){
  uint8_t n;
  uint8_t i;					// index
  uint8_t ipre;					// previous index
  t_res d1;					// last delta delay
  t_ctrl_struct *p;			// for faster access

  for( n = 0;; ){
    p = &mt_timerCtrlLst[n];
    if( p->next == MT_T_FREE )					// if "next = MT_T_FREE" then THIS array element is available
      break;
    n++;
    if( n == MT_MAXTIMERS )
      return 1;                                 // error, list full
  }

  if (flags & MT_T_ONESHOT)						// take care of one-shot flag here, as delay will have changed later
  	p->ticks = 0;
  else
  	p->ticks = delay;


  i = mt_timerfirst;
  d1 = mt_timerdelay;
  while( i != MT_T_LAST ){				// check until end
    if( d1 >= delay ){ 	  	      		// last >= new
      mt_timerCtrlLst[i].ticks_delta = d1 - delay;		// correct following entry
      break;
    }else{
      delay -= d1;				// remaining delay
      ipre = i;                         	// previous entry
      i = mt_timerCtrlLst[i].next;			// index of next entry
      d1 = mt_timerCtrlLst[i].ticks_delta; 		// next delay delta
    }
  }						// insert new entry

  p->next = i;					// following entry
  p->ticks_delta= delay;				// store remaining delay
  p->func = func;

  if( i == mt_timerfirst ){ 				// insert at first
    mt_timerfirst = n;
    mt_timerdelay = delay;
  }else{
    mt_timerCtrlLst[ipre].next = n;			// previous entry
  }
  return 0;					// successful
}


// remove timer by the function handle that would be executed
// if more than one timer were given the same function pointer,
// only the most recent timer will be removed

uint8_t mt_timerremove( funcp func ){
  uint8_t ipre;					// previous index
  uint8_t irem;					// index to be removed
  uint8_t ifol = mt_timerfirst;				// following index
  t_ctrl_struct *p;			// for faster access

  do{
    if( ifol == MT_T_LAST )	
      return 1;					// not found
    ipre = irem;
    irem = ifol;
    p = &mt_timerCtrlLst[irem];
    ifol = p->next;                             // get next
  }while( p->func != func );                    // found it

  p->next = MT_T_FREE;                             // mark it as free
  if( irem == mt_timerfirst ){
    mt_timerfirst = ifol;                           	// serve next entry
    mt_timerdelay += mt_timerCtrlLst[ifol].ticks_delta;		// correct current delta
  }else{
    mt_timerCtrlLst[ipre].next = ifol;              	// skip index
    if( ifol != MT_T_LAST )                   	// correct following delta
      mt_timerCtrlLst[ifol].ticks_delta += p->ticks_delta;
  }
  return 0;					// successful
}

/************************************************************************/
/*  Name:     mt_Robin                                                  */
/*  Funktion: erzwingt Taskwechsel                                      */
/*  Input:    none                                                      */
/*  Output:   none                                                      */
/*  Anm.:                                                               */
/************************************************************************/
void mt_Robin(void)
{
	mt_timertick();
}

/************************************************************************/
/*  Name:     mt_TWait                                                  */
/*  Funktion: akt. Task wartet bestimmte Zeit                           */
/*  Input:    t = Zeit in MT-Ticks                                      */
/*  Output:   none                                                      */
/*  Anm.:                                                               */
/************************************************************************/
void mt_TWait(uint16_t t)
{
//	char output[20];

	mMtkTimer = t;					// wielange
	do{
		mt_Robin();
	}
	while (mMtkTimer != 0);			// Zeit warten
}

/************************************************************************/
/*	Name:     mt_EWait                                                  */
/*	Funktion: akt. Task wartet auf Ereignis                             */
/*	Input:    e = Eventindex (0, 1, 2, usw)                             */
/*	Output:   none                                                      */
/*	Anm.:                                                               */
/************************************************************************/
void mt_EWait(uint8_t e)
{
	mWaitIdx = e;
	do{
		mt_Robin();
		}
	while (mt_TestEvent(e) == 0);		// auf Ereignis warten
	mt_ResEvent(e);   
}

/**
This funtions is used inside a task to wait on an event with timeout. Programm execution continous 
Input:    e = Eventindex (0, 1, 2 usw) , t = Zeit in MT-Ticks       
Output:   none                                                      
Anm.:                                                               
*/
void mt_ETWait(uint8_t e, uint16_t t)
{
	mMtkTimer = t;                 // max. wielange
	mWaitIdx = e;
	do{
		mt_Robin();
	}
	while (mMtkTimer != 0 && mt_TestEvent(e) == 0);	// Zeit warten auf Ereignis
}

/************************************************************************/
/*  Name:     mt_ETimeout                                               */
/*  Funktion: Prüft ob Zeit abgelaufen war                              */
/*            Aufruf ist nur nach dem Befehl mt_ETWait sinnvoll      	*/
/*  Input:    none                                                      */
/*  Output:   boolean  1 = Timeout, 0 = Interrupt                       */
/*  Anm.:                                                               */
/************************************************************************/
uint8_t mt_ETimeout(void)
{
	if (mMtkTimer == 0)					// Zähler abgelaufen ?
	{
		if (mt_TestEvent(mWaitIdx))		// wurde doch noch gesetzt ?
		{
			mt_ResEvent(mWaitIdx);		// Event rücksetzen
			return(0);					// kein TimeOut
		}
		else return(1);					// TimeOut
	}
	mt_ResEvent(mWaitIdx);				// Event rücksetzen
	return(0);							// kein TimeOut
}

/************************************************************************
*  Name:		mt_TimerInt                                               
*  Funktion:	Interrupt function to allow multitasking  using Timer0 Overflow interrupt
*  Input:		none                                                      
*  Output:	none                                                    
*  Anm.:		
************************************************************************/

ISR(TIMER0_COMP_vect)
{

	uint8_t i;				// Loop counter

	if (mMtkTimer)				/* läuft der MTK-Down-Counter ? */
		mMtkTimer--;				/* Down-Counter dekrementieren */

	if (mt_timerdelay)				// 
		mt_timerdelay--;

	for (i = 0;i < _NUM_FDC; i++)
	{
		if (FDC[i])					/* läuft der Down-Counter ? */
			FDC[i]--;				/* Down-Counter dekrementieren */
	}

	if (FDC[_FDC_MTintern] == 0)	/* Zeitpunkt zur Bearbeitung der langsamen Down-Counter erreicht? */
	{
		FDC[_FDC_MTintern] = MTC_1s;    /* Zeit neu starten */
		for (i = 0; i < _NUM_SDC; i++) 		/* Alle langsamen Down-Counter bearbeiten */
		{
			if (SDC[i])				/* läuft der Timer ? */
				SDC[i]--;			/* Down-Counter dekrementieren */
		}
		mt_Event(_EVT_SDC);  		// SDC-Aktualisierungs-Event erzeugen  
//		Debug("MTC_ISR\n");
	}
}
                             
/************************************************************************/
/*  Name:     mt_Init                                                   */
/*  Funktion: Initialisiert den Multitasking-Kernel                     */
/*  Input:    none                                                      */
/*  Output:   none                                                      */
/*  Anm.:                                                               */
/************************************************************************/
void mt_Init(void)
{
	uint8_t i;          /* Schleifenzähler */

	mMtkTimer = 0;

	for (i=0; i < _NUM_FDC; i++)
		FDC[i] = 0;     /* alle schnellen Down-Counter initialisieren */

	for (i=0; i < _NUM_SDC; i++)
		SDC[i] = 0;     /* alle langsamen Down-Counter initialisieren */

	FDC[_FDC_MTintern] = MTC_1s; /* Zeitintervall für LDC starten */

	for (i=0; i< _NUM_EVT; i++)
		Event[i] = 0;   /* alle Events initialisieren */

	for(i = MT_MAXTIMERS; i; i-- )
	   mt_timerCtrlLst[i-1].next = MT_T_FREE;		   // mark all free

	mt_timerfirst = MT_T_LAST; 				   // set no timer served



	// Output Compare Register (OCR0) wird auf 240 gesetzt.
	// Damit ergibt sich mit bei einem Prescaler von 1024
	// alle 20ms ein CTC-Event.
	OCR0 = 192;

	// Timer/Counter0 CompareMatchInterrupt einschalten.
	// Weiter oben ist bereits die zugehörige ISR definiert worden.
	TIMSK |= (1<<OCIE0);	

	// TimerCounter0 arbeitet im CTC (Clear Timer on Compare) Mode.
	// (WGM00=1 und WGM01=1)
	// Es findet keine Portausgabe statt (COM00=0 und COM01=1)	
	// Prescaler des TimerCounter0 wird auf 128 gestellt
	// bei 4.9152 MHz ergeben sich so 38400 ticks pro Sekunde.
	// Ein 8-bit Counter, der bis 192 zählt, läuft somit 200x pro Sekunde über
	// Mit dem setzen des Prescalers startet der Timer.

	TCCR0 = (1<<FOC0 | 0<<WGM00 | 0<<COM01 | 0<<COM00 | 1<<WGM01 | 1<<CS02 | 0<<CS01 | 1<<CS00);
	
	#if DEBUG
		Debug("mt_Init() done.\n");
	#endif		

}

