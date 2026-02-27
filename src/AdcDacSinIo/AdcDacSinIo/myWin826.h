#ifndef _MYADC1
#define _MYADC1

#ifndef _LINUX
#include <windows.h>
#include <conio.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#include "826api.h"

// Helpful macros for DIOs
#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's boolean state from uint[2] array


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ERROR HANDLING
// These examples employ very simple error handling: if an error is detected, the example functions will immediately return an error code.
// This behavior may not be suitable for some real-world applications but it makes the code easier to read and understand. In a real
// application, it's likely that additional actions would need to be performed. The examples use the following X826 macro to handle API
// function errors; it calls an API function and stores the returned value in errcode, then returns immediately if an error was detected.

#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode;}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Counter utility functions used by the demos.

// PERIODIC TIMER --------------------

// Counter mode: count down at 1MHz, preload upon start or when 0 reached, assert ExtOut when not 0
#define TMR_MODE  (S826_CM_K_1MHZ | S826_CM_UD_REVERSE | S826_CM_PX_ZERO | S826_CM_PX_START | S826_CM_OM_NOTZERO)


// PWM GENERATOR -----------------------

// Counter mode: count down at 1MHz, preload when 0 reached, use both preload registers, assert ExtOut when Preload1 is active
#define PWM_MODE  (S826_CM_K_1MHZ | S826_CM_UD_REVERSE | S826_CM_PX_ZERO | S826_CM_PX_START | S826_CM_BP_BOTH | S826_CM_OM_PRELOAD)

//#define PWM_MODE 0x01682020

// Configure a counter channel to operate as a pwm generator and start it running.

// Halt channel operating as pwm generator.


// Configure a counter channel to operate as a periodic timer and start it running.
extern int PeriodicTimerStart(uint board, uint counter, uint period);

// Halt channel operating as periodic timer.
extern int PeriodicTimerStop(uint board, uint counter);

// Wait for periodic timer event.
extern int PeriodicTimerWait(uint board, uint counter, uint *timestamp);

// Helper: Read data from one slot
extern int AdcReadSlot(uint board, uint slot, int *slotdata);

// The demo
extern int ADCfun(uint board);


#endif // !MYADC1

