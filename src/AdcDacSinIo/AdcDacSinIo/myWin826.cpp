#include "myWin826.h"


// Helper: Read data from one slot
int AdcReadSlot(uint board, uint slot, int *slotdata)
{
	int adcbuf[16];
	uint slotlist = 1 << slot;                                                      // set up the adc slot list; we are only interested in one slot
	int errcode = S826_AdcRead(board, adcbuf, NULL, &slotlist, 1000);                     // wait for data to arrive on the slot of interest (in response to adc hardware trigger)
	if (errcode == S826_ERR_MISSEDTRIG)      // this application doesn't care about adc missed triggers
		errcode = S826_ERR_OK;
	*slotdata = adcbuf[slot] & 0xFFFF;                                                       // copy adc data to slotdata buffer

	return errcode;
}

