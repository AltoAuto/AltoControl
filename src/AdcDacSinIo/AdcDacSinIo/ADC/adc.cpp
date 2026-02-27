
///////////////////////////////////////////////////////////////////////////////////////////////
// ADC file: Reading Analog signal. 2021 by Hamid
///////////////////////////////////////////////////////////////////////////////////////////////

#include "../myWin826.h"
#include "../826api.h"


#define IO_BOARD_NUM  0						  // no change
#define ADC_SLOT      0						  // no change

#define ADC_CHANNEL  1                        // SET ME! CHECK YOUR SETUP: analog input channel


#define ADC_CNT_RANGE 0xFFFF			      // 2^16= 0xFFFF (16 bit converter) 
#define ADC_GAIN     S826_ADC_GAIN_1          
#define ADC_VRANGE   20.0          // this must correspond to gain setting



int main()
{
	int errcode = S826_ERR_OK;
	int boardflags = S826_SystemOpen();        // open 826 driver and find all 826 boards

	int slotdata;               // adc data from the slot of interest
	char key = '0';


	// Configure interfaces and start them running.
	X826(S826_AdcSlotConfigWrite(IO_BOARD_NUM, ADC_SLOT, ADC_CHANNEL, 0, ADC_GAIN));  // program adc timeslot attributes: slot, chan, 0us settling time. For -5V-5V use: "S826_ADC_GAIN_2"
	X826(S826_AdcSlotlistWrite(IO_BOARD_NUM, 1 << ADC_SLOT, S826_BITWRITE));  // enable adc timeslot; disable all other slots
	X826(S826_AdcEnableWrite(IO_BOARD_NUM, 1));  // enable adc conversions
	
	while (key != 'q')                                          // repeat until q is presseed
	{
		printf("Press any key to read ADC value, press 'q' to quit \n");
		key = _getch();
		X826(AdcReadSlot(IO_BOARD_NUM, ADC_SLOT, &slotdata));          // wait for adc data sample
		signed short int sample = slotdata;
		printf("%d \n", sample);
		printf("%3.3fV \n", ADC_VRANGE*sample/ADC_CNT_RANGE);
	}

	X826(S826_AdcEnableWrite(IO_BOARD_NUM, 0));  // halt adc conversions

	S826_SystemClose();
	return 0;
}



