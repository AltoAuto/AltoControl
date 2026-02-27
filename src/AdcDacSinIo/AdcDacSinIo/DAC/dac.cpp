///////////////////////////////////////////////////////////////////////////////////////////////
// Sample progrqam for DAC. 2021 by Hamid 
///////////////////////////////////////////////////////////////////////////////////////////////

#include "../myWin826.h"
#include "../826api.h"

// IO card configuration constants
#define IO_BOARD_NUM      0
#define DAC_CHANNEL       7					    // SET ME! CHECK YOUR SETUP: DAC channel output
#define DAC_ZERO_OUTPUT   0x8000
#define DAC_CONFIG_GAIN   S826_DAC_SPAN_10_10   
#define DAC_VRANGE        20.0                  // This MUST match the DAC_SPAN just above
#define DAC_CNT_RANGE     0xFFFF				// 16-bit DAC
#define DAC_OFFSET_COUNTS 0x8000                // 32768 offset for a signed desired output mapped to unsigned DAC write.

int main()
{
	int errcode = S826_ERR_OK;
	int boardflags = S826_SystemOpen();        // open 826 driver and find all 826 boards
	
	double voltageout = 0.0;
	uint dacout;
	char key = '0';

	X826(S826_DacRangeWrite(IO_BOARD_NUM, DAC_CHANNEL, DAC_CONFIG_GAIN, 0));

	while (key != 'q')        //  repeat until q is pressed
	{
		printf("Enter voltage: \n");
		scanf_s("%lf", &voltageout);

		dacout = (uint)(voltageout * (DAC_CNT_RANGE / DAC_VRANGE) + DAC_OFFSET_COUNTS);

		X826(S826_DacDataWrite(IO_BOARD_NUM, DAC_CHANNEL, dacout,0 ));   // output dac sample - cast to short to get sign
		printf("\nPress a key to continue, 'q' for exit\n");
		key = _getch();
	}

	X826(S826_DacDataWrite(IO_BOARD_NUM, DAC_CHANNEL, DAC_ZERO_OUTPUT , 0));   // set DAC output to zero on close
	S826_SystemClose();
	return 0;
}
