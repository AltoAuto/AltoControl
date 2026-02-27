///////////////////////////////////////////////////////////////////////////////////////////////
// SINEIO  ->  Lab 3 Prelab #2: 1st-order IIR low-pass
///////////////////////////////////////////////////////////////////////////////////////////////

#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include "../myWin826.h"
#include "../826api.h"
#include "../RealTime.h"

#define SAMPLE_RATE  750      // MUST be 750 Hz for Lab 3

// Filter specs
#define FC_HZ  20.0           // cutoff frequency (Hz)

// IO card configuration constants
#define IO_BOARD_NUM  0						  // no change
#define ADC_SLOT      0						  // no change

#define DAC_ZERO_OUTPUT 0x8000
#define DAC_CONFIG_GAIN S826_DAC_SPAN_10_10   // -10 to 10 spans 20 volts so...
#define DAC_VRANGE    20.0                    // This MUST correspond to the DAC_SPAN just above
#define DAC_CNT_RANGE 0xFFFF				  // 16-bit DAC
#define DAC_OFFSET_COUNTS 0x8000              // 32768 offset for a signed desired output mapped to unsigned DAC write.

#define DAC_CHANNEL   7						  // SET ME! CHECK YOUR SETUP: DAC channel output
#define ADC_CHANNEL   1                       // SET ME! CHECK YOUR SETUP: analog input channel to track

#define ADC_CNT_RANGE 0xFFFF				  // 16-bit converter
#define ADC_GAIN     S826_ADC_GAIN_1          // -10 to 10 option
#define ADC_VRANGE   20                       // must correspond to gain setting

#define ADC_ENABLE 1

static double clamp(double v, double vmin, double vmax)
{
	if (v < vmin) return vmin;
	if (v > vmax) return vmax;
	return v;
}

int main()
{
	int  errcode = S826_ERR_OK;
	int  boardflags = S826_SystemOpen();        // open 826 driver and find all 826 boards
	int  slotdata;               // adc data from the slot of interest
	int  ncount = 0;
	uint dacout;

	double voltagein = 0.0;
	double voltageout = 0.0;

	// First order A and B value
	const double A = 0.8460633876;
	const double B = 0.1539366124;

	// --- Filter state y[n-1] ---
	double y_prev = 0.0;
	int first = 1;

	// instantiate our "real time" object that will pace our loop
	RealTime realTime(SAMPLE_RATE);

	// Configure data acquisition interfaces and start them running.
	X826(S826_AdcSlotConfigWrite(IO_BOARD_NUM, ADC_SLOT, ADC_CHANNEL, 0, ADC_GAIN));
	X826(S826_AdcSlotlistWrite(IO_BOARD_NUM, 1 << ADC_SLOT, S826_BITWRITE));
	X826(S826_AdcEnableWrite(IO_BOARD_NUM, ADC_ENABLE));

	X826(S826_DacRangeWrite(IO_BOARD_NUM, DAC_CHANNEL, DAC_CONFIG_GAIN, 0));

	// commence pseudo-real-time loop.
	realTime.Start();

	while (!_kbhit())
	{
		// read ADC sample
		X826(AdcReadSlot(IO_BOARD_NUM, ADC_SLOT, &slotdata));
		signed short int adcin = slotdata;

		// convert ADC counts -> volts
		voltagein = (double)(adcin * ADC_VRANGE) / ADC_CNT_RANGE;

		// optional: start "settled" to avoid a big transient at the beginning
		if (first) { y_prev = voltagein; first = 0; }

		// ------------------------------
		// 1st-order IIR low-pass filter:
		// y[n] = A*y[n-1] + B*x[n]
		// ------------------------------
		voltageout = A * y_prev + B * voltagein;
		y_prev = voltageout;

		// clamp to DAC range to avoid wraparound garbage
		voltageout = clamp(voltageout, -DAC_VRANGE / 2.0, DAC_VRANGE / 2.0);

		// volts -> DAC counts
		dacout = (uint)(voltageout * (DAC_CNT_RANGE / DAC_VRANGE) + DAC_OFFSET_COUNTS);

		// output to the DAC
		X826(S826_DacDataWrite(IO_BOARD_NUM, DAC_CHANNEL, dacout, 0));

		realTime.Sleep();
		ncount++;
	}

	realTime.Stop(ncount);

	X826(S826_DacDataWrite(IO_BOARD_NUM, DAC_CHANNEL, DAC_ZERO_OUTPUT, 0));
	S826_SystemClose();

	return 0;
}
