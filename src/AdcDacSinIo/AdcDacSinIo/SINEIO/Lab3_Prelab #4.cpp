///////////////////////////////////////////////////////////////////////////////////////////////
// SINEIO  ->  Lab 3 Prelab #4: 2nd-order IIR low-pass via double filtering
///////////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <time.h>
#include "../myWin826.h"
#include "../826api.h"
#include "../RealTime.h"

#define SAMPLE_RATE  750      // MUST be 750 Hz for Lab 3
#define FC_HZ        20.0     // cutoff frequency (Hz)

// IO card configuration constants
#define IO_BOARD_NUM  0
#define ADC_SLOT      0

#define DAC_ZERO_OUTPUT 0x8000
#define DAC_CONFIG_GAIN S826_DAC_SPAN_10_10
#define DAC_VRANGE    20.0
#define DAC_CNT_RANGE 0xFFFF
#define DAC_OFFSET_COUNTS 0x8000

#define DAC_CHANNEL   7       // SET ME
#define ADC_CHANNEL   1       // SET ME

#define ADC_CNT_RANGE 0xFFFF
#define ADC_GAIN     S826_ADC_GAIN_1
#define ADC_VRANGE   20

#define ADC_ENABLE 1

static double clamp(double v, double vmin, double vmax)
{
    if (v < vmin) return vmin;
    if (v > vmax) return vmax;
    return v;
}

int main2()
{
    int  errcode = S826_ERR_OK;
    int  boardflags = S826_SystemOpen();
    int  slotdata;
    int  ncount = 0;
    uint dacout;

    double voltagein = 0.0;
    double voltageout = 0.0;

    // Second order coefficent 
    const double A = 0.771587023;
    const double B = 0.228413;

    // --- Double-filter state ---
    // Stage 1 output: y[n]
    double y_prev = 0.0;
    // Stage 2 output: z[n] (this is your 2nd-order output)
    double z_prev = 0.0;

    int first = 1;

    RealTime realTime(SAMPLE_RATE);

    X826(S826_AdcSlotConfigWrite(IO_BOARD_NUM, ADC_SLOT, ADC_CHANNEL, 0, ADC_GAIN));
    X826(S826_AdcSlotlistWrite(IO_BOARD_NUM, 1 << ADC_SLOT, S826_BITWRITE));
    X826(S826_AdcEnableWrite(IO_BOARD_NUM, ADC_ENABLE));

    X826(S826_DacRangeWrite(IO_BOARD_NUM, DAC_CHANNEL, DAC_CONFIG_GAIN, 0));

    realTime.Start();

    while (!_kbhit())
    {
        // Read ADC
        X826(AdcReadSlot(IO_BOARD_NUM, ADC_SLOT, &slotdata));
        signed short int adcin = slotdata;

        // Counts -> volts (x[n])
        voltagein = (double)(adcin * ADC_VRANGE) / ADC_CNT_RANGE;

        // Initialize to avoid dumb startup transient
        if (first) {
            y_prev = voltagein;   // y(0) = x(0)
            z_prev = voltagein;   // z(0) = x(0) (or y(0); either is fine)
            first = 0;
        }

        // ------------------------------
        // Double filtering (2nd order):
        // y[n] = A*y[n-1] + B*x[n]
        // z[n] = A*z[n-1] + B*y[n]
        // output = z[n]
        // ------------------------------
        const double y = A * y_prev + B * voltagein;  // stage 1
        const double z = A * z_prev + B * y;          // stage 2

        y_prev = y;
        z_prev = z;

        voltageout = z;

        // Clamp to DAC range (-10..+10)
        voltageout = clamp(voltageout, -DAC_VRANGE / 2.0, DAC_VRANGE / 2.0);

        // Volts -> DAC counts
        dacout = (uint)(voltageout * (DAC_CNT_RANGE / DAC_VRANGE) + DAC_OFFSET_COUNTS);

        // Write DAC
        X826(S826_DacDataWrite(IO_BOARD_NUM, DAC_CHANNEL, dacout, 0));

        realTime.Sleep();
        ncount++;
    }

    realTime.Stop(ncount);

    X826(S826_DacDataWrite(IO_BOARD_NUM, DAC_CHANNEL, DAC_ZERO_OUTPUT, 0));
    S826_SystemClose();

    return 0;
}
