///////////////////////////////////////////////////////////////////////////////////////////////
// SINEIO  ->  Lab 3 Prelab #6: Derivative of ADC0 written to DAC0
///////////////////////////////////////////////////////////////////////////////////////////////

#include <time.h>
#include <math.h>
#include "../myWin826.h"
#include "../826api.h"
#include "../RealTime.h"

#define SAMPLE_RATE  750      // REQUIRED

// IO card configuration constants
#define IO_BOARD_NUM  0
#define ADC_SLOT      0

#define DAC_ZERO_OUTPUT 0x8000
#define DAC_CONFIG_GAIN S826_DAC_SPAN_10_10   // -10V to +10V
#define DAC_VRANGE    20.0                    // matches DAC span above
#define DAC_CNT_RANGE 0xFFFF                  // 16-bit DAC
#define DAC_OFFSET_COUNTS 0x8000

#define DAC_CHANNEL   7       // SET ME (your DAC0 channel)
#define ADC_CHANNEL   1       // SET ME (your ADC0 channel)

#define ADC_CNT_RANGE 0xFFFF
#define ADC_GAIN     S826_ADC_GAIN_1          // -10V to +10V
#define ADC_VRANGE   20.0

#define ADC_ENABLE 1


// Pick gain so output stays within +/-10V.b
// For 20 Hz, 1 Vrms sine: peak dx/dt ~ 178 V/s.
// If want ~5 V peak output: gain ~ 5/178 = 0.028.
#define DERIV_GAIN   0.028   // [seconds] effectively; output_V = DERIV_GAIN * (dV/dt)

static double clamp(double v, double vmin, double vmax)
{
    if (v < vmin) return vmin;
    if (v > vmax) return vmax;
    return v;
}

int main3()
{
    int  errcode = S826_ERR_OK;
    int  boardflags = S826_SystemOpen();
    int  slotdata;
    int  ncount = 0;
    uint dacout;

    double voltagein = 0.0;
    double voltageout = 0.0;

    const double T = 1.0 / (double)SAMPLE_RATE;   // sample period

    // State for derivative
    double x_prev = 0.0;
    int first = 1;

    RealTime realTime(SAMPLE_RATE);

    // ADC setup
    X826(S826_AdcSlotConfigWrite(IO_BOARD_NUM, ADC_SLOT, ADC_CHANNEL, 0, ADC_GAIN));
    X826(S826_AdcSlotlistWrite(IO_BOARD_NUM, 1 << ADC_SLOT, S826_BITWRITE));
    X826(S826_AdcEnableWrite(IO_BOARD_NUM, ADC_ENABLE));

    // DAC setup
    X826(S826_DacRangeWrite(IO_BOARD_NUM, DAC_CHANNEL, DAC_CONFIG_GAIN, 0));

    realTime.Start();

    while (!_kbhit())
    {
        // Read ADC sample
        X826(AdcReadSlot(IO_BOARD_NUM, ADC_SLOT, &slotdata));
        signed short int adcin = slotdata;

        // ADC counts -> volts (x[n])
        voltagein = (double)(adcin * ADC_VRANGE) / ADC_CNT_RANGE;

        if (first)
        {
            // no derivative on first sample; initialize state
            x_prev = voltagein;
            voltageout = 0.0;
            first = 0;
        }
        else
        {
            // Discrete derivative: dx/dt ≈ (x[n] - x[n-1]) / T
            const double dxdt = (voltagein - x_prev) / T;

            // Scale derivative to a DAC-friendly voltage
            voltageout = DERIV_GAIN * dxdt;

            x_prev = voltagein;
        }

        // Clamp to DAC range (-10V .. +10V)
        voltageout = clamp(voltageout, -DAC_VRANGE / 2.0, DAC_VRANGE / 2.0);

        // Volts -> DAC counts
        dacout = (uint)(voltageout * (DAC_CNT_RANGE / DAC_VRANGE) + DAC_OFFSET_COUNTS);

        // Output to DAC
        X826(S826_DacDataWrite(IO_BOARD_NUM, DAC_CHANNEL, dacout, 0));

        realTime.Sleep();
        ncount++;
    }

    realTime.Stop(ncount);

    X826(S826_DacDataWrite(IO_BOARD_NUM, DAC_CHANNEL, DAC_ZERO_OUTPUT, 0));
    S826_SystemClose();

    return 0;
}
