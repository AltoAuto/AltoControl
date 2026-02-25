#include "IIRdf1filt.h"

IIRdf1filt::IIRdf1filt(int N, const double* ain, const double* bin)
{
	// n is filter order plus 1
	n = N;

	// initialize filter coefficients
	for (int i = 0; i < N; i++) {
		a.push_back(ain[i]);
		b.push_back(bin[i]);
		xr.push_back(0);       // initialize the x shift register to zeros
		yr.push_back(0);       // initialize the y shift register to zeros
	}
}

double IIRdf1filt::FilterSample(float s) 
{
	double x = 0;
	double y = 0;
	double out = 0; // output

	for (int j = n - 1; j > 0; j--) {
		yr[j] = yr[j - 1];
		xr[j] = xr[j - 1];

		y = y - a[j] * yr[j];
		x = x + b[j] * xr[j];
	}
	x = x + b[0] * s;
	out = y + x;
	
	xr[0] = s;
	yr[0] = out;

	return(out);
}

IIRdf1filt::~IIRdf1filt() {
	a.clear();
	b.clear();
}



