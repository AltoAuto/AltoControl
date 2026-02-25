#pragma once
#include <vector>

using namespace std;
class IIRdf1filt
{
public:
	IIRdf1filt(int N, const double* ain, const double* bin);
	~IIRdf1filt();

	double FilterSample(float s);

private:
	vector<double> a;
	vector<double> b;
	vector<double> xr;
	vector<double> yr;
	int n;
};

