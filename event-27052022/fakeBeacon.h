#ifndef FAKEBEACON_H
#define FAKEBEACON_H

#include "kalman.h"
#include "event.h"

using namespace Eigen;
using namespace std;

class fakeBeacon {
public:
	vector<roadSq> fmp;
	vector<car> fcr;
	vector<beacon> fbeaconPool;
	vector<beacon> trafficSimulate(car c);
	vector<beacon> possibleMeasurement(vector<RSUMsg> msg, vector<roadSq> rsq);
};
#endif