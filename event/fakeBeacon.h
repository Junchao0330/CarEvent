#ifndef FAKEBEACON_H
#define FAKEBEACON_H

#include "kalman.h"
#include "event.h"
#include <random>
#include <algorithm>
#include <iterator>

using namespace Eigen;
using namespace std;


class fakeBeacon {
public:
	vector<roadSq> fmp;
	vector<car> fcr;
	vector<beacon> fbeaconPool;
	vector<beacon> trafficSimulate(car ca);
	car newCar(RSUMsg msg, int cx, int cy, double ti, double sp, int mspeed, vector<roadSq> rsq);
	vector<car> newCarGroup(RSUMsg msg, int cx, int cy, double ti, double sp, int mspeed, vector<roadSq> rsq);
};
#endif