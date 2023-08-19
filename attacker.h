#ifndef ATTACKER_H
#define ATTACKER_H

#include <map>
#include "math.h"
#include <fstream>
#include <iostream>
#include "mapData.h"
#include "carData.h"
#include "kalman.h"
#include <numeric>
#include <ctime>
#include <cstdlib>

using namespace std;
KalmanFilter kalman;
vector<beacon> prePool; //store beacons before entering junctions
vector<beacon> intPool; //store beacons inside junctions
vector<beacon> rnnPool; //store beacons after RNN prediction
bool fakeBeaconAllowed;

int initialArrival = 2;
int initialTrial = 1;

//junction infomation
double junc1[4][2] = { {10, 0},{0, 10},{-10, 0},{0, -10} };
double junc2[3][2] = { {10, 0},{2.59, 9.66},{-10, 0} };
double junc3[3][2] = { {10, 0},{5, 8.66},{-5, -8.66} };
double junc4[4][2] = { {10, 0},{0, 10},{-9.6, -3.42},{7.07, -7.07} };
double junc5[5][2] = { {10, 0},{0, 10},{-9.85, 1.74},{-8.19, -5.74}, {7.66, -6.43} };

#endif
