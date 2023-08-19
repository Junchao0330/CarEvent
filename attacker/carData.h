#ifndef CARDATA_H
#define CARDATA_H
#include <vector>
#include <fstream>
#include <malloc.h>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

struct car
{
	int index;
	double showupTime; //the timestamp of showing up in the map
	double maxSpeed;
	double currentSpeed = 0;
	double arriveTime; //the timestamp of arriving at a new square
	string route;
	int currentX = 0;
	int currentY = 0;
	int pseudonym;
	double beaconTime = 0; 
	int passedJunc = 0;
	bool isActive = false;
};

struct carState
{
	double realTime;
	double realX;
	double realY;
};

struct beacon
{
	double timestamp;
	int pseudonym;
	double currX;
	double currY;
	double speedX;
	double speedY;
};

struct RSUMsg
{
	double timestamp;
	int carIndex;
	int pseudonym;
	double currX;
	double currY;
	double speedX;
	double speedY;
};

struct pairTime
{
	int mixZoneIndex;
	double enterTime;
	double leaveTime;
};

class CarData {
public:
	vector<car> ReadCarData(string fileName);    //read car data to event list to initiate
	int getCurrentRoad(string s);
	string restRoad(string s);
};
#endif