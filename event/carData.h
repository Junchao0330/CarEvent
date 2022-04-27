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
	float showupTime; //the timestamp of showing up in the map
	float maxSpeed;
	float currentSpeed = 0;
	float arriveTime; //the timestamp of arriving at a new square
	string route;
	int currentX = 0;
	int currentY = 0;
	int pseudonym;
};

struct beacon
{
	float timestamp;
	int pseudonym;
	int currX;
	int currY;
	float speedX;
	float speedY;
};

struct RSUMsg
{
	float timestamp;
	int carIndex;
	int pseudonym;
	int currX;
	int currY;
	float speedX;
	float speedY;
};

class CarData {
public:
	vector<car> ReadCarData(const char* fileName);    //read car data to event list to initiate
	int getCurrentRoad(string s);
	string restRoad(string s);
};
#endif