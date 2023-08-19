#ifndef MAPDATA_H
#define MAPDATA_H
#include <vector>
#include <string>
#include <fstream>
#include <malloc.h>
#include <vector>
#include <random>
#include <chrono>
#include <stack>
#include <map>

using namespace std;

// @Config number of cars
int turn = 10;
// @10,20,30,40,50 cars respectively
int carNum = 50;
// @Config arrival interval
// @every 10,20,30,40,50 second arrives a car
int interval = 300;
// @Config number of junctions passed
// @pass 1,2,3,4,5 junctions
int junNum = 5;
// @Config average speed: mean of normal distribution
// @high speed = 8; low speed = 4; 
int meanSpeed = 4;
bool poissonDis = false;

struct roadSq
{
	int roadNum = 0; //road number
	int cooX, cooY;  //road square x/y coordinate
	int seq = 0;  //junction sequence
	int roadType;  //road square type: Edge=1, Junction entrance=2, Junction exit = 3, Normal=0
};

struct car
{
	int index;  //car index
	int time; //the timestamp of showing up in the map
	int maxSpeed;  //max speed of the car
	string route;  //car route
};

class randomTrips {
public:
	static vector<roadSq> ReadMapData(const char* fileName);    //read map data
};

#endif