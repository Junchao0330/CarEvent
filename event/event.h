#ifndef EVENT_H
#define EVENT_H

#include <map>
#include "math.h"
#include <fstream>
#include <iostream>
#include "mapData.h"
#include "carData.h"
#include "kalman.h"
#include "fakeBeacon.h"
#include <numeric>
#include <ctime>

using namespace std;

#define MAX 500000

//event type
enum eventType { carArrive, carLeave, carPlan, carMove};  //event type
enum scheme {mixZoneScheme, fakeBeaconScheme, advancedFakeBeaconScheme, minFakeBeaconScheme}; //pseudonym changing shceme


class Event
{
public:
	Event() {};
	Event(int i, double aTime, int cX, int cY, int spd, eventType _eType) //car arrive event construction
	{
		carIndex = i;
		time = aTime;
		speed = spd;
		eType = _eType;
		coorX = cX;
		coorY = cY;
	} 

	Event(int i, double aTime, int pX, int pY, int cX, int cY, int maxSpd, int spd, eventType _eType) //car plan/move event construction
	{
		carIndex = i;
		time = aTime;
		mspeed = maxSpd;
		speed = spd;
		eType = _eType;
		coorX = cX;
		coorY = cY;
		preX = pX;
		preY = pY;
	}

	Event(int i, int cX, int cY, double lTime, eventType _eType) //car leave event construction
	{
		carIndex = i;
		leaveTime = lTime;
		eType = _eType;
		coorX = cX;
		coorY = cY;
	}

	double getTime() { return time; }
	int getCarIndex() { return carIndex; }
	static int getX(vector<roadSq> s, int r);
	static int getY(vector<roadSq> s, int r);
	eventType  getEType() { return eType; }

	int carIndex;
	int coorX, coorY; //coordinate of the car
	int preX, preY;  //last coordinate of the car
	int mspeed;
	int speed;        //car set speed
	double time;         //car arrive time to new square
	//double preTime;     //car depart time from last square
	double leaveTime;    //car leaves the map
	eventType eType;  //event type
};



//event list class
class EventList
{
private:
	Event* events;
	int count;  //list length
public:
	EventList()
	{
		events = new Event[MAX];
		count = 0;
	}
	void addArrivalEvent(vector<car> &cc, vector<roadSq> &sq);          //add event to the list
	void addFakeArrivalEvent(car fc, vector<roadSq>& sq);  //add fake car info
    void carPlanEvent(Event e, vector<car> &cc, vector<roadSq> sq, double t);  //normal car move
	void carMoveEvent(Event e, vector<car> &cc, vector<roadSq> sq, double t);  //obstruct in front

	vector<carState> composeState(double pt, double ct, double bt, int px, int py, int cx, int cy);
	beacon createBeacon(double time, int pseudo, double cx, double cy, double vx, double vy);    //create beacons
	RSUMsg createRSUMsg(double time, int carID, int pseudo, double cx, double cy, double vx, double vy); //create rsu messages
	bool enteredMixZone(int cx, int cy, vector<roadSq> sq);   //check if the car entered a mix-zone
	bool enteringMixZone(int cx, int cy, vector<roadSq> sq);   //check if the car is going to enter a mix-zone
	Event deleteEvent();                  //The event with minimum time given top priority to proceed

	bool isEmpty() { return count == 0; }  //isEmpty
	//bool isFull() { return count == MAX; }  //isFull
	int length() { return count; }          //get the length

};


//function

#endif