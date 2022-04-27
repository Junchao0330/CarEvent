#ifndef EVENT_H
#define EVENT_H

#include <map>
#include "math.h"
#include <fstream>
#include <iostream>
#include "mapData.h"
#include "carData.h"
#include "kalman.h"

using namespace std;

#define MAX 500000

//event type
enum eventType { carArrive, carLeave, carPlan, carMove, sendBeacon };  //event type
vector<beacon> beaconPool;   //store all beacons (only contains car pseudonym)
vector<RSUMsg> RSUPool;    //store all rephrased beacons (contain both car index and pseudonym)



class Event
{
public:
	Event() {};
	Event(int i, float aTime, int cX, int cY, int spd, eventType _eType) //car arrive event construction
	{
		carIndex = i;
		time = aTime;
		speed = spd;
		eType = _eType;
		coorX = cX;
		coorY = cY;
	} 

	Event(int i, float aTime, int pX, int pY, int cX, int cY, int maxSpd, int spd, eventType _eType) //car plan/move event construction
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

	Event(int i, int cX, int cY, float lTime, eventType _eType) //car leave event construction
	{
		carIndex = i;
		leaveTime = lTime;
		eType = _eType;
		coorX = cX;
		coorY = cY;
	}

	float getTime() { return time; }
	int getCarIndex() { return carIndex; }
	static int getX(vector<roadSq> s, int r);
	static int getY(vector<roadSq> s, int r);
	eventType  getEType() { return eType; }

	int carIndex;
	int coorX, coorY; //coordinate of the car
	int preX, preY;  //last coordinate of the car
	int mspeed;
	int speed;        //car set speed
	float time;         //car arrive time to new square
	//float preTime;     //car depart time from last square
	float leaveTime;    //car leaves the map
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
    void carPlanEvent(Event e, vector<car> &cc, vector<roadSq> sq, float t);  //normal car move
	void carMoveEvent(Event e, vector<car> &cc, vector<roadSq> sq, float t);  //obstruct in front

	beacon createBeacon(float time, int pseudo, int cx, int cy, float vx, float vy);    //create beacons
	RSUMsg createRSUMsg(float time, int carID, int pseudo, int cx, int cy, float vx, float vy); //create rsu messages
	bool enterMixZone(int cx, int cy, vector<roadSq> sq);   //check if the car enters a mix-zone
	Event deleteEvent();                  //The event with minimum time given top priority to proceed

	bool isEmpty() { return count == 0; }  //isEmpty
	//bool isFull() { return count == MAX; }  //isFull
	int length() { return count; }          //get the length

};

vector<roadSq> mp;
vector<car> cr;
//function

#endif