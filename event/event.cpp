#include "event.h"
#include <fstream>
#include <iostream>


using namespace std;
MapData md;
CarData cd;
EventList eList;
KalmanFilter kalman;
fakeBeacon fb;

vector<roadSq> mp;
vector<car> cr;
vector<beacon> beaconPool;   //store all beacons (only contains car pseudonym)
vector<RSUMsg> RSUPool;    //store all rephrased beacons (contain both car index and pseudonym)


int Event::getX(vector<roadSq> s, int r) {
	for (int j = 0; j < s.size(); j++) {
		if (s[j].roadNum == r) {
			return s[j].cooX;
		}
	}
}

int Event::getY(vector<roadSq> s, int r) {
	for (int j = 0; j < s.size(); j++) {
		if (s[j].roadNum == r) {
			return s[j].cooY;
		}
	}
}

//add arrival event to the list 
void EventList::addArrivalEvent(vector<car> &cc, vector<roadSq> &sq)
{
	int roadStart = 0;
	int tX, tY;
	for (int i = 0; i < cc.size(); i++) {
		roadStart = cd.getCurrentRoad(cc[i].route); //start road
		cc[i].route = cd.restRoad(cc[i].route); //update rest of the route
		cc[i].arriveTime = cc[i].showupTime;
		tX = Event::getX(sq, roadStart);
		tY = Event::getY(sq, roadStart);
		Event evt(cc[i].index, cc[i].showupTime, tX, tY, cc[i].maxSpeed, carArrive);
		events[count] = evt;
		count++;
		//return cc;
	}
	
}

void EventList::addFakeArrivalEvent(car fc, vector<roadSq>& sq)
{
	int roadStart = 0;
	int tX, tY;
		roadStart = stoi(fc.route); //start road
		tX = fc.currentX;
		tY = fc.currentY;
		Event evt(fc.index, fc.arriveTime, tX, tY, fc.maxSpeed, carArrive);
		events[count] = evt;
		count++;
		//return cc;
	

}

//normal car move
void EventList::carPlanEvent(Event e, vector<car> &cc, vector<roadSq> sq, double t) {
	int nextX, nextY;
	double estArriTime;
	int tempid = -1;

	for (int j = 0; j < cc.size(); j++) {
		if (cc[j].index == e.carIndex) {
			tempid = j;
		}
	}
	
	for (int i = 0; i < sq.size(); i++) {
		if (e.coorX == sq[i].cooX && e.coorY == sq[i].cooY) {  
			if (cc[tempid].route == "") {  //it is the last road segment of the route
				if (sq[i].roadNum != sq[i+1].roadNum) {  //and it is the last step of the road
					Event leaveE(e.carIndex, e.coorX, e.coorY, t + 0.001, carLeave);
					events[count] = leaveE;
					count++;
				}
				else { //set normal move plan
					nextX = sq[i + 1].cooX;
					nextY = sq[i + 1].cooY;
					if (cc[tempid].currentSpeed < cc[tempid].maxSpeed) {
						if (cc[tempid].currentSpeed + 1 > cc[tempid].maxSpeed) {
							cc[tempid].currentSpeed = cc[tempid].maxSpeed;
						}
						else {
							cc[tempid].currentSpeed += 1;
						}
					}
					estArriTime = t + sqrt(pow((nextX - e.coorX), 2) + pow((nextY - e.coorY), 2)) / cc[tempid].currentSpeed;
					Event p(e.carIndex, estArriTime, e.coorX, e.coorY, nextX, nextY, cc[tempid].maxSpeed, cc[tempid].currentSpeed, carPlan);
					events[count] = p;
					count++;
				}
			}
			else {  //there are other roads in the rest of the route
				if (sq[i].roadNum != sq[i + 1].roadNum) {  //the car needs to change route
					nextX = Event::getX(sq, cd.getCurrentRoad(cc[tempid].route));
					nextY = Event::getY(sq, cd.getCurrentRoad(cc[tempid].route));
					cc[tempid].route = cd.restRoad(cc[tempid].route); //update rest of the route
					estArriTime = t + sqrt(pow((nextX - e.coorX), 2) + pow((nextY - e.coorY), 2)) / cc[tempid].currentSpeed;
					Event p(e.carIndex, estArriTime, e.coorX, e.coorY, nextX, nextY, cc[tempid].maxSpeed, cc[tempid].currentSpeed, carPlan);
					events[count] = p;
					count++;
				}
				else { //set normal move plan
					nextX = sq[i + 1].cooX;
					nextY = sq[i + 1].cooY;
					if ((sq[i + 3].roadType == 2)&&(cc[tempid].currentSpeed > 1.67)){ //minimum speed for passing junctions is approximately 1
						cc[tempid].currentSpeed = 0.6* cc[tempid].currentSpeed;
					}
					else if ((sq[i + 2].roadType == 2) && (cc[tempid].currentSpeed > 1.67)) {
						cc[tempid].currentSpeed = 0.6 * cc[tempid].currentSpeed;
					}
					else if ((sq[i + 1].roadType == 2) && (cc[tempid].currentSpeed > 1.67)) {
						cc[tempid].currentSpeed = 0.6 * cc[tempid].currentSpeed;
					}
					else {
						if (cc[tempid].currentSpeed < cc[tempid].maxSpeed) {
							if (cc[tempid].currentSpeed + 1 > cc[tempid].maxSpeed) {
								cc[tempid].currentSpeed = cc[tempid].maxSpeed;
							}
							else {
								cc[tempid].currentSpeed += 1;
							}
						}
					}
					estArriTime = t + sqrt(pow((nextX - e.coorX), 2) + pow((nextY - e.coorY), 2)) / cc[tempid].currentSpeed;
					Event p(e.carIndex, estArriTime, e.coorX, e.coorY, nextX, nextY, cc[tempid].maxSpeed, cc[tempid].currentSpeed, carPlan);
					events[count] = p;
					count++;
				}
			}
		}
		
	}
}

void EventList::carMoveEvent(Event e, vector<car> &cc, vector<roadSq> sq, double t) {
	double tryTime;
	int tempi = e.carIndex;
	for (int i = 0; i < sq.size(); i++) {
		if (e.coorX == sq[i].cooX && e.coorY == sq[i].cooY) { 
		      if (!sq[i].isOcpd) {  //next square not occupied
					Event move(e.carIndex, e.time + 0.0001, e.preX, e.preY, e.coorX, e.coorY, e.mspeed, e.speed, carMove);
					events[count] = move;
					count++;
			}
				else if (sq[i].isOcpd) { //next square occupied
					tryTime = e.time + 0.2;  //try to move 0.1s later (same parameter in SUMO)
					Event delayE(e.carIndex, tryTime, e.preX, e.preY, e.coorX, e.coorY, e.mspeed, e.speed, carPlan);
					events[count] = delayE;
					count++;
				}
			}
		  
	}
}



Event EventList::deleteEvent()  //The event with earliest timestamp given top priority
{
	if (!isEmpty())
	{
		double min = events[0].getTime();
		int minindex = 0;

		for (int i = 1; i < count; i++)
		{
			if (events[i].getTime() < min)
			{
				min = events[i].getTime();
				minindex = i;//find min time event
			}
		}
		Event et = events[minindex];
		events[minindex] = events[count - 1];
		count--;
		return et;
	}
	else
		throw("delete error");
}

vector<carState> EventList::composeState(double pt, double ct, double bt, int px, int py, int cx, int cy)
{
	vector<carState> tempState;
	vector<double> w;
	double i = bt;
	while (i <= ct) {
		if (i > pt) {
			w.push_back(i);
			i = i + 0.5;  //broadcast beacon every 0.5 sec
		}
		else { i = i + 0.5; }
	}
	for (int j = 0; j < w.size(); j++) {
		double realx, realy;
		carState cs;
		realx = (w[j] - pt) * (cx - px) / (ct - pt) + px;
		realy = (w[j] - pt) * (cy - py) / (ct - pt) + py;
		cs.realTime = w[j];
		cs.realX = realx;
		cs.realY = realy;
		tempState.push_back(cs);
	}
	return tempState;
}

beacon EventList::createBeacon(double time, int pseudo, double cx, double cy, double vx, double vy) //create beacon
{
	beacon bcn;
	bcn.timestamp = time;
	bcn.pseudonym = pseudo;
	bcn.currX = cx;
	bcn.currY = cy;
	bcn.speedX = vx;
	bcn.speedY = vy;
	return bcn;
}

RSUMsg EventList::createRSUMsg(double time, int carID, int pseudo, double cx, double cy, double vx, double vy)  //create RSU message
{
	RSUMsg msg;
	msg.timestamp = time;
	msg.carIndex = carID;
	msg.pseudonym = pseudo;
	msg.currX = cx;
	msg.currY = cy;
	msg.speedX = vx;
	msg.speedY = vy;
	return msg;
}

bool EventList::enteredMixZone(int cx, int cy, vector<roadSq> sq)
{
	for (int i = 0; i < sq.size(); i++) {
		if (cx == sq[i].cooX && cy == sq[i].cooY) {
			if (sq[i].roadType == 3) {
				return true;
			}
			else {
				return false;
			}
				
			
		}

	}
}

bool EventList::enteringMixZone(int cx, int cy, vector<roadSq> sq)
{
	for (int i = 0; i < sq.size(); i++) {
		if (cx == sq[i].cooX && cy == sq[i].cooY) {
			if (sq[i].roadType == 2) {
				return true;
			}
			else {
				return false;
			}


		}

	}
}

int main() //main function
{
	fstream out;  //start log file
	fstream fout;
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////CONFIGURATION///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
	scheme schemeType = mixZoneScheme;  //config pseudonym changing scheme
	int fakeBeaconNum = 0;  //how many fake beacons are generated, i.e.,how much resource we use
	int fakeRouteNum = 0;  //how many fake routes are generated, i.e.,how much resource we use
	if (schemeType == fakeBeaconScheme) {  //deploy fake beacon scheme
		fout.open("C:/Users/wangj/source/repos/SimulationResults_fake.txt", ios_base::app);
	}
	else if (schemeType == mixZoneScheme) {  //deploy traditional mix-zone scheme
		fout.open("C:/Users/wangj/source/repos/SimulationResults.txt", ios_base::app);
	}
	else if (schemeType == advancedFakeBeaconScheme) {  //deploy advanced fake beacon scheme
		fout.open("C:/Users/wangj/source/repos/SimulationResults_advfake.txt", ios_base::app);
	}
	else if (schemeType == minFakeBeaconScheme) {  //deploy advanced fake beacon scheme
		fout.open("C:/Users/wangj/source/repos/SimulationResults_minfake.txt", ios_base::app);
	}
	mp = md.ReadMapData("C:/Users/wangj/source/repos/ScottsvilleMap.txt");  //load map info
    //main programme 
	vector<double> lifeTimes;
	vector<vector<int>> density;
	int MaxJunctionPass = 5;  //Max number of junctions
	int MaxTestNum = 9;  //Max index of test cases, starting from 0
	int carNum = 20;
	int interval = 150;
	int cn = 20;
	while (cn <= carNum) {
		int iv = 150;
		while (iv <= interval) {
			int jn = 5;
			while (jn <= MaxJunctionPass) {
				int name = 0;
				while (name <= MaxTestNum) {
					vector<int> activeCars;
					vector<double> arriveT;
					vector<double> leaveT;
					int arrt = 1;
					while (arrt <= (cn-1) * interval + 1) {
						arriveT.push_back(arrt);
						arrt += interval;
					}
					cr = cd.ReadCarData("C:/Users/wangj/source/repos/carFileInterval/" + to_string(cn) + "V" + to_string(iv) + "T/" + to_string(jn) + "junc/route_" + to_string(name) + ".txt");  //load car info

					//open test input files
					if (schemeType == fakeBeaconScheme) {  //deploy fake beacon scheme
						//out.open("C:/Users/wangj/source/repos/carresults-fake/" + to_string(cn) + "V" + to_string(iv) + "T/" + to_string(jn) + "juncroute_" + to_string(name) + "output.txt", ios::out);
						out.open("C:/Users/wangj/source/repos/resultforintervals/" + to_string(cn) + "V_" + to_string(iv) + "T_" + to_string(jn) + "junc_route" + to_string(name) + "_output.txt", ios::out);
					}
					else if (schemeType == mixZoneScheme) {  //deploy traditional mix-zone scheme
						//out.open("C:/Users/wangj/source/repos/carresults-mixzone/" + to_string(cn) + "V" + to_string(iv) + "T/" + to_string(jn) + "juncroute_" + to_string(name) + "output.txt", ios::out);
						out.open("C:/Users/wangj/source/repos/resultforintervals/" + to_string(cn) + "V_" + to_string(iv) + "T_" + to_string(jn) + "junc_route" + to_string(name) + "_output.txt", ios::out);
					}
					else if (schemeType == advancedFakeBeaconScheme) {  //deploy advanced fake beacon scheme
						out.open("C:/Users/wangj/source/repos/carresults-advfake/" + to_string(cn) + "V" + to_string(iv) + "T/" + to_string(jn) + "juncroute_" + to_string(name) + "output.txt", ios::out);
					}
					else if (schemeType == minFakeBeaconScheme) {  //deploy advanced fake beacon scheme
						out.open("C:/Users/wangj/source/repos/carresults-minfake/" + to_string(cn) + "V" + to_string(iv) + "T/" + to_string(jn) + "juncroute_" + to_string(name) + "output.txt", ios::out);
					}
					/////////////////////////////////////////////////////////////////////////////////////////
					/////////////////////////////////////////////////////////////////////////////////////////
					/////////////////////////////////////////////////////////////////////////////////////////
					int junNum = md.getJunctionNumber(mp);  //number of junctions
					eList.addArrivalEvent(cr, mp);    //add car arrive events
					double currentTime = 0; //simulation starts
					int tempi = -1;

					int carID;
					out << "t = 0:  Simulation begins" << endl;   //write data to file

					//start traffic simulation
					while (!eList.isEmpty())
					{
						Event tempE = eList.deleteEvent();
						currentTime = tempE.getTime();
						carID = tempE.getCarIndex();

						//the sequence number in car vector
						for (int j = 0; j < cr.size(); j++) {
							if (cr[j].index == tempE.carIndex) {
								tempi = j;
							}
						}
						if (tempE.getEType() == carArrive)          //Process carArrive event
						{
							cr[tempi].isActive = true;
							cr[tempi].currentX = tempE.coorX;
							cr[tempi].currentY = tempE.coorY;
							cr[tempi].beaconTime = 0.0001 * cr[tempi].index;
							for (int j = 0; j < mp.size(); j++) {
								if (tempE.coorX == mp[j].cooX && tempE.coorY == mp[j].cooY) {
									mp[j].isOcpd = true;  //delete if overtake allowed
								}
							}
							out << "t = " << currentTime << ", car " << carID << " arrives at " << tempE.coorX << "," << tempE.coorY << endl;
							beacon Beacon = eList.createBeacon(tempE.getTime(), cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, 0, 0); //dedicate beacon
							beaconPool.push_back(Beacon); //start to "broadcast" beacon
							RSUMsg Beacon_Id = eList.createRSUMsg(tempE.getTime(), cr[tempi].index, cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, 0, 0); //dedicate RSU message
							RSUPool.push_back(Beacon_Id); //start to log RSU message
							eList.carPlanEvent(tempE, cr, mp, currentTime);
						}
						else if (tempE.getEType() == carLeave) {    //Process carLeave event
							leaveT.push_back(tempE.leaveTime);
							cr[tempi].isActive = false;
							cr[tempi].currentX = tempE.coorX;
							cr[tempi].currentY = tempE.coorY;
							for (int i = 0; i < mp.size(); i++) {
								if (tempE.coorX == mp[i].cooX && tempE.coorY == mp[i].cooY) {
									mp[i].isOcpd = false;  //release the last stepstone the car occupied before discard
								}
							}
							out << "t = " << tempE.leaveTime << ", car " << carID << " leaves at " << tempE.coorX << "," << tempE.coorY << endl;
						}
						else if (tempE.getEType() == carPlan) {     //Process carPlan event
							eList.carMoveEvent(tempE, cr, mp, currentTime);
						}
						else if (tempE.getEType() == carMove) {     //Process carMove event
							//if the car has entered a mix-zone, it updates its pseudonym, and keep silent
							if (eList.enteredMixZone(tempE.coorX, tempE.coorY, mp)) {
								cr[tempi].pseudonym += 1;//change pseudonym
								cr[tempi].currentX = tempE.coorX;
								cr[tempi].currentY = tempE.coorY;
								for (int i = 0; i < mp.size(); i++) {
									if (tempE.coorX == mp[i].cooX && tempE.coorY == mp[i].cooY) {  //set arrived setpstone as occupied and release previous one
										mp[i].isOcpd = true;  //delete if overtake allowed
									}
									if (tempE.preX == mp[i].cooX && tempE.preY == mp[i].cooY) {  //set arrived setpstone as occupied and release previous one
										mp[i].isOcpd = false;
									}
								}
								eList.carPlanEvent(tempE, cr, mp, tempE.time + 0.0001); //proceed new plan after the movement
								out << "t = " << tempE.getTime() << ", car " << carID << " goes to " << tempE.coorX << "," << tempE.coorY << endl;

								double realSpeed = sqrt(pow((tempE.coorY - tempE.preY), 2) + pow((tempE.coorX - tempE.preX), 2)) / (currentTime - cr[tempi].arriveTime);

								//only proceed beacon and RSUMsg once when the car leaves the junction 
								beacon Beacon_Psu = eList.createBeacon(tempE.getTime(), cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, cos(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed, sin(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed); //dedicate beacon (only contains pseudonym)
								RSUMsg Beacon_Id = eList.createRSUMsg(tempE.getTime(), cr[tempi].index, cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, cos(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed, sin(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed); //dedicate RSU message (contains car ID and pseudonym)
								beaconPool.push_back(Beacon_Psu); //start to "broadcast" beacon
								RSUPool.push_back(Beacon_Id); //log RSU message
								cr[tempi].arriveTime = currentTime;
								cr[tempi].beaconTime = currentTime - floor(currentTime); //change the time of broadcast since this is a new pseudonym
							}
							else {
								cr[tempi].currentX = tempE.coorX;
								cr[tempi].currentY = tempE.coorY;
								for (int i = 0; i < mp.size(); i++) {
									if (tempE.coorX == mp[i].cooX && tempE.coorY == mp[i].cooY) {  //set arrived setpstone as occupied and release previous one
										mp[i].isOcpd = true;  //delete if overtake allowed
									}
									if (tempE.preX == mp[i].cooX && tempE.preY == mp[i].cooY) {  //set arrived setpstone as occupied and release previous one
										mp[i].isOcpd = false;
									}
								}
								eList.carPlanEvent(tempE, cr, mp, tempE.time + 0.0001); //proceed new plan after the movement
								out << "t = " << tempE.getTime() << ", car " << carID << " goes to " << tempE.coorX << "," << tempE.coorY << endl;

								double realSpeed = sqrt(pow((tempE.coorY - tempE.preY), 2) + pow((tempE.coorX - tempE.preX), 2)) / (currentTime - cr[tempi].arriveTime);

								//proceed as many beacons as required for this piece of carMove
								vector<carState> states = eList.composeState(cr[tempi].arriveTime, currentTime, cr[tempi].beaconTime, tempE.preX, tempE.preY, tempE.coorX, tempE.coorY);
								for (int i = 0; i < states.size(); i++) {
									beacon Beacon_PSU = eList.createBeacon(states[i].realTime, cr[tempi].pseudonym, states[i].realX, states[i].realY, cos(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed, sin(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed); //dedicate beacon (only contains pseudonym)
									RSUMsg Beacon_ID = eList.createRSUMsg(states[i].realTime, cr[tempi].index, cr[tempi].pseudonym, states[i].realX, states[i].realY, cos(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed, sin(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed); //dedicate RSU message (contains car ID and pseudonym)
									beaconPool.push_back(Beacon_PSU); //start to "broadcast" beacon
									RSUPool.push_back(Beacon_ID); //log RSU message
								}
								cr[tempi].arriveTime = currentTime;


								if (eList.enteringMixZone(tempE.coorX, tempE.coorY, mp)) { //if approaching a junction/mix-zone
									cr[tempi].passedJunc += 1;
									if (schemeType == fakeBeaconScheme) {  //deploy fake beacon algorithm
										vector<RSUMsg> whichCar;  //collect RSUMsg sent from this car
										for (int i = 0; i < RSUPool.size(); i++) {
											if (RSUPool[i].carIndex == tempE.carIndex) {
												whichCar.push_back(RSUPool[i]);
											}
										}
										//make sure we have the initial state of the target car
										int firstIndex = 0;
										for (int i = 0; i < whichCar.size(); i++) {
											if (whichCar[i].timestamp >= whichCar[firstIndex].timestamp) {
												firstIndex = i;
											}
										}
										car fakeCar = fb.newCar(whichCar[firstIndex], tempE.coorX, tempE.coorY, tempE.time, tempE.speed, tempE.mspeed, mp); //compose fake car
										vector<beacon> fakeBeacons = fb.trafficSimulate(fakeCar); //generate fake beacons
										beaconPool.insert(beaconPool.end(), fakeBeacons.begin(), fakeBeacons.end());  //store fake beacons in the beacon pool
										//fakeBeaconNum += fakeBeacons.size();
										//fakeRouteNum++;
									}
									else if (schemeType == advancedFakeBeaconScheme) {  //deploy fake beacon algorithm
										vector<RSUMsg> whichCar;  //collect RSUMsg sent from this car
										for (int i = 0; i < RSUPool.size(); i++) {
											if (RSUPool[i].carIndex == tempE.carIndex) {
												whichCar.push_back(RSUPool[i]);
											}
										}
										//make sure we have the initial state of the target car
										int firstIndex = 0;
										for (int i = 0; i < whichCar.size(); i++) {
											if (whichCar[i].timestamp >= whichCar[firstIndex].timestamp) {
												firstIndex = i;
											}
										}
										if ((cr[tempi].passedJunc > 1) && (cr[tempi].passedJunc < 4)) { //pass 2-3 junctions
											car fakeCar = fb.newCar(whichCar[firstIndex], tempE.coorX, tempE.coorY, tempE.time, tempE.speed, tempE.mspeed, mp); //compose fake car
											vector<beacon> fakeBeacons = fb.trafficSimulate(fakeCar); //generate fake beacons
											beaconPool.insert(beaconPool.end(), fakeBeacons.begin(), fakeBeacons.end());  //store fake beacons in the beacon pool
											fakeBeaconNum += fakeBeacons.size();
											fakeRouteNum++;
										}
										else if (cr[tempi].passedJunc <= 1) { //pass junction for the first time
											vector<car> fakeCarGroup = fb.newCarGroup(whichCar[firstIndex], tempE.coorX, tempE.coorY, tempE.time, tempE.speed, tempE.mspeed, mp);
											for (int i = 0; i < fakeCarGroup.size(); i++) {
												car fakeCar = fakeCarGroup[i];
												vector<beacon> fakeBeacons = fb.trafficSimulate(fakeCar); //generate fake beacons
												beaconPool.insert(beaconPool.end(), fakeBeacons.begin(), fakeBeacons.end());  //store fake beacons in the beacon pool
												fakeBeaconNum += fakeBeacons.size();
												fakeRouteNum++;
											}
										}
										//don't compose fake cars if passing too many junctions (more than 3)
									}
									else if (schemeType == minFakeBeaconScheme) {  //deploy fake beacon algorithm
										if (cr[tempi].passedJunc < 4) {
											vector<RSUMsg> whichCar;  //collect RSUMsg sent from this car
											for (int i = 0; i < RSUPool.size(); i++) {
												if (RSUPool[i].carIndex == tempE.carIndex) {
													whichCar.push_back(RSUPool[i]);
												}
											}
											//make sure we have the initial state of the target car
											int firstIndex = 0;
											for (int i = 0; i < whichCar.size(); i++) {
												if (whichCar[i].timestamp >= whichCar[firstIndex].timestamp) {
													firstIndex = i;
												}
											}
											car fakeCar = fb.newCar(whichCar[firstIndex], tempE.coorX, tempE.coorY, tempE.time, tempE.speed, tempE.mspeed, mp); //compose fake car
											vector<beacon> fakeBeacons = fb.trafficSimulate(fakeCar); //generate fake beacons
											beaconPool.insert(beaconPool.end(), fakeBeacons.begin(), fakeBeacons.end());  //store fake beacons in the beacon pool
											fakeBeaconNum += fakeBeacons.size();
											fakeRouteNum++;
										}
										//don't compose fake cars if passing too many junctions (more than 3)
									}
								}
							}
						}
					}

					out << "simulation ends" << endl;    //traffic simulation ends

					
					//the following code is for calculating vehicles' lifetime only
					
					double aveLifeTTime = 0;
					for (int i = 0; i < cr.size(); i++) {
						aveLifeTTime += cr[i].arriveTime - cr[i].showupTime;
					}
					aveLifeTTime /= cr.size();
					lifeTimes.push_back(aveLifeTTime); //average lifetime for this scenario
					
					
					////////////////////////////////////////////////////////////////////////////////////
					////////////start tracking algorithm using collected beacons////////////////////////
					////////////////////////////////////////////////////////////////////////////////////
					
					map<int, vector<beacon>> multihyposis;  //key = pseudonym. value = vector of beacons
					for (int i = 0; i < beaconPool.size(); i++) {
						multihyposis[beaconPool[i].pseudonym].push_back(beaconPool[i]); //beacons with same pseudonym go to same vector
					}

					map<int, vector<RSUMsg>> carTrace;  //key = car index, value = vector of RSU msgs
					for (int i = 0; i < RSUPool.size(); i++) {
						carTrace[RSUPool[i].carIndex].push_back(RSUPool[i]); //car traces classified by their index
					}

					map<int, vector<beacon>> endPoint; //trace segments of the same junction to drive in;
					map<int, vector<beacon>> startPoint; //trace segments of the same junction to drive out;
					map<int, vector<beacon>> endTen; //endPoint trace segments with last 10 beacons;
					vector<beacon> measurement; //first beacon in startPoint trace segments;
					vector<proTable> probab;  //probability table
					vector<proTable> tempTable;

					//start to traversal all junctions
					int junSeq = 1;
					for (junSeq = 1; junSeq <= junNum; junSeq++) {
						for (map<int, vector<beacon>>::iterator it = multihyposis.begin(); it != multihyposis.end(); it++) {
							for (int p = 0; p < mp.size(); p++) {
								for (int k = it->second.size() - 1; k >= 0; k--) {
									if (it->second[k].currX == mp[p].cooX && it->second[k].currY == mp[p].cooY) {
										int tempNum = mp[p].roadNum;
										for (int j = 0; j < mp.size(); j++) {
											if ((mp[j].roadNum == tempNum) && (mp[j].roadType == 2) && (mp[j + 1].roadNum != tempNum) && ((mp[j].seq == junSeq))) {
												endPoint.insert(pair<int, vector<beacon>>(it->first, it->second));
											}
										}
										break;
									}
								}
							}
							for (map<int, vector<beacon>>::iterator itr = multihyposis.begin(); itr != multihyposis.end(); itr++) {
								for (int q = 0; q < mp.size(); q++) {
									if (itr->second.front().currX == mp[q].cooX && itr->second.front().currY == mp[q].cooY) {
										if (mp[q].seq == junSeq) {
											startPoint.insert(pair<int, vector<beacon>>(itr->first, itr->second));
										}
									}
								}
							}
						}

						vector<beacon> tempTen;
						int whichIndex;
						for (map<int, vector<beacon>>::iterator ita = endPoint.begin(); ita != endPoint.end(); ita++) {
							for (int i = 10; i > 0; i--) {
								whichIndex = ita->second.size() - i - 1;
								tempTen.push_back(ita->second[whichIndex]);
							}
							endTen.insert(pair<int, vector<beacon>>(ita->first, tempTen));
							tempTen.clear();
						}

						for (map<int, vector<beacon>>::iterator itb = startPoint.begin(); itb != startPoint.end(); itb++) {
							measurement.push_back(itb->second.front());
						}

						for (map<int, vector<beacon>>::iterator itc = endTen.begin(); itc != endTen.end(); itc++) {
							for (int i = 0; i < measurement.size(); i++) {
								tempTable = kalman.kalmanPrediction(itc->second, measurement[i]);  //create probability table for each pair of end/start trace segments
								probab.insert(probab.end(), tempTable.begin(), tempTable.end()); //add new iteration result to the probability table
							}
						}


						//proceed probability table

						//delete probabilities that are extremely low to enhance efficiency
						int mm = 0;
						int mmsize = probab.size();
						for (mm = 0; mm < mmsize; mm++)
						{
							if (probab[mm].probability < 1e-100) //filter out the segment pair which probability too low (here < 1e-100)
							{
								vector <proTable> ::iterator it = probab.begin() + mm;
								probab.erase(it);
								mmsize = probab.size();
								--mm;
							}
						}


						int nn = 0;
						int maxP = 0;
						while (probab.size() > 0) {
							double tempro = probab.front().probability;
							for (nn = 0; nn < probab.size(); nn++) {
								if (probab[nn].probability > tempro) { //the highest probability
									tempro = probab[nn].probability;
									maxP = nn; //find index which has the highest probability
								}
							}
							int lastPs = probab[maxP].lastPseu;
							int nextPs = probab[maxP].nextPseu;
							//merge a new trace
							vector<beacon> merge;
							map<int, vector<beacon>>::iterator key1 = multihyposis.find(probab[maxP].lastPseu);
							if (key1 != multihyposis.end()) {
								merge.insert(merge.end(), key1->second.begin(), key1->second.end());
								multihyposis.erase(key1);
							}
							else break;
							map<int, vector<beacon>>::iterator key2 = multihyposis.find(probab[maxP].nextPseu);
							if (key2 != multihyposis.end()) {
								for (int i = 0; i < key2->second.size(); i++) {
									key2->second[i].pseudonym = probab[maxP].lastPseu;
								}
								merge.insert(merge.end(), key2->second.begin(), key2->second.end());
								multihyposis.erase(key2);
							}
							else break;
							multihyposis.insert(pair<int, vector<beacon>>(probab[maxP].lastPseu, merge));
							vector <proTable> ::iterator ite = probab.begin() + maxP;
							probab.erase(ite);

							vector <proTable>::iterator itf;
							for (itf = probab.begin(); itf != probab.end(); ) {
								if ((itf->lastPseu == lastPs) || (itf->nextPseu == nextPs)) {
									probab.erase(itf);
									itf = probab.begin();
								}
								else itf++;
							}

							maxP = 0;
						}

						endPoint.clear();
						startPoint.clear();
						probab.clear();
						endTen.clear();
						measurement.clear();
						tempTable.clear();

					}

					//start to calculate success rate
					int successTrace = 0;
					map<int, vector<RSUMsg>>::iterator iter;
					for (iter = carTrace.begin(); iter != carTrace.end(); iter++) {
						int pse = iter->second[0].pseudonym;
						map<int, vector<beacon>>::iterator it = multihyposis.find(pse);
						if (it != multihyposis.end()) {
							vector<RSUMsg> comp1 = iter->second;
							vector<beacon> comp2 = it->second;
							int check = 0;
							for (int i = 0; i < comp1.size(); i++) {
								for (int j = 0; j < comp2.size(); j++) {
									if (comp1[i].timestamp == comp2[j].timestamp) {
										check++;
									}
								}
							}
							if (check == iter->second.size()) successTrace++; //total number of successful trace
						}
					}

					//a map containing mix zone indexs, enter times and leave times of cars' whole journeys
					map<int, vector<pairTime>> mixZoneTimes;
					map<int, vector<RSUMsg>>::iterator itw = carTrace.begin();
					while (itw != carTrace.end()) {
						vector<pairTime> tempMZTimes;
						for (int i = 1; i < itw->second.size(); i++) {
							if ((itw->second[i].timestamp - itw->second[i - 1].timestamp) > 0.5) {
								pairTime pte;
								for (int j = 0; j < mp.size(); j++) {
									if (itw->second[i].currX == mp[j].cooX && itw->second[i].currY == mp[j].cooY) {
										pte.mixZoneIndex = mp[j].seq;
									}
								}
								pte.enterTime = itw->second[i - 1].timestamp;
								pte.leaveTime = itw->second[i].timestamp;
								tempMZTimes.push_back(pte);
							}
						}
						mixZoneTimes.insert(pair<int, vector<pairTime>>(itw->first, tempMZTimes));
						itw++;
					}

					//how many cars does a car encounter in a mix-zone in average
					vector<double> avgTotCarEncounter;
					for (map<int, vector<pairTime>>::iterator it = mixZoneTimes.begin(); it != mixZoneTimes.end(); it++) {
						vector<double> avgCarEncounter;
						for (int i = 0; i < it->second.size(); i++) {
							int targetZone = it->second[i].mixZoneIndex;
							int eTime = it->second[i].enterTime;
							int lTime = it->second[i].leaveTime;
							int carCount = 0;
							for (map<int, vector<pairTime>>::iterator ita = mixZoneTimes.begin(); ita != mixZoneTimes.end(); ita++) {
								if (it->first != ita->first) {
									for (int j = 0; j < ita->second.size(); j++) {
										if ((ita->second[j].mixZoneIndex == targetZone) && (eTime <= ita->second[j].leaveTime) && (ita->second[j].enterTime <= lTime)) {
											carCount++;
										}
									}
								}
							}
							avgCarEncounter.push_back(carCount);
						}
						double avgCar = accumulate(avgCarEncounter.begin(), avgCarEncounter.end(), 0.00) / avgCarEncounter.size();
						avgTotCarEncounter.push_back(avgCar);
					}

					double avgTotCar = accumulate(avgTotCarEncounter.begin(), avgTotCarEncounter.end(), 0.000) / avgTotCarEncounter.size(); //in average, how many cars are encountered in each junction for a car


					vector<int> avgTotJunEncounter;
					double avgTotJun = 0;

					for (map<int, vector<pairTime>>::iterator it = mixZoneTimes.begin(); it != mixZoneTimes.end(); it++) {
						int junEncounter = 0;
						for (int i = 0; i < it->second.size(); i++) {
							int targetZone = it->second[i].mixZoneIndex;
							int eTime = it->second[i].enterTime;
							int lTime = it->second[i].leaveTime;
							for (map<int, vector<pairTime>>::iterator ita = mixZoneTimes.begin(); ita != mixZoneTimes.end(); ita++) {
								bool isEncountered = false;
								if (it->first != ita->first) {
									for (int j = 0; j < ita->second.size(); j++) {
										if ((ita->second[j].mixZoneIndex == targetZone) && (eTime <= ita->second[j].leaveTime) && (ita->second[j].enterTime <= lTime)) {
											junEncounter++;
											isEncountered = true;
											break;
										}
									}
								}
								if (isEncountered) break;
							}
						}

						avgTotJunEncounter.push_back(junEncounter);
					}
					avgTotJun = accumulate(avgTotJunEncounter.begin(), avgTotJunEncounter.end(), 0.000) / avgTotJunEncounter.size();  //in average, how many junctions a car encoutered other cars during its journey




					//check beacon pool
					out << " " << endl;
					out << "BEACONS:" << endl;
					for (int i = 0; i < beaconPool.size(); i++) {
						out << beaconPool[i].timestamp << "\t" << beaconPool[i].pseudonym << "\t" << beaconPool[i].currX << "\t" << beaconPool[i].currY << "\t" << beaconPool[i].speedX << "\t" << beaconPool[i].speedY << endl;
					}

					out << " " << endl;
					out << "RSUMessages:" << endl;
					//check RSU pool
					for (int i = 0; i < RSUPool.size(); i++) {
						out << RSUPool[i].timestamp << "\t" << RSUPool[i].carIndex << "\t" << RSUPool[i].pseudonym << "\t" << RSUPool[i].currX << "\t" << RSUPool[i].currY << "\t" << RSUPool[i].speedX << "\t" << RSUPool[i].speedY << endl;
					}

					out << " " << endl;
					out << "MultiHyposis:" << endl;
					//check MultiHyposis
					for (map<int, vector<beacon>>::iterator ita = multihyposis.begin(); ita != multihyposis.end(); ita++) {
						for (int i = 0; i < ita->second.size(); i++)
							out << ita->second[i].timestamp << " " << ita->second[i].pseudonym << endl;
					}



					//print results
					out << " " << endl;
					out << "average car encouter is " << avgTotCar << endl;
					out << " " << endl;
					out << "average junction encouter is " << avgTotJun << endl;
					out << " " << endl;
					out << "success rate of tracking is " << 100 * successTrace / cr.size() << "%" << endl;

					
					cout << "average car encouter is " << avgTotCar << endl;
					cout << "average junction encouter is " << avgTotJun << endl;
					cout << "success rate of tracking is " << 100 * successTrace / cr.size() << "%" << endl;
					

					//fout << avgTotCar << "\t" << avgTotJun << "\t" << 100 * successTrace / cr.size() << "\t" << fakeBeaconNum << "\t" << fakeRouteNum << endl;
					fout << avgTotCar << "\t" << avgTotJun << "\t" << 100 * successTrace / cr.size() << "\t" << aveLifeTTime << endl;
					
					//close file
					out.close();
					cr.clear();
					beaconPool.clear();   //store all beacons (only contains car pseudonym)
					RSUPool.clear();
					name++;
					int gap = 0;
					while (gap <= 1800) {
						int numA = 0;
						int numL = 0;
						int iii = 0;
						int jjj = 0;
						if (gap >= arriveT.back()) {
							numA = 20;
						}
						else {
							while (arriveT[iii] <= gap) {
								numA++;
								iii++;
							}
						}
						if (gap >= leaveT.back()) {
							numL = 20;
						}
						else {
							while (leaveT[jjj] <= gap) {
								numL++;
								jjj++;
							}
						}
						activeCars.push_back(numA - numL);
						gap += 100;
					}

					density.push_back(activeCars);

				}
				jn++;
			}
			iv += 30;
		}
		cn += 10;
	}

	//the following code is for calaculating vehicles' lifetime only
	
	fout.close();
	double life1 = 0;
	double life2 = 0;
	double life3 = 0;
	double life4 = 0;
	double life5 = 0;
	for (int i = 0; i < lifeTimes.size(); i++) {
		if (i % 50 <= 9) {
			life1 += lifeTimes[i];
		}
		else if (i % 50 >= 10 && i % 50 <= 19) {
			life2 += lifeTimes[i];
		}
		else if (i % 50 >= 20 && i % 50 <= 29) {
			life3 += lifeTimes[i];
		}
		else if (i % 50 >= 30 && i % 50 <= 39) {
			life4 += lifeTimes[i];
		}
		else {
			life5 += lifeTimes[i];
		}
	}
	life1 /= 400;
	life2 /= 400;
	life3 /= 400;
	life4 /= 400;
	life5 /= 400;
	cout << "Average Lifetime for Cars Passing 1 junction is " << life1 << endl;
	cout << "Average Lifetime for Cars Passing 2 junction is " << life2 << endl;
	cout << "Average Lifetime for Cars Passing 3 junction is " << life3 << endl;
	cout << "Average Lifetime for Cars Passing 4 junction is " << life4 << endl;
	cout << "Average Lifetime for Cars Passing 5 junction is " << life5 << endl;

	for (int i = 0; i < density[0].size(); i++) {
		double ress = 0.0;
		for (int j = 0; j < density.size(); j++) {
			ress += density[j][i];
		}
		double resss = ress * 0.1;
		cout << resss << endl;
	}

	return 0;
}

/*
int main() //test main function
{
	fstream out;  //start log file
	fstream fout;
	/////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////CONFIGURATION///////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////
	scheme schemeType = mixZoneScheme;  //config pseudonym changing scheme
	int fakeBeaconNum = 0;  //how many fake beacons are generated, i.e.,how much resource we use
	int fakeRouteNum = 0;  //how many fake routes are generated, i.e.,how much resource we use
	if (schemeType == fakeBeaconScheme) {  //deploy fake beacon scheme
		fout.open("C:/Users/wangj/source/repos/SimulationResults_fake.txt", ios_base::app);
	}
	else if (schemeType == mixZoneScheme) {  //deploy traditional mix-zone scheme
		fout.open("C:/Users/wangj/source/repos/SimulationResults.txt", ios_base::app);
	}
	else if (schemeType == advancedFakeBeaconScheme) {  //deploy advanced fake beacon scheme
		fout.open("C:/Users/wangj/source/repos/SimulationResults_advfake.txt", ios_base::app);
	}
	else if (schemeType == minFakeBeaconScheme) {  //deploy advanced fake beacon scheme
		fout.open("C:/Users/wangj/source/repos/SimulationResults_minfake.txt", ios_base::app);
	}
	mp = md.ReadMapData("C:/Users/wangj/source/repos/ScottsvilleMap.txt");  //load map info
	cr = cd.ReadCarData("C:/Users/wangj/source/repos/test.txt");  //load car info
    out.open("C:/Users/wangj/source/repos/output.txt", ios::out);
					
					/////////////////////////////////////////////////////////////////////////////////////////
					/////////////////////////////////////////////////////////////////////////////////////////
					/////////////////////////////////////////////////////////////////////////////////////////
					int junNum = md.getJunctionNumber(mp);  //number of junctions
					eList.addArrivalEvent(cr, mp);    //add car arrive events
					double currentTime = 0; //simulation starts
					int tempi = -1;

					int carID;
					out << "t = 0:  Simulation begins" << endl;   //write data to file

					//start traffic simulation
					while (!eList.isEmpty())
					{
						Event tempE = eList.deleteEvent();
						currentTime = tempE.getTime();
						carID = tempE.getCarIndex();

						//the sequence number in car vector
						for (int j = 0; j < cr.size(); j++) {
							if (cr[j].index == tempE.carIndex) {
								tempi = j;
							}
						}
						if (tempE.getEType() == carArrive)          //Process carArrive event
						{
							cr[tempi].isActive = true;
							cr[tempi].currentX = tempE.coorX;
							cr[tempi].currentY = tempE.coorY;
							cr[tempi].beaconTime = 0.0001 * cr[tempi].index;
							for (int j = 0; j < mp.size(); j++) {
								if (tempE.coorX == mp[j].cooX && tempE.coorY == mp[j].cooY) {
									mp[j].isOcpd = true;  //delete if overtake allowed
								}
							}
							out << "t = " << currentTime << ", car " << carID << " arrives at " << tempE.coorX << "," << tempE.coorY << endl;
							beacon Beacon = eList.createBeacon(tempE.getTime(), cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, 0, 0); //dedicate beacon
							beaconPool.push_back(Beacon); //start to "broadcast" beacon
							RSUMsg Beacon_Id = eList.createRSUMsg(tempE.getTime(), cr[tempi].index, cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, 0, 0); //dedicate RSU message
							RSUPool.push_back(Beacon_Id); //start to log RSU message
							eList.carPlanEvent(tempE, cr, mp, currentTime);
						}
						else if (tempE.getEType() == carLeave) {    //Process carLeave event
							cr[tempi].isActive = false;
							cr[tempi].currentX = tempE.coorX;
							cr[tempi].currentY = tempE.coorY;
							for (int i = 0; i < mp.size(); i++) {
								if (tempE.coorX == mp[i].cooX && tempE.coorY == mp[i].cooY) {
									mp[i].isOcpd = false;  //release the last stepstone the car occupied before discard
								}
							}
							out << "t = " << tempE.leaveTime << ", car " << carID << " leaves at " << tempE.coorX << "," << tempE.coorY << endl;
						}
						else if (tempE.getEType() == carPlan) {     //Process carPlan event
							eList.carMoveEvent(tempE, cr, mp, currentTime);
						}
						else if (tempE.getEType() == carMove) {     //Process carMove event
							//if the car has entered a mix-zone, it updates its pseudonym, and keep silent
							if (eList.enteredMixZone(tempE.coorX, tempE.coorY, mp)) {
								cr[tempi].pseudonym += 1;//change pseudonym
								cr[tempi].currentX = tempE.coorX;
								cr[tempi].currentY = tempE.coorY;
								for (int i = 0; i < mp.size(); i++) {
									if (tempE.coorX == mp[i].cooX && tempE.coorY == mp[i].cooY) {  //set arrived setpstone as occupied and release previous one
										mp[i].isOcpd = true;  //delete if overtake allowed
									}
									if (tempE.preX == mp[i].cooX && tempE.preY == mp[i].cooY) {  //set arrived setpstone as occupied and release previous one
										mp[i].isOcpd = false;
									}
								}
								eList.carPlanEvent(tempE, cr, mp, tempE.time + 0.0001); //proceed new plan after the movement
								out << "t = " << tempE.getTime() << ", car " << carID << " goes to " << tempE.coorX << "," << tempE.coorY << endl;

								double realSpeed = sqrt(pow((tempE.coorY - tempE.preY), 2) + pow((tempE.coorX - tempE.preX), 2)) / (currentTime - cr[tempi].arriveTime);

								//only proceed beacon and RSUMsg once when the car leaves the junction 
								beacon Beacon_Psu = eList.createBeacon(tempE.getTime(), cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, cos(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed, sin(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed); //dedicate beacon (only contains pseudonym)
								RSUMsg Beacon_Id = eList.createRSUMsg(tempE.getTime(), cr[tempi].index, cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, cos(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed, sin(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed); //dedicate RSU message (contains car ID and pseudonym)
								beaconPool.push_back(Beacon_Psu); //start to "broadcast" beacon
								RSUPool.push_back(Beacon_Id); //log RSU message
								cr[tempi].arriveTime = currentTime;
								cr[tempi].beaconTime = currentTime - floor(currentTime); //change the time of broadcast since this is a new pseudonym
							}
							else {
								cr[tempi].currentX = tempE.coorX;
								cr[tempi].currentY = tempE.coorY;
								for (int i = 0; i < mp.size(); i++) {
									if (tempE.coorX == mp[i].cooX && tempE.coorY == mp[i].cooY) {  //set arrived setpstone as occupied and release previous one
										mp[i].isOcpd = true;  //delete if overtake allowed
									}
									if (tempE.preX == mp[i].cooX && tempE.preY == mp[i].cooY) {  //set arrived setpstone as occupied and release previous one
										mp[i].isOcpd = false;
									}
								}
								eList.carPlanEvent(tempE, cr, mp, tempE.time + 0.0001); //proceed new plan after the movement
								out << "t = " << tempE.getTime() << ", car " << carID << " goes to " << tempE.coorX << "," << tempE.coorY << endl;

								double realSpeed = sqrt(pow((tempE.coorY - tempE.preY), 2) + pow((tempE.coorX - tempE.preX), 2)) / (currentTime - cr[tempi].arriveTime);

								//proceed as many beacons as required for this piece of carMove
								vector<carState> states = eList.composeState(cr[tempi].arriveTime, currentTime, cr[tempi].beaconTime, tempE.preX, tempE.preY, tempE.coorX, tempE.coorY);
								for (int i = 0; i < states.size(); i++) {
									beacon Beacon_PSU = eList.createBeacon(states[i].realTime, cr[tempi].pseudonym, states[i].realX, states[i].realY, cos(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed, sin(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed); //dedicate beacon (only contains pseudonym)
									RSUMsg Beacon_ID = eList.createRSUMsg(states[i].realTime, cr[tempi].index, cr[tempi].pseudonym, states[i].realX, states[i].realY, cos(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed, sin(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed); //dedicate RSU message (contains car ID and pseudonym)
									beaconPool.push_back(Beacon_PSU); //start to "broadcast" beacon
									RSUPool.push_back(Beacon_ID); //log RSU message
								}
								cr[tempi].arriveTime = currentTime;


								if (eList.enteringMixZone(tempE.coorX, tempE.coorY, mp)) { //if approaching a junction/mix-zone
									cr[tempi].passedJunc += 1;
									if (schemeType == fakeBeaconScheme) {  //deploy fake beacon algorithm
										vector<RSUMsg> whichCar;  //collect RSUMsg sent from this car
										for (int i = 0; i < RSUPool.size(); i++) {
											if (RSUPool[i].carIndex == tempE.carIndex) {
												whichCar.push_back(RSUPool[i]);
											}
										}
										//make sure we have the initial state of the target car
										int firstIndex = 0;
										for (int i = 0; i < whichCar.size(); i++) {
											if (whichCar[i].timestamp >= whichCar[firstIndex].timestamp) {
												firstIndex = i;
											}
										}
										car fakeCar = fb.newCar(whichCar[firstIndex], tempE.coorX, tempE.coorY, tempE.time, tempE.speed, tempE.mspeed, mp); //compose fake car
										vector<beacon> fakeBeacons = fb.trafficSimulate(fakeCar); //generate fake beacons
										beaconPool.insert(beaconPool.end(), fakeBeacons.begin(), fakeBeacons.end());  //store fake beacons in the beacon pool
										//fakeBeaconNum += fakeBeacons.size();
										//fakeRouteNum++;
									}
									else if (schemeType == advancedFakeBeaconScheme) {  //deploy fake beacon algorithm
										vector<RSUMsg> whichCar;  //collect RSUMsg sent from this car
										for (int i = 0; i < RSUPool.size(); i++) {
											if (RSUPool[i].carIndex == tempE.carIndex) {
												whichCar.push_back(RSUPool[i]);
											}
										}
										//make sure we have the initial state of the target car
										int firstIndex = 0;
										for (int i = 0; i < whichCar.size(); i++) {
											if (whichCar[i].timestamp >= whichCar[firstIndex].timestamp) {
												firstIndex = i;
											}
										}
										if ((cr[tempi].passedJunc > 1) && (cr[tempi].passedJunc < 4)) { //pass 2-3 junctions
											car fakeCar = fb.newCar(whichCar[firstIndex], tempE.coorX, tempE.coorY, tempE.time, tempE.speed, tempE.mspeed, mp); //compose fake car
											vector<beacon> fakeBeacons = fb.trafficSimulate(fakeCar); //generate fake beacons
											beaconPool.insert(beaconPool.end(), fakeBeacons.begin(), fakeBeacons.end());  //store fake beacons in the beacon pool
											fakeBeaconNum += fakeBeacons.size();
											fakeRouteNum++;
										}
										else if (cr[tempi].passedJunc <= 1) { //pass junction for the first time
											vector<car> fakeCarGroup = fb.newCarGroup(whichCar[firstIndex], tempE.coorX, tempE.coorY, tempE.time, tempE.speed, tempE.mspeed, mp);
											for (int i = 0; i < fakeCarGroup.size(); i++) {
												car fakeCar = fakeCarGroup[i];
												vector<beacon> fakeBeacons = fb.trafficSimulate(fakeCar); //generate fake beacons
												beaconPool.insert(beaconPool.end(), fakeBeacons.begin(), fakeBeacons.end());  //store fake beacons in the beacon pool
												fakeBeaconNum += fakeBeacons.size();
												fakeRouteNum++;
											}
										}
										//don't compose fake cars if passing too many junctions (more than 3)
									}
									else if (schemeType == minFakeBeaconScheme) {  //deploy fake beacon algorithm
										if (cr[tempi].passedJunc < 4) {
											vector<RSUMsg> whichCar;  //collect RSUMsg sent from this car
											for (int i = 0; i < RSUPool.size(); i++) {
												if (RSUPool[i].carIndex == tempE.carIndex) {
													whichCar.push_back(RSUPool[i]);
												}
											}
											//make sure we have the initial state of the target car
											int firstIndex = 0;
											for (int i = 0; i < whichCar.size(); i++) {
												if (whichCar[i].timestamp >= whichCar[firstIndex].timestamp) {
													firstIndex = i;
												}
											}
											car fakeCar = fb.newCar(whichCar[firstIndex], tempE.coorX, tempE.coorY, tempE.time, tempE.speed, tempE.mspeed, mp); //compose fake car
											vector<beacon> fakeBeacons = fb.trafficSimulate(fakeCar); //generate fake beacons
											beaconPool.insert(beaconPool.end(), fakeBeacons.begin(), fakeBeacons.end());  //store fake beacons in the beacon pool
											fakeBeaconNum += fakeBeacons.size();
											fakeRouteNum++;
										}
										//don't compose fake cars if passing too many junctions (more than 3)
									}
								}
							}
						}
					}

					out << "simulation ends" << endl;    //traffic simulation ends

					
					//the following code is for calculating vehicles' lifetime only
					
					double aveLifeTTime = 0;
					for (int i = 0; i < cr.size(); i++) {
						aveLifeTTime += cr[i].arriveTime - cr[i].showupTime;
					}
					aveLifeTTime /= cr.size();
					lifeTimes.push_back(aveLifeTTime); //average lifetime for this scenario
					
					
					////////////////////////////////////////////////////////////////////////////////////
					////////////start tracking algorithm using collected beacons////////////////////////
					////////////////////////////////////////////////////////////////////////////////////
					
					map<int, vector<beacon>> multihyposis;  //key = pseudonym. value = vector of beacons
					for (int i = 0; i < beaconPool.size(); i++) {
						multihyposis[beaconPool[i].pseudonym].push_back(beaconPool[i]); //beacons with same pseudonym go to same vector
					}

					map<int, vector<RSUMsg>> carTrace;  //key = car index, value = vector of RSU msgs
					for (int i = 0; i < RSUPool.size(); i++) {
						carTrace[RSUPool[i].carIndex].push_back(RSUPool[i]); //car traces classified by their index
					}

					map<int, vector<beacon>> endPoint; //trace segments of the same junction to drive in;
					map<int, vector<beacon>> startPoint; //trace segments of the same junction to drive out;
					map<int, vector<beacon>> endTen; //endPoint trace segments with last 10 beacons;
					vector<beacon> measurement; //first beacon in startPoint trace segments;
					vector<proTable> probab;  //probability table
					vector<proTable> tempTable;

					//start to traversal all junctions
					int junSeq = 1;
					for (junSeq = 1; junSeq <= junNum; junSeq++) {
						for (map<int, vector<beacon>>::iterator it = multihyposis.begin(); it != multihyposis.end(); it++) {
							for (int p = 0; p < mp.size(); p++) {
								for (int k = it->second.size() - 1; k >= 0; k--) {
									if (it->second[k].currX == mp[p].cooX && it->second[k].currY == mp[p].cooY) {
										int tempNum = mp[p].roadNum;
										for (int j = 0; j < mp.size(); j++) {
											if ((mp[j].roadNum == tempNum) && (mp[j].roadType == 2) && (mp[j + 1].roadNum != tempNum) && ((mp[j].seq == junSeq))) {
												endPoint.insert(pair<int, vector<beacon>>(it->first, it->second));
											}
										}
										break;
									}
								}
							}
							for (map<int, vector<beacon>>::iterator itr = multihyposis.begin(); itr != multihyposis.end(); itr++) {
								for (int q = 0; q < mp.size(); q++) {
									if (itr->second.front().currX == mp[q].cooX && itr->second.front().currY == mp[q].cooY) {
										if (mp[q].seq == junSeq) {
											startPoint.insert(pair<int, vector<beacon>>(itr->first, itr->second));
										}
									}
								}
							}
						}

						vector<beacon> tempTen;
						int whichIndex;
						for (map<int, vector<beacon>>::iterator ita = endPoint.begin(); ita != endPoint.end(); ita++) {
							for (int i = 10; i > 0; i--) {
								whichIndex = ita->second.size() - i - 1;
								tempTen.push_back(ita->second[whichIndex]);
							}
							endTen.insert(pair<int, vector<beacon>>(ita->first, tempTen));
							tempTen.clear();
						}

						for (map<int, vector<beacon>>::iterator itb = startPoint.begin(); itb != startPoint.end(); itb++) {
							measurement.push_back(itb->second.front());
						}

						for (map<int, vector<beacon>>::iterator itc = endTen.begin(); itc != endTen.end(); itc++) {
							for (int i = 0; i < measurement.size(); i++) {
								tempTable = kalman.kalmanPrediction(itc->second, measurement[i]);  //create probability table for each pair of end/start trace segments
								probab.insert(probab.end(), tempTable.begin(), tempTable.end()); //add new iteration result to the probability table
							}
						}


						//proceed probability table

						//delete probabilities that are extremely low to enhance efficiency
						int mm = 0;
						int mmsize = probab.size();
						for (mm = 0; mm < mmsize; mm++)
						{
							if (probab[mm].probability < 1e-100) //filter out the segment pair which probability too low (here < 1e-100)
							{
								vector <proTable> ::iterator it = probab.begin() + mm;
								probab.erase(it);
								mmsize = probab.size();
								--mm;
							}
						}


						int nn = 0;
						int maxP = 0;
						while (probab.size() > 0) {
							double tempro = probab.front().probability;
							for (nn = 0; nn < probab.size(); nn++) {
								if (probab[nn].probability > tempro) { //the highest probability
									tempro = probab[nn].probability;
									maxP = nn; //find index which has the highest probability
								}
							}
							int lastPs = probab[maxP].lastPseu;
							int nextPs = probab[maxP].nextPseu;
							//merge a new trace
							vector<beacon> merge;
							map<int, vector<beacon>>::iterator key1 = multihyposis.find(probab[maxP].lastPseu);
							if (key1 != multihyposis.end()) {
								merge.insert(merge.end(), key1->second.begin(), key1->second.end());
								multihyposis.erase(key1);
							}
							else break;
							map<int, vector<beacon>>::iterator key2 = multihyposis.find(probab[maxP].nextPseu);
							if (key2 != multihyposis.end()) {
								for (int i = 0; i < key2->second.size(); i++) {
									key2->second[i].pseudonym = probab[maxP].lastPseu;
								}
								merge.insert(merge.end(), key2->second.begin(), key2->second.end());
								multihyposis.erase(key2);
							}
							else break;
							multihyposis.insert(pair<int, vector<beacon>>(probab[maxP].lastPseu, merge));
							vector <proTable> ::iterator ite = probab.begin() + maxP;
							probab.erase(ite);

							vector <proTable>::iterator itf;
							for (itf = probab.begin(); itf != probab.end(); ) {
								if ((itf->lastPseu == lastPs) || (itf->nextPseu == nextPs)) {
									probab.erase(itf);
									itf = probab.begin();
								}
								else itf++;
							}

							maxP = 0;
						}

						endPoint.clear();
						startPoint.clear();
						probab.clear();
						endTen.clear();
						measurement.clear();
						tempTable.clear();

					}

					//start to calculate success rate
					int successTrace = 0;
					map<int, vector<RSUMsg>>::iterator iter;
					for (iter = carTrace.begin(); iter != carTrace.end(); iter++) {
						int pse = iter->second[0].pseudonym;
						map<int, vector<beacon>>::iterator it = multihyposis.find(pse);
						if (it != multihyposis.end()) {
							vector<RSUMsg> comp1 = iter->second;
							vector<beacon> comp2 = it->second;
							int check = 0;
							for (int i = 0; i < comp1.size(); i++) {
								for (int j = 0; j < comp2.size(); j++) {
									if (comp1[i].timestamp == comp2[j].timestamp) {
										check++;
									}
								}
							}
							if (check == iter->second.size()) successTrace++; //total number of successful trace
						}
					}

					//a map containing mix zone indexs, enter times and leave times of cars' whole journeys
					map<int, vector<pairTime>> mixZoneTimes;
					map<int, vector<RSUMsg>>::iterator itw = carTrace.begin();
					while (itw != carTrace.end()) {
						vector<pairTime> tempMZTimes;
						for (int i = 1; i < itw->second.size(); i++) {
							if ((itw->second[i].timestamp - itw->second[i - 1].timestamp) > 0.5) {
								pairTime pte;
								for (int j = 0; j < mp.size(); j++) {
									if (itw->second[i].currX == mp[j].cooX && itw->second[i].currY == mp[j].cooY) {
										pte.mixZoneIndex = mp[j].seq;
									}
								}
								pte.enterTime = itw->second[i - 1].timestamp;
								pte.leaveTime = itw->second[i].timestamp;
								tempMZTimes.push_back(pte);
							}
						}
						mixZoneTimes.insert(pair<int, vector<pairTime>>(itw->first, tempMZTimes));
						itw++;
					}

					//how many cars does a car encounter in a mix-zone in average
					vector<double> avgTotCarEncounter;
					for (map<int, vector<pairTime>>::iterator it = mixZoneTimes.begin(); it != mixZoneTimes.end(); it++) {
						vector<double> avgCarEncounter;
						for (int i = 0; i < it->second.size(); i++) {
							int targetZone = it->second[i].mixZoneIndex;
							int eTime = it->second[i].enterTime;
							int lTime = it->second[i].leaveTime;
							int carCount = 0;
							for (map<int, vector<pairTime>>::iterator ita = mixZoneTimes.begin(); ita != mixZoneTimes.end(); ita++) {
								if (it->first != ita->first) {
									for (int j = 0; j < ita->second.size(); j++) {
										if ((ita->second[j].mixZoneIndex == targetZone) && (eTime <= ita->second[j].leaveTime) && (ita->second[j].enterTime <= lTime)) {
											carCount++;
										}
									}
								}
							}
							avgCarEncounter.push_back(carCount);
						}
						double avgCar = accumulate(avgCarEncounter.begin(), avgCarEncounter.end(), 0.00) / avgCarEncounter.size();
						avgTotCarEncounter.push_back(avgCar);
					}

					double avgTotCar = accumulate(avgTotCarEncounter.begin(), avgTotCarEncounter.end(), 0.000) / avgTotCarEncounter.size(); //in average, how many cars are encountered in each junction for a car


					vector<int> avgTotJunEncounter;
					double avgTotJun = 0;

					for (map<int, vector<pairTime>>::iterator it = mixZoneTimes.begin(); it != mixZoneTimes.end(); it++) {
						int junEncounter = 0;
						for (int i = 0; i < it->second.size(); i++) {
							int targetZone = it->second[i].mixZoneIndex;
							int eTime = it->second[i].enterTime;
							int lTime = it->second[i].leaveTime;
							for (map<int, vector<pairTime>>::iterator ita = mixZoneTimes.begin(); ita != mixZoneTimes.end(); ita++) {
								bool isEncountered = false;
								if (it->first != ita->first) {
									for (int j = 0; j < ita->second.size(); j++) {
										if ((ita->second[j].mixZoneIndex == targetZone) && (eTime <= ita->second[j].leaveTime) && (ita->second[j].enterTime <= lTime)) {
											junEncounter++;
											isEncountered = true;
											break;
										}
									}
								}
								if (isEncountered) break;
							}
						}

						avgTotJunEncounter.push_back(junEncounter);
					}
					avgTotJun = accumulate(avgTotJunEncounter.begin(), avgTotJunEncounter.end(), 0.000) / avgTotJunEncounter.size();  //in average, how many junctions a car encoutered other cars during its journey




					//check beacon pool
					out << " " << endl;
					out << "BEACONS:" << endl;
					for (int i = 0; i < beaconPool.size(); i++) {
						out << beaconPool[i].timestamp << "\t" << beaconPool[i].pseudonym << "\t" << beaconPool[i].currX << "\t" << beaconPool[i].currY << "\t" << beaconPool[i].speedX << "\t" << beaconPool[i].speedY << endl;
					}

					out << " " << endl;
					out << "RSUMessages:" << endl;
					//check RSU pool
					for (int i = 0; i < RSUPool.size(); i++) {
						out << RSUPool[i].timestamp << "\t" << RSUPool[i].carIndex << "\t" << RSUPool[i].pseudonym << "\t" << RSUPool[i].currX << "\t" << RSUPool[i].currY << "\t" << RSUPool[i].speedX << "\t" << RSUPool[i].speedY << endl;
					}

					out << " " << endl;
					out << "MultiHyposis:" << endl;
					//check MultiHyposis
					for (map<int, vector<beacon>>::iterator ita = multihyposis.begin(); ita != multihyposis.end(); ita++) {
						for (int i = 0; i < ita->second.size(); i++)
							out << ita->second[i].timestamp << " " << ita->second[i].pseudonym << endl;
					}



					//print results
					out << " " << endl;
					out << "average car encouter is " << avgTotCar << endl;
					out << " " << endl;
					out << "average junction encouter is " << avgTotJun << endl;
					out << " " << endl;
					out << "success rate of tracking is " << 100 * successTrace / cr.size() << "%" << endl;

					
					cout << "average car encouter is " << avgTotCar << endl;
					cout << "average junction encouter is " << avgTotJun << endl;
					cout << "success rate of tracking is " << 100 * successTrace / cr.size() << "%" << endl;
					

					//fout << avgTotCar << "\t" << avgTotJun << "\t" << 100 * successTrace / cr.size() << "\t" << fakeBeaconNum << "\t" << fakeRouteNum << endl;
					fout << avgTotCar << "\t" << avgTotJun << "\t" << 100 * successTrace / cr.size() << endl;
					
					//close file
					out.close();
					cr.clear();
					beaconPool.clear();   //store all beacons (only contains car pseudonym)
					RSUPool.clear();
	return 0;
}
*/