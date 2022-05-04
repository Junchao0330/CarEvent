#include "event.h"
#include <fstream>
#include <iostream>


using namespace std;
MapData md;
CarData cd;
EventList eList;
KalmanFilter kalman;


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
		for (int j = 0; j < sq.size(); j++) {
			if (tX == sq[j].cooX && tY == sq[j].cooY) {
				sq[j].isOcpd = true;
			}
		}
		Event evt(cc[i].index, cc[i].showupTime, tX, tY, cc[i].maxSpeed, carArrive);
		events[count] = evt;
		count++;
		//return cc;
	}
	
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
				if (sq[i].roadNum != sq[i + 1].roadNum) {  //and it is the last step of the road
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
					if (sq[i + 3].roadType == 2) {
						cc[tempid].currentSpeed = 0.6* cc[tempid].currentSpeed;
					}
					else if (sq[i + 2].roadType == 2) {
						cc[tempid].currentSpeed = 0.6 * cc[tempid].currentSpeed;
					}
					else if (sq[i + 1].roadType == 2) {
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
					tryTime = e.time + 0.5;  //try to move 0.5s later
					e.speed = 1;
					Event delayE(e.carIndex, tryTime, e.preX, e.preY, e.coorX, e.coorY, e.mspeed, e.speed, carPlan);
					events[count] = delayE;
					count++;
				}
			}
		  
	}
}



Event EventList::deleteEvent()  //The event with minimum time given top priority
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

beacon EventList::createBeacon(double time, int pseudo, int cx, int cy, double vx, double vy) //create beacon
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

RSUMsg EventList::createRSUMsg(double time, int carID, int pseudo, int cx, int cy, double vx, double vy)  //create RSU message
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

bool EventList::enterMixZone(int cx, int cy, vector<roadSq> sq)
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

int main() //main function
{
	mp = md.ReadMapData("C:/Users/wangj/source/repos/ScottsvilleMap.txt");  //load map info
	int junNum = md.getJunctionNumber(mp);  //number of junctions
	cr = cd.ReadCarData("C:/Users/wangj/source/repos/car.txt");  //load car info

	eList.addArrivalEvent(cr, mp);    //add car arrive events



	fstream out;  //start log file
	out.open("C:/Users/wangj/source/repos/output.txt");

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
			cr[tempi].currentX = tempE.coorX;
			cr[tempi].currentY = tempE.coorY;
			out << "t = " << currentTime << ", car " << carID << " arrives at " << tempE.coorX << "," << tempE.coorY << endl;
			beacon Beacon = eList.createBeacon(tempE.getTime(), cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, 0, 0); //dedicate beacon
			beaconPool.push_back(Beacon); //start to "broadcast" beacon
			RSUMsg Beacon_Id = eList.createRSUMsg(tempE.getTime(), cr[tempi].index, cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, 0, 0); //dedicate RSU message
			RSUPool.push_back(Beacon_Id); //start to log RSU message
			eList.carPlanEvent(tempE, cr, mp, currentTime);
		}
		else if (tempE.getEType() == carLeave) {    //Process carLeave event
			cr[tempi].currentX = tempE.coorX;
			cr[tempi].currentY = tempE.coorY;
			for (int i = 0; i < mp.size(); i++) {
				if (tempE.coorX == mp[i].cooX && tempE.coorY == mp[i].cooY) {
					mp[i].isOcpd = false;  //release the last stepstone the car occupied before discard
				}
			}
			out << "t = " << tempE.leaveTime << ", car " << carID << " leaves at " << tempE.coorX << "," << tempE.coorY << endl;
		}
		else if (tempE.getEType() == carPlan) {     //Process carMove event
			eList.carMoveEvent(tempE, cr, mp, currentTime);
		}
		else if (tempE.getEType() == carMove) {     //Process carMove event

			//if enters a mix-zone, car updates its pseudonym
			if (eList.enterMixZone(tempE.coorX, tempE.coorY, mp)) {
				cr[tempi].pseudonym += 1;
			}

			cr[tempi].currentX = tempE.coorX;
			cr[tempi].currentY = tempE.coorY;
			for (int i = 0; i < mp.size(); i++) {
				if (tempE.coorX == mp[i].cooX && tempE.coorY == mp[i].cooY) {  //set arrived setpstone as occupied and release previous one
					mp[i].isOcpd = true;
				}
				if (tempE.preX == mp[i].cooX && tempE.preY == mp[i].cooY) {  //set arrived setpstone as occupied and release previous one
					mp[i].isOcpd = false;
				}
			}
			eList.carPlanEvent(tempE, cr, mp, tempE.time + 0.0001); //proceed new plan after the movement
			out << "t = " << tempE.getTime() << ", car " << carID << " goes to " << tempE.coorX << "," << tempE.coorY << endl;

			double realSpeed = sqrt(pow((tempE.coorY - tempE.preY), 2) + pow((tempE.coorX - tempE.preX), 2)) / (currentTime - cr[tempi].arriveTime);
			beacon Beacon_Psu = eList.createBeacon(tempE.getTime(), cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, cos(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed, sin(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed); //dedicate beacon (only contains pseudonym)
			RSUMsg Beacon_Id = eList.createRSUMsg(tempE.getTime(), cr[tempi].index, cr[tempi].pseudonym, cr[tempi].currentX, cr[tempi].currentY, cos(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed, sin(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed); //dedicate RSU message (contains car ID and pseudonym)
			beaconPool.push_back(Beacon_Psu); //start to "broadcast" beacon
			RSUPool.push_back(Beacon_Id); //log RSU message
			cr[tempi].arriveTime = currentTime;

		}
	}
	out << "simulation ends" << endl;    //traffic simulation ends

	//start tracking algorithm using collected beacons

	map<int,vector<beacon>> multihyposis;  //key = pseudonym. value = vector of beacons
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
		cout <<"No" << junSeq << ": " << endl;
		for (map<int, vector<beacon>>::iterator it = multihyposis.begin(); it != multihyposis.end(); it++) {
			for (int p = 0; p < mp.size(); p++) {
				if (it->second.back().currX == mp[p].cooX && it->second.back().currY == mp[p].cooY) {
					if (mp[p].seq == junSeq) {
						endPoint.insert(pair<int, vector<beacon>>(it->first,it->second));  
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
		for (int i = 0; i < probab.size(); i++) {
			cout << probab[i].lastPseu << " " << probab[i].nextPseu << " "<<probab[i].probability << endl;
		}
		//proceed probability table

		//delete probabilities that are extremely low to enhance efficiency
		int mm = 0;
		int mmsize = probab.size();
		for (mm = 0; mm < mmsize; mm++)
		{
			if (probab[mm].probability < exp(-100)) //filter out the segment pair which probability too low (here < exp(-100))
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
			//cout << check << endl;
			if (check == iter->second.size()) successTrace++; //total number of successful trace
		}
	}


	//check beacon pool
	out <<" " << endl;
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
		for (int i = 0;i< ita->second.size();i++)
		out << ita->second[i].timestamp<<" " << ita->second[i].pseudonym << endl;
	}
		
	

	//print success rate
	out << " " << endl;
	out << "success rate of tracking is " << 100 * successTrace / cr.size() << "%" << endl;
	cout << 100 * successTrace / cr.size() << "%" << endl;

	//close file
	out.close();
	return 0;
}
