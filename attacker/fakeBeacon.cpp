#include "fakeBeacon.h"
MapData fmd;
CarData fcd;
EventList feList;


vector<beacon> fakeBeacon::trafficSimulate(car ca) {
	fbeaconPool.clear();
	fmp = fmd.ReadMapData("C:/Users/wangj/source/repos/ScottsvilleMap.txt");  //load map info
	int junNum = fmd.getJunctionNumber(fmp);  //number of junctions
	feList.addFakeArrivalEvent(ca, fmp);    //add car arrive events
	double currentTime = 0; //simulation starts
	int tempi;
	int carID;
	fcr.push_back(ca);
	//start traffic simulation
	while (!feList.isEmpty())
	{
		Event tempE = feList.deleteEvent();
		currentTime = tempE.getTime();
		carID = tempE.getCarIndex();
		tempi = 0;
		//the sequence number in car vector
		
		if (tempE.getEType() == carArrive)          //Process carArrive event
		{
			fcr[tempi].currentX = tempE.coorX;
			fcr[tempi].currentY = tempE.coorY;
			fcr[tempi].beaconTime = 0.0001 * fcr[tempi].index;
			beacon Beaconf = feList.createBeacon(tempE.getTime(), ca.pseudonym, ca.currentX, ca.currentY, 0, 0); //dedicate beacon
			fbeaconPool.push_back(Beaconf); //start to "broadcast" beacon
			feList.carPlanEvent(tempE, fcr, fmp, currentTime);
		}
		else if (tempE.getEType() == carLeave) {    //Process carLeave event
			fcr[tempi].currentX = tempE.coorX;
			fcr[tempi].currentY = tempE.coorY;
		}
		else if (tempE.getEType() == carPlan) {     //Process carMove event
			feList.carMoveEvent(tempE, fcr, fmp, currentTime);
		}
		else if (tempE.getEType() == carMove) {     //Process carMove event
				fcr[tempi].currentX = tempE.coorX;
				fcr[tempi].currentY = tempE.coorY;
				feList.carPlanEvent(tempE, fcr, fmp, tempE.time + 0.0001); //proceed new plan after the movement

				double realSpeed = sqrt(pow((tempE.coorY - tempE.preY), 2) + pow((tempE.coorX - tempE.preX), 2)) / (currentTime - fcr[tempi].arriveTime);

				//proceed as many beacons as required for this piece of carMove
				vector<carState> states = feList.composeState(fcr[tempi].arriveTime, currentTime, fcr[tempi].beaconTime, tempE.preX, tempE.preY, tempE.coorX, tempE.coorY);
				for (int i = 0; i < states.size(); i++) {
					beacon Beacon_f = feList.createBeacon(states[i].realTime, fcr[tempi].pseudonym, states[i].realX, states[i].realY, cos(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed, sin(atan2((tempE.coorY - tempE.preY), (tempE.coorX - tempE.preX))) * realSpeed); //dedicate beacon (only contains pseudonym)
					fbeaconPool.push_back(Beacon_f); //start to "broadcast" beacon
				}
				fcr[tempi].arriveTime = currentTime;
		}
	}
	fmp.clear();
	fcr.clear();
	
	return fbeaconPool;
}

/*vector<beacon> fakeBeacon::possibleMeasurement(RSUMsg msg, vector<roadSq> rsq) {
	vector<beacon> possibleMeasure;
	double last;
	int xcor, ycor;
	double xspd, yspd;
	int lastIndex = 0;
	int junc = 0;
	for (int i = 0; i < msg.size(); i++) {
		if (msg[i].timestamp >= msg[lastIndex].timestamp) {
			lastIndex = i;
		}
	}
	xcor = msg[lastIndex].currX;
	ycor = msg[lastIndex].currY;
	xspd = msg[lastIndex].speedX;
	yspd = msg[lastIndex].speedY;
	last = msg[lastIndex].timestamp;
	for (int i = 0; i < rsq.size(); i++) {
		if (xcor == rsq[i].cooX && ycor == rsq[i].cooY) {
			junc = rsq[i].seq;
		}
	}
	vector<beacon> measurements;
	for (int i = 0; i < rsq.size(); i++) {
		if ((rsq[i].seq == junc) && (rsq[i].roadType == 3)) {
			double estTime = last + sqrt(pow((rsq[i].cooX - xcor), 2) + pow((rsq[i].cooY - ycor), 2)) / sqrt(pow(xspd, 2) + pow(yspd, 2));
			double newXspd = cos(atan2((rsq[i].cooY - ycor), (rsq[i].cooX - xcor))) * sqrt(pow(xspd, 2) + pow(yspd, 2));
			double newYspd = sin(atan2((rsq[i].cooY - ycor), (rsq[i].cooX - xcor))) * sqrt(pow(xspd, 2) + pow(yspd, 2));
			beacon tempMeasure = feList.createBeacon(estTime, msg[0].carIndex, rsq[i].cooX, rsq[i].cooY, newXspd, newYspd);
			possibleMeasure.push_back(tempMeasure);
		}
	}
	return possibleMeasure;
}
*/

car fakeBeacon::newCar(RSUMsg msg, int cx, int cy, double ti, double sp, int mspeed, vector<roadSq> rsq) {
	car fakecar;
	double last;
	int fxcor, fycor, lxcor, lycor;
	double spd;
	int lastIndex = 0, firstIndex = 0;
	int junc = 0;
	int maxSp;
	fxcor = msg.currX; //initial X coordinate
	fycor = msg.currY; //initial Ycoordinate

	lxcor = cx; //last X coordinate
	lycor = cy; //last Ycoordinate
	last = ti; //last timestamp
	spd = sp; //speed will not change within the mix-zone

	for (int i = 0; i < rsq.size(); i++) { //find which mix-zone
		if (lxcor == rsq[i].cooX && lycor == rsq[i].cooY) {
			junc = rsq[i].seq;
		}
	}
	vector<roadSq> candidate;
	for (int i = 0; i < rsq.size(); i++) { //find all possible exits
		if (3 == rsq[i].roadType && junc == rsq[i].seq) {
			candidate.push_back(rsq[i]);
		}
	}
	roadSq decision;
	double theta = -1;
	for (int i = 0; i < candidate.size(); i++) { //use Cosine similarity to choose a desination
		double tempTheta = ((candidate[i].cooX - lxcor) * (lxcor - fxcor) + (candidate[i].cooY - lycor) * (lycor - fycor)) / (sqrt(pow((candidate[i].cooX - lxcor), 2) + pow((candidate[i].cooY - lycor), 2)) * sqrt(pow((lxcor - fxcor), 2) + pow((lycor - fycor), 2)));
		if (tempTheta >= theta) {
			theta = tempTheta;
			decision = candidate[i];
		}
	}
	//compose fake car info
	fakecar.index = 1000 + rand() % 1000;
	fakecar.showupTime = 0;
	fakecar.arriveTime = sqrt(pow((decision.cooY - lycor), 2) + pow((decision.cooX - lxcor), 2)) / spd + last; //the time the car arrive at new road
	fakecar.currentSpeed = spd;
	fakecar.maxSpeed = mspeed;
	fakecar.route = to_string(decision.roadNum);
	fakecar.currentX = decision.cooX;
	fakecar.currentY = decision.cooY;
	fakecar.pseudonym = fakecar.index - 1000;
	fakecar.beaconTime = fakecar.arriveTime - floor(fakecar.arriveTime);
	return fakecar;

}

vector<car> fakeBeacon::newCarGroup(RSUMsg msg, int cx, int cy, double ti, double sp, int mspeed, vector<roadSq> rsq) {
	vector<car> fakecargroup;
	double last;
	int fxcor, fycor, lxcor, lycor;
	double spd;
	int lastIndex = 0, firstIndex = 0;
	int junc = 0;
	int maxSp;
	fxcor = msg.currX; //initial X coordinate
	fycor = msg.currY; //initial Ycoordinate

	lxcor = cx; //last X coordinate
	lycor = cy; //last Ycoordinate
	last = ti; //last timestamp
	spd = sp; //speed will not change within the mix-zone

	for (int i = 0; i < rsq.size(); i++) { //find which mix-zone
		if (lxcor == rsq[i].cooX && lycor == rsq[i].cooY) {
			junc = rsq[i].seq;
		}
	}
	vector<roadSq> candidate;
	for (int i = 0; i < rsq.size(); i++) { //find all possible exits
		if (3 == rsq[i].roadType && junc == rsq[i].seq) {
			candidate.push_back(rsq[i]);
		}
	}
	random_device rd;
	mt19937 g(rd());
	shuffle(candidate.begin(), candidate.end(), g);
	vector<roadSq> decisions;
	for (int i = 0; i < 2; i++) {
		roadSq decision = candidate[i];
		decisions.push_back(decision);
	}
	//compose fake car info
	for (int i = 0; i < 2; i++) {
		car fakecar;
		fakecar.index = 1000 + rand() % 1000;
		fakecar.showupTime = 0;
		fakecar.arriveTime = sqrt(pow((decisions[i].cooY - lycor), 2) + pow((decisions[i].cooX - lxcor), 2)) / spd + last; //the time the car arrive at new road
		if (spd > 6) {
			fakecar.currentSpeed = 6;
		}
		else {
			fakecar.currentSpeed = spd;
		}
		fakecar.maxSpeed = mspeed;
		fakecar.route = to_string(decisions[i].roadNum);
		fakecar.currentX = decisions[i].cooX;
		fakecar.currentY = decisions[i].cooY;
		fakecar.pseudonym = fakecar.index - 1000;
		fakecar.beaconTime = fakecar.arriveTime - floor(fakecar.arriveTime);
		fakecargroup.push_back(fakecar);
	}
	return fakecargroup;
}

