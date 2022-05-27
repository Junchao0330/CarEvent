#include "fakeBeacon.h"
MapData fmd;
CarData fcd;
EventList feList;
KalmanFilter fkalman;


vector<beacon> fakeBeacon::trafficSimulate(car c) {
	fmp = fmd.ReadMapData("C:/Users/wangj/source/repos/ScottsvilleMap.txt");  //load map info
	int junNum = fmd.getJunctionNumber(fmp);  //number of junctions
	fcr.push_back(c);
	feList.addArrivalEvent(fcr, fmp);    //add car arrive events

	double currentTime = 0; //simulation starts
	int tempi = -1;

	int carID;

	//start traffic simulation
	while (!feList.isEmpty())
	{
		Event tempE = feList.deleteEvent();
		currentTime = tempE.getTime();
		carID = tempE.getCarIndex();

		//the sequence number in car vector
		for (int j = 0; j < fcr.size(); j++) {
			if (fcr[j].index == tempE.carIndex) {
				tempi = j;
			}
		}

		if (tempE.getEType() == carArrive)          //Process carArrive event
		{
			fcr[tempi].currentX = tempE.coorX;
			fcr[tempi].currentY = tempE.coorY;
			fcr[tempi].beaconTime = 0.0001 * fcr[tempi].index;
			beacon Beaconf = feList.createBeacon(tempE.getTime(), fcr[tempi].pseudonym, fcr[tempi].currentX, fcr[tempi].currentY, 0, 0); //dedicate beacon
			fbeaconPool.push_back(Beaconf); //start to "broadcast" beacon
			feList.carPlanEvent(tempE, fcr, fmp, currentTime);
		}
		else if (tempE.getEType() == carLeave) {    //Process carLeave event
			fcr[tempi].currentX = tempE.coorX;
			fcr[tempi].currentY = tempE.coorY;
			for (int i = 0; i < fmp.size(); i++) {
				if (tempE.coorX == fmp[i].cooX && tempE.coorY == fmp[i].cooY) {
					fmp[i].isOcpd = false;  //release the last stepstone the car occupied before discard
				}
			}
		}
		else if (tempE.getEType() == carPlan) {     //Process carMove event
			feList.carMoveEvent(tempE, fcr, fmp, currentTime);
		}
		else if (tempE.getEType() == carMove) {     //Process carMove event
				fcr[tempi].currentX = tempE.coorX;
				fcr[tempi].currentY = tempE.coorY;
				for (int i = 0; i < fmp.size(); i++) {
					if (tempE.coorX == fmp[i].cooX && tempE.coorY == fmp[i].cooY) {  //set arrived setpstone as occupied and release previous one
						fmp[i].isOcpd = true;
					}
					if (tempE.preX == fmp[i].cooX && tempE.preY == fmp[i].cooY) {  //set arrived setpstone as occupied and release previous one
						fmp[i].isOcpd = false;
					}
				}
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
	return fbeaconPool;
}

vector<beacon> fakeBeacon::possibleMeasurement(vector<RSUMsg> msg, vector<roadSq> rsq) {
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