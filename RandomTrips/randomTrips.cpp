#include "randomTrips.h"
#include <iostream>
randomTrips trips;
vector<roadSq> mp;
vector<int> intervals;

vector<roadSq> randomTrips::ReadMapData(const char* fileName)  //read map data into a vector
{
	fstream in;
	in.open(fileName);
	if (!in)
	{
		throw("can not open the file.");
	}
	int temRoadNum;
	double tempX, tempY;
	int temSeq;
	int temRoadType; //Edge=1, Junction entrance=2, Junction exit = 3, Normal=0
	roadSq r;
	vector<roadSq> roadmap;
	//int i = 0;

	while (!in.eof())
	{
		in >> temRoadNum >> tempX >> tempY >> temRoadType >> temSeq;
		r.roadNum = temRoadNum;
		r.cooX = tempX;
		r.cooY = tempY;
		r.roadType = temRoadType;
		r.seq = temSeq;
		roadmap.push_back(r);
	}
	in.close();
	return roadmap;
}

bool isNotEdge(int r) {
	vector<roadSq> mapa = trips.ReadMapData("C:/Users/wangj/source/repos/ScottsvilleMap.txt");
	for (int i = 0; i < mapa.size(); i++) {
		if ((mapa[i].roadNum == r) && (mapa[i + 1].roadNum != r)) {
			if (mapa[i].roadType == 1) {
				return false;
			}
			else return true;
		}
	}
}

vector<int> findTenInterval(int cN, int iV) {
	vector<int> timeGap; //initial poisson distribution
	vector<int> poissonInterval; //final intervals
	const int nrolls = cN - 1; //generate (cn-1) gaps for (cn) vehicles
	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
	default_random_engine gen(seed);
	poisson_distribution<int> distribution(iV);
	for (int i = 0; i < nrolls; i++) {
		int number = distribution(gen);
		timeGap.push_back(number);
	}
	poissonInterval.push_back(1);
	for (int i = 0; i < timeGap.size(); i++) {
		int finInt = poissonInterval[i] + timeGap[i];
		poissonInterval.push_back(finInt);
	}
	return poissonInterval;
}

int main() {
	mp = trips.ReadMapData("C:/Users/wangj/source/repos/ScottsvilleMap.txt");  //read map
	
	int cn = 10;
	while (cn <= carNum) {
		int iv = interval/turn;
		while (iv <= interval) {
			int jn = 1;
			while (jn <= junNum) {
				vector<string> routePool;  //compare duplicate result
				vector<int> speedPool; //speeds to allocate

				while (speedPool.size() <= 10 * cn) {  //10 trails
					unsigned seed = chrono::system_clock::now().time_since_epoch().count();
					default_random_engine gen(seed);
					normal_distribution<double> dis(meanSpeed, 4);
					int s = round(dis(gen));
					if ((s > 0) && (s < 8)) {  //min speed 1, max speed 7
						speedPool.push_back(s);
					}
				}

				while (routePool.size() <= 10 * cn) {  //10 trails
					string route;
					unsigned seed = chrono::system_clock::now().time_since_epoch().count();
					default_random_engine gen(seed);
					uniform_real_distribution<double> dis(1, 62);  //road 1 to road 62 included
					int p = round(dis(gen));
					if (isNotEdge(p)) {
						stack<int> path;
						path.push(p);
						int num = 0;
						route = to_string(p);
						while (num < jn) {
							int junc;
							for (int i = 0; i < mp.size(); i++) {
								if ((mp[i].roadNum == p) && (mp[i].roadType == 2)) {
									junc = mp[i].seq;
								}
							}
							vector<int> exits;
							for (int j = 0; j < mp.size(); j++) {
								if ((mp[j].seq == junc) && (mp[j].roadType == 3)) {
									exits.push_back(mp[j].roadNum);
								}
							}
							unsigned seed2 = chrono::system_clock::now().time_since_epoch().count();
							default_random_engine gen2(seed2);
							uniform_real_distribution<double> dis2(0, exits.size() - 1);  //randomly choose an exit
							int temp = exits[round(dis2(gen2))];
							path.push(temp);
							if (isNotEdge(temp)) {
								p = temp;
								path.push(p);
								route = route + ' ' + to_string(p);
								num++;
							}
							else {
								path.pop();
								p = path.top();
							}
						}
					}
					if (route != "") {
						routePool.push_back(route);
					}
				}

				if (poissonDis) {
					int round = 0;
					while (round < 10) {  //10 trials
						vector<int> tmpInt = findTenInterval(cn, iv);
						intervals.insert(intervals.end(), tmpInt.begin(), tmpInt.end());
						round++;
					}
				}
				//create 10 route file by requirements
				for (int i = 0; i < 10; i++) {
					ofstream out;
					string name = "";
					name = to_string(i);
					if (poissonDis) {
						out.open("C:/Users/wangj/source/repos/carFile_poisson/" + to_string(cn) + "V" + to_string(iv) + "T/" + to_string(jn) + "junc/route_" + name + ".txt", ios::out);
						for (int j = 0; j < cn; j++) {
							out << j + i * cn + 1 << " " << intervals[j + i * cn] << " " << speedPool[j + i * cn] << " " << routePool[j + i * cn] << endl;
						}
					}
					else {
						out.open("C:/Users/wangj/source/repos/carFileInterval/" + to_string(cn) + "V" + to_string(iv) + "T/" + to_string(jn) + "junc/route_" + name + ".txt", ios::out);
						for (int j = 0; j < cn; j++) {
							out << j + i * cn + 1 << " " << j * iv + 1 << " " << speedPool[j + i * cn] << " " << routePool[j + i * cn] << endl;
						}
					}
					out.close();
				}
				routePool.clear();
				speedPool.clear();
				intervals.clear();
				jn++;
			}
			iv += interval / turn;
		}
		cn += 10;
	}
	return 0;
}