#include "carData.h"
#include <iostream>
using namespace std;

vector<car> CarData::ReadCarData(string fileName)
{
	fstream inf;
	inf.open(fileName);
	if (!inf)
	{
		throw("can not open the file.");
	}

	car c;
	vector<car> carinfo;
	string line;

	while (getline(inf, line))
	{
		stringstream ss(line);
		string tmp;
		int i = 0;
		int index = 0, stime = 0, speed = 0;  //car index, arrive time, speed, route
		string route;    //car route

		while (getline(ss, tmp, ' ')) {
			if (i == 0) index = stoi(tmp);      //array starts from 0, car index starts from 1
			else if (i == 1) { stime = stof(tmp); }
			else if (i == 2) { speed = stof(tmp); }
			else { route = route + tmp + " "; }  //the rest of the line is route info
			i++;
		}
		c.index = index;
		c.showupTime = stime;
		c.maxSpeed = speed;
		c.route = route;
		c.pseudonym = index << 10; //ensure the reepeated pseudonym won't occur until proceeding 2^10=1024 times, and the max vehicle capacity is 1024.
		carinfo.push_back(c);
	}
	inf.close();
	return carinfo;
}


int CarData::getCurrentRoad(string s) {
	string road;
	int r;
	int i = 0;
	while (i < s.length()) {
		i++;
		if ((s[i] == ' ')) {  //not the last road
			road = s.substr(0, i);
			break;
		}
		else {  //last road
			road = s;
		}
	}
	r = stoi(road);
	return r;
}


string CarData::restRoad(string s) {
	int i = 0;
	string t;
	while (i < s.length()) {
		i++;
		if ((s[i] == ' ')) {  //not the last road
			t = s.erase(0, i + 1);
			break;
		}
		else {  //last road
			t = "";
		}
	}
	return t;
}