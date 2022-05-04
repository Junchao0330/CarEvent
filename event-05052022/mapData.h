#ifndef MAPDATA_H
#define MAPDATA_H
#include <vector>
#include <string>
#include <fstream>
#include <malloc.h>
#include <vector>

using namespace std;

struct roadSq 
{
	int roadNum = 0; //road number
	int cooX, cooY;  //road square x/y coordinate
	int seq = 0;  //junction sequence
	int roadType;  //road square type: Edge=1, Junction entrance=2, Junction exit = 3, Normal=0
	bool isOcpd = false;  //occupied by car or not
};


class MapData {
public:
	static vector<roadSq> ReadMapData(const char* fileName);    //read map data
	int getJunctionNumber(vector<roadSq> sq); //get junction number
};
#endif