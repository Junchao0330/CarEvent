#include "mapData.h"
#include <iostream>
using namespace std;

vector<roadSq> MapData::ReadMapData(const char* fileName)
{
	fstream in;
	in.open(fileName);
	if (!in)
	{
		throw("can not open the file.");
	}
	int temRoadNum;
	float tempX, tempY;
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

int MapData::getJunctionNumber(vector<roadSq> sq) {
	int i, j, count = 1;
	for (i = 1; i < sq.size(); i++) {
		for (j = 0; j < i; j++) {
			if (sq[i].seq == sq[j].seq) {
				break;
			}
		}
		if (i == j) {
			count++;
		}
	}
	return count-1; //0 does not represent a junction
}