#include "attacker.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <sqlite3.h>
#include <fstream>
#include <streambuf> 


int main()
{
	//new outstream
	ofstream oFile;
	double ans[2][5];  //first column = fake beacon, second column = mix-zone
	for (int j = 0; j < 2; j++) {
		for (int i = 0; i < 5; i++) {
			ans[j][i] = 0;
		}
	}

	
	fakeBeaconAllowed = true;  //fake beacon on
	int arrival = initialArrival;
	int round = 0;

	/*
	while (arrival <= 5 * initialArrival) {
		vector<int> tmpAns;
		for (int ini = initialTrial; ini <= 5 * initialTrial; ini++) {
			int successTrace = 0;
			sqlite3* db;
			sqlite3_stmt* stmt;

			// connect to database
			string s = "C:/Users/wangj/source/pythonProject/ar_" + to_string(arrival) + "_test_" + to_string(ini) + ".db";
			int rc = sqlite3_open(s.c_str(), &db);
			if (rc != SQLITE_OK) {
				std::cerr << "Failed to open database: " << sqlite3_errmsg(db) << std::endl;
				return rc;
			}

			// select from IntTable
			const char* sql_int = "SELECT * FROM IntTable;";
			rc = sqlite3_prepare_v2(db, sql_int, -1, &stmt, NULL);
			if (rc != SQLITE_OK) {
				std::cerr << "Failed to prepare SQL statement: " << sqlite3_errmsg(db) << std::endl;
				return rc;
			}


			// get result for int table
			while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
				beacon bea;
				bea.pseudonym = sqlite3_column_int(stmt, 0); //distingiush from real cars
				bea.currX = sqlite3_column_double(stmt, 1);
				bea.currY = sqlite3_column_double(stmt, 2);
				bea.speedX = sqlite3_column_double(stmt, 3);
				bea.speedY = sqlite3_column_double(stmt, 4);
				bea.timestamp = sqlite3_column_double(stmt, 5);
				intPool.push_back(bea);
			}

			// throw errors
			if (rc != SQLITE_DONE) {
				std::cerr << "Failed to fetch data: " << sqlite3_errmsg(db) << std::endl;
			}

			// select from rnnTable
			const char* sql_rnn = "SELECT * FROM RnnTable;";
			rc = sqlite3_prepare_v2(db, sql_rnn, -1, &stmt, NULL);
			if (rc != SQLITE_OK) {
				std::cerr << "Failed to prepare SQL statement: " << sqlite3_errmsg(db) << std::endl;
				return rc;
			}


			// get result for rnn table
			while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
				beacon bea;
				bea.pseudonym = sqlite3_column_int(stmt, 0) + 1000;
				bea.currX = sqlite3_column_double(stmt, 1);
				bea.currY = sqlite3_column_double(stmt, 2);
				bea.speedX = sqlite3_column_double(stmt, 3);
				bea.speedY = sqlite3_column_double(stmt, 4);
				bea.timestamp = sqlite3_column_double(stmt, 5);
				rnnPool.push_back(bea);
			}

			// throw errors
			if (rc != SQLITE_DONE) {
				std::cerr << "Failed to fetch data: " << sqlite3_errmsg(db) << std::endl;
			}

			// select from preTable
			const char* sql_pre = "SELECT * FROM PreTable;";
			rc = sqlite3_prepare_v2(db, sql_pre, -1, &stmt, NULL);
			if (rc != SQLITE_OK) {
				std::cerr << "Failed to prepare SQL statement: " << sqlite3_errmsg(db) << std::endl;
				return rc;
			}


			// get result for pre table
			while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
				beacon bea;
				bea.pseudonym = sqlite3_column_int(stmt, 0);
				bea.currX = sqlite3_column_double(stmt, 1);
				bea.currY = sqlite3_column_double(stmt, 2);
				bea.speedX = sqlite3_column_double(stmt, 3);
				bea.speedY = sqlite3_column_double(stmt, 4);
				bea.timestamp = sqlite3_column_double(stmt, 5);
				prePool.push_back(bea);
			}

			// throw errors
			if (rc != SQLITE_DONE) {
				std::cerr << "Failed to fetch data: " << sqlite3_errmsg(db) << std::endl;
			}
			// release connection
			sqlite3_finalize(stmt);
			sqlite3_close(db);


			map<int, vector<beacon>> endPoint;  //key = pseudonym. value = vector of beacons, trace segments of the same junction to drive in;
			for (int i = 0; i < prePool.size(); i++) {
				endPoint[prePool[i].pseudonym].push_back(prePool[i]); //beacons with same pseudonym go to same vector
			}

			map<int, vector<beacon>> startPoint; //trace segments of the same junction to drive out;
			for (int i = 0; i < intPool.size(); i++) {
				startPoint[intPool[i].pseudonym].push_back(intPool[i]); //beacons with same pseudonym go to same vector
			}
			if (fakeBeaconAllowed) {
				for (int i = 0; i < rnnPool.size(); i++) {
					startPoint[rnnPool[i].pseudonym].push_back(rnnPool[i]); //rnn result included
				}
			}

			map<int, vector<beacon>> endTen; //endPoint trace segments with last 10 beacons;
			vector<beacon> measurement; //first beacon in startPoint trace segments;
			vector<proTable> probab;  //probability table
			vector<proTable> tempTable;

			//start to traversal all 5 junctions
			int carNum = prePool.size() / 10;  //number of cars
			int minIndex = 0;
			int maxIndex = carNum / 5;
			int junc = 1;
			while (junc <= 5) {
				for (map<int, vector<beacon>>::iterator it = endPoint.begin(); it != endPoint.end(); it++) {
					if (it->first > minIndex && it->first <= maxIndex) {
						endTen.insert(pair<int, vector<beacon>>(it->first, it->second));  //all inbound cars for this junction
					}
				}
				for (map<int, vector<beacon>>::iterator it1 = startPoint.begin(); it1 != startPoint.end(); it1++) {
					if ((it1->first > minIndex && it1->first <= maxIndex) || (it1->first > minIndex + 1000 && it1->first <= maxIndex + 1000)) {
						measurement.push_back(it1->second.back());  //all outbound cars for this junction

					}
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
					if (lastPs == nextPs) successTrace++; //successful attack


					vector <proTable> ::iterator ite = probab.begin() + maxP;
					probab.erase(ite); //remove this hypothesis pair

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

				probab.clear();
				endTen.clear();
				measurement.clear();
				prePool.clear();
				intPool.clear();
				rnnPool.clear();
				minIndex += carNum / 5;
				maxIndex += carNum / 5;
				junc += 1;
			}
			tmpAns.push_back(100 * successTrace / carNum);
			cout << 100 * successTrace / carNum << endl;
		}
		double sum = 0;
		for (int i = 0; i < tmpAns.size(); i++) {
			sum += tmpAns[i];
		}
		ans[0][round] = sum / 5;
		arrival += initialArrival;
		round++;
	}
	*/

	fakeBeaconAllowed = false;  //fake beacon off
	arrival = initialArrival;
	round = 0;
	while (arrival <= 5 * initialArrival) {
		vector<int> tmpAns;
		for (int ini = initialTrial; ini <= 5 * initialTrial; ini++) {
			int successTrace = 0;
			sqlite3* db;
			sqlite3_stmt* stmt;

			// connect to database
			string s = "C:/Users/wangj/source/pythonProject/ar_" + to_string(arrival) + "_test_" + to_string(ini) + ".db";
			int rc = sqlite3_open(s.c_str(), &db);
			if (rc != SQLITE_OK) {
				std::cerr << "Failed to open database: " << sqlite3_errmsg(db) << std::endl;
				return rc;
			}

			// select from IntTable
			const char* sql_int = "SELECT * FROM IntTable;";
			rc = sqlite3_prepare_v2(db, sql_int, -1, &stmt, NULL);
			if (rc != SQLITE_OK) {
				std::cerr << "Failed to prepare SQL statement: " << sqlite3_errmsg(db) << std::endl;
				return rc;
			}


			// get result for int table
			while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
				beacon bea;
				bea.pseudonym = sqlite3_column_int(stmt, 0); //distingiush from real cars
				bea.currX = sqlite3_column_double(stmt, 1);
				bea.currY = sqlite3_column_double(stmt, 2);
				bea.speedX = sqlite3_column_double(stmt, 3);
				bea.speedY = sqlite3_column_double(stmt, 4);
				bea.timestamp = sqlite3_column_double(stmt, 5);
				intPool.push_back(bea);
			}

			// throw errors
			if (rc != SQLITE_DONE) {
				std::cerr << "Failed to fetch data: " << sqlite3_errmsg(db) << std::endl;
			}

			// select from rnnTable
			const char* sql_rnn = "SELECT * FROM RnnTable;";
			rc = sqlite3_prepare_v2(db, sql_rnn, -1, &stmt, NULL);
			if (rc != SQLITE_OK) {
				std::cerr << "Failed to prepare SQL statement: " << sqlite3_errmsg(db) << std::endl;
				return rc;
			}


			// get result for rnn table
			while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
				beacon bea;
				bea.pseudonym = sqlite3_column_int(stmt, 0) + 1000;
				bea.currX = sqlite3_column_double(stmt, 1);
				bea.currY = sqlite3_column_double(stmt, 2);
				bea.speedX = sqlite3_column_double(stmt, 3);
				bea.speedY = sqlite3_column_double(stmt, 4);
				bea.timestamp = sqlite3_column_double(stmt, 5);
				rnnPool.push_back(bea);
			}

			// throw errors
			if (rc != SQLITE_DONE) {
				std::cerr << "Failed to fetch data: " << sqlite3_errmsg(db) << std::endl;
			}

			// select from preTable
			const char* sql_pre = "SELECT * FROM PreTable;";
			rc = sqlite3_prepare_v2(db, sql_pre, -1, &stmt, NULL);
			if (rc != SQLITE_OK) {
				std::cerr << "Failed to prepare SQL statement: " << sqlite3_errmsg(db) << std::endl;
				return rc;
			}


			// get result for pre table
			while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
				beacon bea;
				bea.pseudonym = sqlite3_column_int(stmt, 0);
				bea.currX = sqlite3_column_double(stmt, 1);
				bea.currY = sqlite3_column_double(stmt, 2);
				bea.speedX = sqlite3_column_double(stmt, 3);
				bea.speedY = sqlite3_column_double(stmt, 4);
				bea.timestamp = sqlite3_column_double(stmt, 5);
				prePool.push_back(bea);
			}

			// throw errors
			if (rc != SQLITE_DONE) {
				std::cerr << "Failed to fetch data: " << sqlite3_errmsg(db) << std::endl;
			}
			// release connection
			sqlite3_finalize(stmt);
			sqlite3_close(db);


			map<int, vector<beacon>> endPoint;  //key = pseudonym. value = vector of beacons, trace segments of the same junction to drive in;
			for (int i = 0; i < prePool.size(); i++) {
				endPoint[prePool[i].pseudonym].push_back(prePool[i]); //beacons with same pseudonym go to same vector
			}

			/*
			//this code is for generating one fake beacon without the RNN
			//*******************************************************************************************************************************************
			vector<beacon> fakePoint;
			for (map<int, vector<beacon>>::iterator it = endPoint.begin(); it != endPoint.end(); it++) {
				beacon tmpb = it->second.back();
				double v = sqrt(pow(tmpb.speedX, 2) + pow(tmpb.speedY, 2));
				random_device rd;   // 用于获得真正的随机数种子
				mt19937 gen(rd());  // 随机数生成器
				uniform_real_distribution<double> dis(0.95, 1.05);  // 均匀分布的随机数生成器
				double random_v = v * dis(gen);
				int id = tmpb.pseudonym;
				double outx;
				double outy;
				if (id <= 50) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 4);
					outx = junc1[random][0];
					outy = junc1[random][1];
				}
				else if (id <= 100 && id > 50) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 3);
					outx = junc2[random][0];
					outy = junc2[random][1];
				}
				else if (id <= 150 && id > 100) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 3);
					outx = junc3[random][0];
					outy = junc3[random][1];
				}
				else if (id <= 200 && id > 150) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 4);
					outx = junc4[random][0];
					outy = junc4[random][1];
				}
				else {// 200 < ID <= 250
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 5);
					outx = junc5[random][0];
					outy = junc5[random][1];
				}
				beacon fake;
				fake.currX = outx;
				fake.currY = outy;
				fake.pseudonym = tmpb.pseudonym + 10000; //distinguish with current pseudonyms
				fake.speedX = v * outx / 10;
				fake.speedY = v * outy / 10;
				fake.timestamp = tmpb.timestamp + 20 / v;
				fakePoint.push_back(fake);
			}

			//*******************************************************************************************************************************************
			*/

						//this code is for generating two fake beacons without the RNN
			//*******************************************************************************************************************************************
			vector<beacon> fakePoint;
			for (map<int, vector<beacon>>::iterator it = endPoint.begin(); it != endPoint.end(); it++) {
				beacon tmpb = it->second.back();
				double v = sqrt(pow(tmpb.speedX, 2) + pow(tmpb.speedY, 2));
				random_device rd;   // 用于获得真正的随机数种子
				mt19937 gen1(rd());  // 随机数生成器
				uniform_real_distribution<double> dis(0.95, 1.05);  // 均匀分布的随机数生成器
				double random_v1 = v * dis(gen1);
				random_device rdd;   // 用于获得真正的随机数种子
				mt19937 gen2(rdd());  // 随机数生成器
				double random_v2 = v * dis(gen2);
				int id = tmpb.pseudonym;
				double outx1;
				double outy1;
				double outx2;
				double outy2;
				if (id <= 50) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 4);
					outx1 = junc1[random][0];
					outy1 = junc1[random][1];
				}
				else if (id <= 100 && id > 50) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 3);
					outx1 = junc2[random][0];
					outy1 = junc2[random][1];
				}
				else if (id <= 150 && id > 100) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 3);
					outx1 = junc3[random][0];
					outy1 = junc3[random][1];
				}
				else if (id <= 200 && id > 150) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 4);
					outx1 = junc4[random][0];
					outy1 = junc4[random][1];
				}
				else {// 200 < ID <= 250
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 5);
					outx1 = junc5[random][0];
					outy1 = junc5[random][1];
				}
				if (id <= 50) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 4);
					outx2 = junc1[random][0];
					outy2 = junc1[random][1];
				}
				else if (id <= 100 && id > 50) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 3);
					outx2 = junc2[random][0];
					outy2 = junc2[random][1];
				}
				else if (id <= 150 && id > 100) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 3);
					outx2 = junc3[random][0];
					outy2 = junc3[random][1];
				}
				else if (id <= 200 && id > 150) {
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 4);
					outx2 = junc4[random][0];
					outy2 = junc4[random][1];
				}
				else {// 200 < ID <= 250
					srand((unsigned)time(NULL));
					// Get a random number
					int random = (rand() % 5);
					outx2 = junc5[random][0];
					outy2 = junc5[random][1];
				}
				beacon fake1;
				fake1.currX = outx1;
				fake1.currY = outy1;
				fake1.pseudonym = tmpb.pseudonym + 20000; //distinguish with current pseudonyms
				fake1.speedX = v * outx1 / 10;
				fake1.speedY = v * outy1 / 10;
				fake1.timestamp = tmpb.timestamp + 20 / v;
				fakePoint.push_back(fake1);
				beacon fake2;
				fake2.currX = outx1;
				fake2.currY = outy1;
				fake2.pseudonym = tmpb.pseudonym + 30000; //distinguish with current pseudonyms
				fake2.speedX = v * outx1 / 10;
				fake2.speedY = v * outy1 / 10;
				fake2.timestamp = tmpb.timestamp + 20 / v;
				fakePoint.push_back(fake2);
			}

			//*******************************************************************************************************************************************

			map<int, vector<beacon>> startPoint; //trace segments of the same junction to drive out;
			for (int i = 0; i < intPool.size(); i++) {
				startPoint[intPool[i].pseudonym].push_back(intPool[i]); //beacons with same pseudonym go to same vector
			}
			if (fakeBeaconAllowed) {
				for (int i = 0; i < rnnPool.size(); i++) {
					startPoint[rnnPool[i].pseudonym].push_back(rnnPool[i]); //rnn result included
				}
			}

			map<int, vector<beacon>> endTen; //endPoint trace segments with last 10 beacons;
			vector<beacon> measurement = fakePoint; //first beacon in startPoint trace segments;
			vector<proTable> probab;  //probability table
			vector<proTable> tempTable;

			//start to traversal all 5 junctions
			int carNum = prePool.size() / 10;  //number of cars
			int minIndex = 0;
			int maxIndex = carNum / 5;
			int junc = 1;
			while (junc <= 5) {
				for (map<int, vector<beacon>>::iterator it = endPoint.begin(); it != endPoint.end(); it++) {
					if (it->first > minIndex && it->first <= maxIndex) {
						endTen.insert(pair<int, vector<beacon>>(it->first, it->second));  //all inbound cars for this junction
					}
				}
				for (map<int, vector<beacon>>::iterator it1 = startPoint.begin(); it1 != startPoint.end(); it1++) {
					if ((it1->first > minIndex && it1->first <= maxIndex) || (it1->first > minIndex + 1000 && it1->first <= maxIndex + 1000)) {
						measurement.push_back(it1->second.back());  //all outbound cars for this junction

					}
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
					if (lastPs == nextPs) successTrace++; //successful attack


					vector <proTable> ::iterator ite = probab.begin() + maxP;
					probab.erase(ite); //remove this hypothesis pair

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

				probab.clear();
				endTen.clear();
				measurement.clear();
				prePool.clear();
				intPool.clear();
				rnnPool.clear();
				minIndex += carNum / 5;
				maxIndex += carNum / 5;
				junc += 1;
			}
			tmpAns.push_back(100 * successTrace / carNum);
			cout << 100 * successTrace / carNum << endl;
		}
		double sum = 0;
		for (int i = 0; i < tmpAns.size(); i++) {
			sum += tmpAns[i];
		}
		ans[1][round] = sum / 5;
		arrival += initialArrival;
		round++;
	}

	//open csv file 
	oFile.open("C:/Users/wangj/source/resultsheet.csv", ios::out | ios::trunc);
	for (int i = 0; i < 5; i++) {
		oFile << ans[0][i] << "," << ans[1][i] << endl;
	}

	oFile.close();
	return 0;
}