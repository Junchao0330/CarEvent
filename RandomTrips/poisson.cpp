#include "poisson.h"
Random::Random(bool pseudo)
/*
Post: The values of seed, add_on, and multiplier are
	  initialized.  The seed is initialized randomly only if pseudo == false.
*/
{
	if (pseudo) seed = 1;
	else seed = time(NULL) % INT_MAX;
	multiplier = 2743;
	add_on = 5923;
}
int Random::reseed()
/*
Post: The seed is replaced by a pseudorandom successor.
*/
{
	seed = seed * multiplier + add_on;
	return seed;
}
double Random::random_real()
/*
Post: A random real number between 0 and 1 is returned.
*/
{
	double max = INT_MAX + 1.0;  //INT_MAX = (2)31 -1
	double temp = reseed();
	if (temp < 0) temp = temp + max;
	return temp / max;
}
int Random::random_integer(int low, int high)
/*
Post: A random integer between low and high is returned.
*/
{
	if (low > high) return random_integer(high, low);
	else return ((int)((high - low) * random_real())) + low;
}
int Random::poisson(double mean)
/*
Post: A random integer, reflecting a Poisson distribution
	  with parameter mean, is returned.
*/
{
	double limit = exp(-mean);
	double product = random_real();
	int count = 0;
	while (product > limit) {
		count++;
		product *= random_real();
	}
	return count;
}

void Random::randomByAvg(double avg, int num) {
	double p = 1 - 1.0 / (avg + 1);
	int t; double sum = 0, ave;
	for (int i = 0; i < num; i++) {

		t = poisson(avg);
		cout << t << " ";
		sum += t;
	}
	cout << endl;
	ave = sum / num * 1.0;
	cout << "随机整数序列的平均值为：" << ave << endl;

}

int main() {
	cout << "请输入概率均值:" << endl;
	double rand;
	cin >> rand;
	cout << "请输入随机整数的个数：" << endl;
	int num;
	cin >> num;
	//产生随机序列
	Random random; int t; double sum = 0;
	random.randomByAvg(rand, num);
	return 0;
}