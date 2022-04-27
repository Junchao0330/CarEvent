#ifndef KALMAN_H
#define KALMAN_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "carData.h"
#include <vector>
#include <math.h>

using namespace Eigen;
using namespace std;

struct proTable
{
    int lastPseu;
    int nextPseu;
    float probability;
};

class KalmanFilter
{
private:
    int stateSize; //state variable's dimenssion
    int measSize; //measurement variable's dimession
    int uSize; //control variables's dimenssion
    Eigen::VectorXd x;
    Eigen::VectorXd z;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd u;
    Eigen::MatrixXd P;//coveriance
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;//measurement noise covariance
    Eigen::MatrixXd Q;//process noise covariance
public:
    KalmanFilter() {};
    KalmanFilter(int stateSize_, int measSize_, int uSize_);
    ~KalmanFilter() {}
    void init(Eigen::VectorXd& x_, Eigen::MatrixXd& P_, Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_, Eigen::MatrixXd& B_, Eigen::VectorXd& u_);
    Eigen::MatrixXd update(Eigen::MatrixXd& H_, Eigen::VectorXd z_meas);
    vector<proTable> kalmanPrediction(vector<beacon> tenBeacon, vector<beacon> measure);
};

#endif
