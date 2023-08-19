#include "kalman.h"
#include <iostream>

#define PIE 3.1416

 KalmanFilter::KalmanFilter(int stateSize_ = 0, int measSize_ = 0, int uSize_ = 0) :stateSize(stateSize_), measSize(measSize_), uSize(uSize_)
    {
        if (stateSize == 0 || measSize == 0)
        {
            std::cerr << "Error, State size and measurement size must bigger than 0\n";
        }

        x.resize(stateSize);
        x.setZero();

        A.resize(stateSize, stateSize);
        A.setIdentity();

        u.resize(uSize);
        u.transpose();
        u.setZero();

        B.resize(stateSize, uSize);
        B.setZero();

        P.resize(stateSize, stateSize);
        P.setIdentity();

        H.resize(measSize, stateSize);
        H.setZero();

        z.resize(measSize);
        z.setZero();

        Q.resize(stateSize, stateSize);
        Q.setZero();

        R.resize(measSize, measSize);
        R.setZero();
    }

void KalmanFilter::init(Eigen::VectorXd & x_, Eigen::MatrixXd & P_, Eigen::MatrixXd & R_, Eigen::MatrixXd & Q_)
    {
        x = x_;
        P = P_;
        R = R_;
        Q = Q_;
    }
Eigen::VectorXd KalmanFilter::predict(Eigen::MatrixXd & A_, Eigen::MatrixXd & B_, Eigen::VectorXd & u_)
    {
        A = A_;
        B = B_;
        u = u_;
        x = A * x + B * u;
        Eigen::MatrixXd A_T = A.transpose();
        P = A * P * A_T + Q;
        return x;
    }

Eigen::VectorXd KalmanFilter::predict(Eigen::MatrixXd & A_)
    {
        A = A_;
        x = A * x;
        Eigen::MatrixXd A_T = A.transpose();
        P = A * P * A_T + Q;
        //  cout << "P-=" << P<< endl;
        return x;
    }

Eigen::MatrixXd KalmanFilter::update(Eigen::MatrixXd & H_, Eigen::VectorXd z_meas)
    {
        H = H_;
        Eigen::MatrixXd temp1, temp2, Ht;
        Ht = H.transpose();
        temp1 = H * P * Ht + R;
        temp2 = temp1.inverse();//(H*P*H'+R)^(-1)
        Eigen::MatrixXd K = P * Ht * temp2;
        z = H * x;
        x = x + K * (z_meas - z);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);
        P = (I - K * H) * P;
        return temp1;
    }

double normalDistribution(double cov) {
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine gen(seed);
    default_random_engine generator;
    normal_distribution<double> distribution(0, cov);

    double d = distribution(gen);
    return d;
}

vector<proTable> KalmanFilter::kalmanPrediction(vector<beacon> tenBeacon, beacon measure)
{
    vector<double> delT; //delta T from last state to current state
    vector<proTable> prob; //probability table that is going to return
    for (int i = 0; i < tenBeacon.size()-1; i++)
    {
        delT.push_back(tenBeacon[i+1].timestamp - tenBeacon[i].timestamp);
    }
    int stateSize = 4;
    int measSize = 2;
    int controlSize = 0;
    KalmanFilter kf(stateSize, measSize, controlSize);
    Eigen::MatrixXd H(measSize, stateSize);
    H << 1, 0, 0, 0,
        0, 1, 0, 0;
    Eigen::MatrixXd P(stateSize, stateSize);
    P.setIdentity();
    Eigen::MatrixXd R(measSize, measSize);
    R.setIdentity() * 0.04;
    Eigen::MatrixXd Q(stateSize, stateSize);
    Q.setIdentity() * 0.001;
    Eigen::VectorXd x(stateSize);
    Eigen::VectorXd u(0);
    Eigen::VectorXd z(measSize);
    z.setZero();
    Eigen::VectorXd res(stateSize);
    Eigen::MatrixXd B(measSize, measSize);

    for (int i = 0; i < 10; i++) //initiation + 9 times iteration
    {
        if (i == 0)
        {
            x << tenBeacon[0].currX, tenBeacon[0].currY, tenBeacon[0].speedX, tenBeacon[0].speedY;
            kf.init(x, P, R, Q);
        }
        else {
            Q = Q.setIdentity() * normalDistribution(0.04 * delT[i - 1]);
            R = R.setIdentity() * normalDistribution(0.04);
            Eigen::MatrixXd A(stateSize, stateSize);
            A << 1, 0, delT[i-1], 0,
                0, 1, 0, delT[i-1],
                0, 0, 1, 0,
                0, 0, 0, 1;
            res << kf.predict(A);
            z << tenBeacon[i].currX, tenBeacon[i].currY;
            B = kf.update(H, z);
        }
    }

     double delt = measure.timestamp - tenBeacon.back().timestamp;
     Eigen::MatrixXd Aa(stateSize, stateSize);
     Aa << 1, 0, delt, 0,
         0, 1, 0, delt,
         0, 0, 1, 0,
         0, 0, 0, 1;
     res << kf.predict(Aa);  //final prediction state
     Eigen::VectorXd zFinal(measSize);
     zFinal << measure.currX, measure.currY; // X,Y coordinate for the observed state
     Eigen::VectorXd zU(measSize);
     Eigen::MatrixXd zU_T;
        
     zU = zFinal - H * res;
     Eigen::MatrixXd B_i;
     B_i = B.inverse();
     zU_T = zU.transpose();
     double probab = exp(-0.5 * (zU_T * B_i * zU).value()) / (2 * PIE * sqrt(B.determinant())); //the probability of this observed state belonging to previous trace in kalman filter iteration
        
        
     proTable pTa;
     pTa.lastPseu = tenBeacon.back().pseudonym;
     pTa.nextPseu = measure.pseudonym;
     pTa.probability = probab;
     prob.push_back(pTa);
    
    return prob;
}