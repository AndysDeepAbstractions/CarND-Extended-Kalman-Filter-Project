#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; // mean object state vector
  P_ = P_in; // object covariance matrix
  F_ = F_in; // state transition matrix/function
  H_ = H_in; // measurement matrix/function
  R_ = R_in; // measurement covariance matrix
  Q_ = Q_in; // process covariance matrix

  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

	std::cout << "x=" << std::endl << x_ << std::endl;
	std::cout << "P=" << std::endl << P_ << std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

	MatrixXd I = MatrixXd::Identity(4, 4); // Identity matrix

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO:
	* update the state by using Extended Kalman Filter equations
	*/
	double px = x_(0),
		py = x_(1),
		vx = x_(2),
		vy = x_(3);

	double rho = pow(pow(x_(0), 2) + pow(x_(1), 2), 0.5),
		phi = atan2(py, px),
		rhodot = (px * vx + py * vy) / rho;

	if (fabs(rho) < 0.01) {
		cout << "UpdateEKF() - Warning - Division by or close to Zero" << endl;
	}

	VectorXd z_pred(3);
	z_pred << rho, phi, rhodot;

	MatrixXd I = MatrixXd::Identity(4, 4); // Identity matrix

	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	P_ = (I - K * H_) * P_;

}

