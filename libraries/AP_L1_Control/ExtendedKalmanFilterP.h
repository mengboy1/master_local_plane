/*
Extended Kalman Filter class by Sam Tabor, 2013.
* http://diydrones.com/forum/topics/autonomous-soaring
* Set up for identifying thermals of Gaussian form, but could be adapted to other
* purposes by adapting the equations for the jacobians.
*/

#pragma once

#include <AP_Math/matrixN.h>

#define EKF_N 4


class ExtendedKalmanFilterP {
public:
    ExtendedKalmanFilterP(void) {}

    VectorN<float, EKF_N> X;
    MatrixN<float, EKF_N> P;
    MatrixN<float, EKF_N> Q;
    float R;
    void reset(const VectorN<float, EKF_N> &x, const MatrixN<float, EKF_N> &p, const MatrixN<float, EKF_N> q, float r);
    void update(float z, float Vx, float Vy);
    float prediction();

private:
    float _z;
    float measurementpredandjacobian(VectorN<float, EKF_N> &A);
};
