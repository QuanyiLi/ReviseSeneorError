#ifndef ATTITUDE_H
#define ATTITUDE_H

//#include "Vector3.h"
//#include "Matrix3.h"
 
#include<iostream>
#include<Eigen\Dense>

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

using namespace std;

#define RtA     57.324841f  //角度转化为弧度   
#define AtR     0.0174533f  //弧度转化为角度
typedef Vector3f  Vector3r;

class AttitudeCalculation {
private:
	Vector3r mAngle;
	Matrix3f mRotateMatrix;
	float q0, q1, q2, q3;     //quaternion of sensor frame relative to auxiliary frame 
	float dq0, dq1, dq2, dq3;  //quaternion of sensor frame relative to auxiliary frame 
	float gyro_bias[3]; // bias estimation 
	float q0q0, q0q1, q0q2, q0q3;
	float q1q1, q1q2, q1q3;
	float q2q2, q2q3;
	float q3q3;
	unsigned char bFilterInit;
public:
	AttitudeCalculation();
	void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
	void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);
	Vector3f GetAngle(Vector3f acc, Vector3f gyro, float deltaT);
	Vector3r GetAngle(Vector3r acc, Vector3r gyro, Vector3r mag, float deltaT);
};
#endif