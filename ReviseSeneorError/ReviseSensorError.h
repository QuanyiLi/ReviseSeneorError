#pragma once
#ifndef	REVISESENSORERROR_H
#define REVISESENSOR_H

#include<Eigen\Dense>
#include "client.h"

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

typedef Vector3f  Vector3r;

class ReviseSensorError {
private:
	float m_Altitude;//修正后的气压计高度
	Vector3r m_Mag;//修正后的磁力计数据
	Vector3r m_Acc;//修正后的线加速度
	Vector3r m_Gyro;//修正后的角速度

	void ReviseAltitude();
	void ReviseMag();
	void ReviseAcc();
	void ReviseGyro();
public:
	ReviseSensorError():m_Altitude(0),m_Mag(0,0,0),m_Acc(0,0,0),m_Gyro(0,0,0){};
	float getAltitude();
	Vector3r getMag();
	Vector3r GetAcc();
	Vector3r GetGyro();
};
#endif // !REVISESENSORERROR_H
