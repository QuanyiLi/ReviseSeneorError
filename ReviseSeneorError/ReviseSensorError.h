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
	float m_Altitude;//���������ѹ�Ƹ߶�
	Vector3r m_Mag;//������Ĵ���������
	Vector3r m_Acc;//��������߼��ٶ�
	Vector3r m_Gyro;//������Ľ��ٶ�

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
