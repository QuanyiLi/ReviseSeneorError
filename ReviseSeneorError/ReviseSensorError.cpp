#include "ReviseSensorError.h"

void ReviseSensorError::ReviseAltitude()
{
	float altitude1 = client.getBarometerdata().altitude;
	float altitude2 = client.getBarometerdata().altitude;
	float altitude3 = client.getBarometerdata().altitude;
	float altitude4 = client.getBarometerdata().altitude;
	float altitude5 = client.getBarometerdata().altitude;

	m_Altitude = (altitude1+ altitude2+ altitude3+ altitude4+ altitude5) / 5.0;
}

void ReviseSensorError::ReviseMag()
{
	Vector3r mag1 = client.getMagnetometerdata().magnetic_field_body;
	Vector3r mag2 = client.getMagnetometerdata().magnetic_field_body;
	Vector3r mag3 = client.getMagnetometerdata().magnetic_field_body;
	Vector3r mag4 = client.getMagnetometerdata().magnetic_field_body;
	Vector3r mag5 = client.getMagnetometerdata().magnetic_field_body;

	m_Mag(0, 0) = (mag1(0, 0), mag2(0, 0), mag3(0, 0), mag4(0, 0), mag5(0, 0)) / 5.0;
	m_Mag(1, 0) = (mag1(1, 0), mag2(1, 0), mag3(1, 0), mag4(1, 0), mag5(1, 0)) / 5.0;
	m_Mag(2, 0) = (mag1(2, 0), mag2(2, 0), mag3(2, 0), mag4(2, 0), mag5(2, 0)) / 5.0;
}

void ReviseSensorError::ReviseAcc()
{
	Vector3r  acc1 = client.getImudata().linear_acceleration;
	Vector3r  acc2 = client.getImudata().linear_acceleration;
	Vector3r  acc3 = client.getImudata().linear_acceleration;
	Vector3r  acc4 = client.getImudata().linear_acceleration;
	Vector3r  acc5 = client.getImudata().linear_acceleration;

	m_Acc(0, 0) = (acc1(0, 0), acc2(0, 0), acc3(0, 0), acc4(0, 0), acc5(0, 0)) / 5.0;
	m_Acc(1, 0) = (acc1(1, 0), acc2(1, 0), acc3(1, 0), acc4(1, 0), acc5(1, 0)) / 5.0;
	m_Acc(2, 0) = (acc1(2, 0), acc2(2, 0), acc3(2, 0), acc4(2, 0), acc5(2, 0)) / 5.0;
}

void ReviseSensorError::ReviseGyro()
{
	Vector3r gyro1 = client.getImudata().angular_velocity;
	Vector3r gyro2 = client.getImudata().angular_velocity;
	Vector3r gyro3 = client.getImudata().angular_velocity;
	Vector3r gyro4 = client.getImudata().angular_velocity;
	Vector3r gyro5 = client.getImudata().angular_velocity;

	m_Gyro(0, 0)= (gyro1(0, 0), gyro2(0, 0), gyro3(0, 0), gyro4(0, 0), gyro5(0, 0)) / 5.0;
	m_Gyro(1, 0) = (gyro1(1, 0), gyro2(1, 0), gyro3(1, 0), gyro4(1, 0), gyro5(1, 0)) / 5.0;
	m_Gyro(2, 0) = (gyro1(2, 0), gyro2(2, 0), gyro3(2, 0), gyro4(2, 0), gyro5(2, 0)) / 5.0;
}

float ReviseSensorError::getAltitude()
{
	ReviseAltitude();
	return m_Altitude;
}

Vector3r ReviseSensorError::getMag()
{
	ReviseMag();
	return m_Mag;
}

Vector3r ReviseSensorError::GetAcc()
{
	ReviseAcc();
	return m_Acc;
}

Vector3r ReviseSensorError::GetGyro()
{
	ReviseGyro();
	return m_Gyro;
}
