
/*
* main.cpp
*
*  Created on: June 18, 2016
*      Author: ljw
*/
#ifndef PID_H__
#define PID_H__
#include "PIDController.h"


extern double kp, ki, kd;
extern double vkp, vki, vkd;
//extern PIDController PID_position_x;//位置控制PID
//extern PIDController PID_position_y;//位置控制PID

class PID {
public:
	//PID(ROSThread &thread, FindRob &find_rob): thread_(thread), 
	//    find_rob_(find_rob), lasterrorx_(0), lasterrory_(0) {
	//  
	//  double vkp = 5, vkd = 20, vki = 0;
	//  pid_vx_.setParam(vkp, vki, vkd, 2);
	//  pid_vy_.setParam(vkp, vki, vkd, 2);
	//}
	PID() {



		pid_vx.reset();
		pid_vy.reset();
		pid_x.reset();
		pid_y.reset();

		pid_vx.setParam(vkp, vki, vkd, 2);
		pid_vy.setParam(vkp, vki, vkd, 2);

		pid_x.setParam(kp, ki, kd, 2);
		pid_y.setParam(kp, vki, kd, 2);

	}

	~PID() {}
	double PID::PIDX(double error, double x_max, double tolerance);
	double PID::PIDY(double error, double y_max, double tolerance);
	double PIDXY(double error, double v_max, bool is_X = true);
	double PID::PIDZ(double reference, double tolerance);
	void PIDReset();
private:
	PIDController pid_vx;//速度控制
	PIDController pid_vy;
	PIDController pid_x;//位置控制
	PIDController pid_y;
	/* double lasterrorx_;
	double lasterrory_;*/
};
#endif /*PID_H__*/
