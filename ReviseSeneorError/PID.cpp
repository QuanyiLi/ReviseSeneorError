#include "PID.h"
#include "Drone_data.h"
#define CLIP3(_n1, _n, _n2)                                                    \
  {                                                                            \
    if (_n < _n1)                                                              \
      _n = _n1;                                                                \
    if (_n > _n2)                                                              \
      _n = _n2;                                                                \
  }



//不同高度PID用的不一样，在比较高的地方调参得到的参数，在低点使用时，可能
//会使得四轴摆动幅度太大。在低点调节得到的参数，在高点上使用没什么问题，就是
//慢了一点。
//double kp = -0.01, ki = 0.0, kd = 0.0;// -445高度下的OK。-450高度时，这个参数幅度太大会偏。
double kp = -0.002, ki = -0.0, kd = 0.00001;//-450高度下的。OK -0.00002;
											//double kp = -0.01, ki = 0.0, kd = 0.0;//OK

double vkp = -0.01, vki = 0, vkd = 0;

double PID::PIDY(double error, double y_max, double tolerance)
{
	double control_pitch;

	if (error < -tolerance || error > tolerance)
	{
		control_pitch = pid_x.getOutput(error, 0.3);//300ms
		CLIP3(-y_max, control_pitch, y_max);
	}
	else
	{
		control_pitch = 0.0;
	}

	printf("control_pitch;%f\n", control_pitch);
	return control_pitch;
}
double PID::PIDX(double error, double x_max, double tolerance) {
	double control_roll;

	if (error < -tolerance || error > tolerance)
	{
		control_roll = pid_x.getOutput(error, 0.3);//300ms
		CLIP3(-x_max, control_roll, x_max);
	}
	else
	{
		control_roll = 0.0;
	}

	printf("control_roll;%f\n", control_roll);
	return control_roll;
}

double PID::PIDXY(double error, double v_max, bool is_X) {
	double targetv, control;

	return control;
}
//double PID::PIDXY(double error, double v_max, bool is_X) {
//  double targetv, control;
//  double kp = 2.0;
//  if (is_X) {
//    targetv = (-2 * error + lasterrorx_) * kp;
//    lasterrorx_ = error;
//  }
//  else {
//    targetv = (-2 * error + lasterrory_) * kp;
//    lasterrory_ = error;
//  }
//
//  if (error > 80 || error < -80) {
//    targetv += (-error + 80) * kp;
//  }
//  CLIP3(-v_max, targetv, v_max);
//
//  if (is_X) {
//    //control = pid_vx_.getOutput(targetv - thread_.navdata.vx, 0.5);
//  }
//  else {
//    //control = pid_vy_.getOutput(targetv - thread_.navdata.vy, 0.5);
//  }
//  control /= 15000;
//  return control;
//}


double PID::PIDZ(double reference, double tolerance) {
	double upd, control_stuff;
	double kp = 0.1;


	//获取圆形标识牌的半径
	//control_stuff = find_rob_.getGroundCenterRadius();

	control_stuff = Barometer_data.altitude;

	if (control_stuff < reference) {
		upd = kp * (reference - control_stuff);
	}
	else if (control_stuff > reference + tolerance) {
		upd = kp * (reference + tolerance - control_stuff);
	}
	else {
		upd = 0;
	}
	CLIP3(-0.3f, upd, 0.3f);
	return upd;
}

void PID::PIDReset() {

	pid_vx.reset();
	pid_vy.reset();
	pid_x.reset();
	pid_y.reset();

	pid_vx.setParam(vkp, vki, vkd, 2);
	pid_vy.setParam(vkp, vki, vkd, 2);

	pid_x.setParam(kp, ki, kd, 2);
	pid_y.setParam(kp, vki, kd, 2);
}

