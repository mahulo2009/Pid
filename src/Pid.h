#ifndef Pid_H
#define Pid_H

#include "Arduino.h"

//#define PID_DEBUG 1

class Pid {
  public:
	Pid();
	void setTarget(double target);
	void setKp(double Kp);
	void setKi(double Ki);
	void setKd(double Kd);
	void setMaxWindup(double max_windup);
	void setAlpha(double alpha);
	double update(double measure,double dt);
	void reset();
  private:
  	double Kp_; 	
  	double Ki_;
  	double Kd_;
  	double max_windup_;
  	double alpha_;
  	double target_;
  	double error_sum_;
  	double error_last_;
};
#endif
