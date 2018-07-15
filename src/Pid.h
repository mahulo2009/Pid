#ifndef Pid_H
#define Pid_H

#include "Arduino.h"

//#define PID_DEBUG 1

class Pid {
  public:
	Pid();
	void setTarget(float target);
	void setKp(float Kp);
	void setKi(float Ki);
	void setKd(float Kd);
	void setMaxWindup(float max_windup);
	void setAlpha(float alpha);
	float update(float measure,float dt);
	void reset();
  private:
  	float Kp_; 	
  	float Ki_;
  	float Kd_;
  	float max_windup_;
  	float alpha_;
  	float target_;
  	float error_sum_;
  	float error_last_;
};
#endif
