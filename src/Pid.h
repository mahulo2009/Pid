#ifndef Pid_H
#define Pid_H

#include "Arduino.h"

#define PID_DEBUG 1

class Pid {
  public:
	Pid();
	void setTarget(float target);
	void setKp(float Kp);
	float update(float measure);
  private:
  	float Kp_; 	
  	float target_;
  	long start_times_;
  	long last_timestamp_;
};
#endif
