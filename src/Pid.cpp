#include "Pid.h"


Pid::Pid() 
{
}

void Pid::setTarget(float target)
{
	this->target_=target;
}
void Pid::setKp(float Kp)
{
	this->Kp_=Kp;
}
float Pid::update(float measure)
{
	float error = this->target_ - measure;
	float p = this->Kp_ * error;
	float u = p;
	#ifdef PID_DEBUG
  	Serial.print("Pid::update:");
  	Serial.print("\t");
  	Serial.print(this->target_);
  	Serial.print("\t");
  	Serial.print(measure);
	Serial.print("\t");
  	Serial.print(p);
	Serial.print("\t");
  	Serial.print(u);
  	Serial.print("\n");
  	#endif
	return u;
}
