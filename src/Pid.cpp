#include "Pid.h"


Pid::Pid() : error_sum_(0), error_last_(0)
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

void Pid::setKi(float Ki)
{
	this->Ki_=Ki;
}

void Pid::setKd(float Kd)
{
	this->Kd_=Kd;
}

void Pid::setMaxWindup(float max_windup)
{
	this->max_windup_=max_windup;
}

void Pid::setAlpha(float alpha)
{
	this->alpha_=alpha;	
}


float Pid::update(float measure,float dt)
{
	float error = this->target_ - measure;
	float error_delta = error - error_last_;
	this->error_sum_ += error * dt;
	
	/*
	if (this->error_sum_>this->max_windup_) 
		this->error_sum_=this->max_windup_;
	else if (this->error_sum_<-this->max_windup_) 
		this->error_sum_=-this->max_windup_;
	*/

	this->error_last_ = error;

	float p = this->Kp_ * error;
	float i = this->Ki_ * error_sum_;
	float d = this->Kd_ * ( this->alpha_ * error_delta/dt + (1-this->alpha_) * error);

	float u = p + i + d;

	#ifdef PID_DEBUG
  	Serial.print("Pid::update:");
  	Serial.print("\t");
  	Serial.print(this->target_);
  	Serial.print("\t");
  	Serial.print(measure);
	Serial.print("\t");
  	Serial.print(p);
	Serial.print("\t");
  	Serial.print(i);
	Serial.print("\t");
  	Serial.print(d);
	Serial.print("\t");
  	Serial.print(u);
  	Serial.print("\n");
  	#endif
	return u;
}

void Pid::reset()
{
	this->Kp_=0; 	
  	this->Ki_=0; 	
  	this->Kd_=0;
  	this->target_=0;
  	this->error_sum_=0;	
  	this->error_last_=0;
}