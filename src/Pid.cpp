#include "Pid.h"


Pid::Pid() : error_sum_(0), error_last_(0),target_(0)
{
}

void Pid::setTarget(double target)
{
	this->target_=target;
}

void Pid::setKp(double Kp)
{
	this->Kp_=Kp;
}

void Pid::setKi(double Ki)
{
	this->Ki_=Ki;
}

void Pid::setKd(double Kd)
{
	this->Kd_=Kd;
}

void Pid::setMaxWindup(double max_windup)
{
	this->max_windup_=max_windup;
}

void Pid::setAlpha(double alpha)
{
	this->alpha_=alpha;	
}


double Pid::update(double measure,double dt)
{
	double error = this->target_ - measure;
	double error_delta = error - error_last_;
	this->error_sum_ += error * dt;

	if (error == 0 && target_ == 0) 
		this->error_sum_=0;
	
	/*
	if (this->error_sum_>this->max_windup_) 
		this->error_sum_=this->max_windup_;
	else if (this->error_sum_<-this->max_windup_) 
		this->error_sum_=-this->max_windup_;
	*/

	this->error_last_ = error;

	double p = this->Kp_ * error;
	double i = this->Ki_ * error_sum_;
	//float d = this->Kd_ * ( this->alpha_ * error_delta/dt + (1-this->alpha_) * error);
	double d = this->Kd_ *  this->alpha_ * (error_delta/dt);

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