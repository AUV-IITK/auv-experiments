// Copyright 2018 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>
#include <errorToPWM.h>

ErrorDescriptor::ErrorDescriptor(std::string _name): p_(2.4), i_(0), d_(0.5),
    band_(1), seed_(0), error_(0), reference_value_(0), previous_value_(0), pwm_(0), previous_time_stamp_(0), 
    present_time_stamp_(0)
{   
    this->name_ = _name;
}

ErrorDescriptor::~ErrorDescriptor() {}

void ErrorDescriptor::setPID(float new_p, float new_i, float new_d, float new_band) {
    this->p_ = new_p;
    this->i_ = new_i;
    this->d_ = new_d;
    this->band_ = new_band;
    ROS_INFO("PID set to P: %f, I: %f, D: %f, BAND: %f", this->p_, this->i_, this->d_, this->band_);
}

void ErrorDescriptor::setReference(double _value) {
    this->reference_value_ = _value;
}

void ErrorDescriptor::setType(std::string _name) {
    this->name_ = _name;
}

void ErrorDescriptor::errorToPWM(double _current_value) {

    if (this->seed_ == 0) {
        this->present_time_stamp_ = ros::Time::now().toSec();
        this->seed_++;
    }

    this->current_value_ = _current_value;

    this->previous_time_stamp_ = this->present_time_stamp_;
    this->present_time_stamp_ = ros::Time::now().toSec();

    if (this->name_ == "ANGLE") {
        if (this->reference_value_ >= 180)
            this->reference_value_ = this->reference_value_ - 360;
        else if (this->reference_value_ <= -180)
            this->reference_value_ = this->reference_value_ + 360;
    }

    float derivative = 0, integral = 0;
    double dt = this->present_time_stamp_ - this->previous_time_stamp_;

    this->error_ = this->reference_value_ - _current_value;

    if (this->name_ == "ANGLE") {
        if (this->error_ < 0)
            this->error_value_ = this->error_ + 360;
        else
            this->error_value_ = this->error_ - 360;

        if (abs(this->error_value_) < abs(this->error_))
            this->error_ = this->error_value_;
    }

    std::cout << "ERROR: " << this->error_ << std::endl;
    integral += (this->error_ * dt);
    derivative = (_current_value - this->previous_value_) / dt;
    double output = (this->p_ * this->error_) + (this->i_ * integral) + (this->d_ * derivative);

    turningOutputPWMMapping(7*output);

    if (this->error_ < this->band_ && this->error_ > -this->band_)
    {
        std::cout << "ERROR: " << "1" << std::endl;
        this->pwm_ = 0;
    }
}

void ErrorDescriptor::turningOutputPWMMapping(float output) // to keep PWM values within a limit
{
    float maxOutput = 1000, minOutput = -maxOutput;
    float scale = 255 / maxOutput;
    if (output > maxOutput)
        output = maxOutput;
    if (output < minOutput)
        output = minOutput;
    float temp = output * scale;
    int output_pwm = static_cast<int>(temp);
    if (output_pwm > 255)
        output_pwm = 255;
    if (output_pwm < -255)
        output_pwm = -255;
    this->pwm_ = output_pwm;
}

int ErrorDescriptor::getPWM() {
    return this->pwm_;
}

double ErrorDescriptor::getCurrentValue() {
    return this->current_value_;
}