#include <iostream>
#include <cmath>
#include "pid.h"

PID::PID(double kP, double kI, double kD) {
    Kp = kP;
    Ki = kI;
    Kd = kD;
    prevError    = 0.0;
    _integral    = 0.0;
    _derivative  = 0.0;
}

double PID::compute(double target, double current, double dt) {
    double currError = target - current;

    _derivative      = currError - this->prevError;
    this->_integral += currError;
    
    this->prevError = currError;
    double control  = ( this->Kp * currError ) + 
                            ( this->Ki * this->_integral * dt ) + 
                                ( this->Kd * this->_derivative / dt );
    return control;
}
