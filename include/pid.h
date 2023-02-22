#pragma once

#include <iostream>
#include <cmath>

class PID {
    public:
        PID(double kP, double kI=0.0, double kD=0.0);
        ~PID(){}
        double Kp, Ki, Kd;
        double prevError;
        double _integral, _derivative;
        double compute(double target, double current, double dt=0.1);
    private:
};