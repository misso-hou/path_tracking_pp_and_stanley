#pragma once
#include <iostream>

using namespace std;

class CommonUse {
    public:
        CommonUse(){}
        ~CommonUse(){}
        double normalizeAngle(double angle){
            double local_angle = angle;
            while (local_angle > M_PI)    local_angle -= 2.0*M_PI;
            while (local_angle < -M_PI)   local_angle += 2.0*M_PI;
            return local_angle;
        };
    private:

};


