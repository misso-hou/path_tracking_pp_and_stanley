#pragma once
#include <iostream>
#include <cmath>
#include <bits/stdc++.h>
#include <boost/circular_buffer.hpp>
#include "vehicle.h"
#include "utilis.h"

namespace Stanley{
using namespace std; 

struct StanleyData {
    double  radiusOrSteer;  // Radius of the Curve or Steer Angle
    int     targetIdx;      // Target Index of the Trajectory Points
    StanleyData(double rS, int tIdx) : radiusOrSteer(rS), targetIdx(tIdx) {}
};

// Stanley Controller //
class StanleyController:public CommonUse {
    public:
        double kE;  // Control Gain
        double kS;  // Softening Constant

        StanleyController(double kE, double kS = 1e-9) : kE(kE), kS(kS) {}
        /*find the closet point on path(goal point)*/
        int getTargetIndex(Vehicle vehicle, vector<vector<double>> points);
        double calculateEFA(Vehicle vehicle, vector<vector<double>> points, int targetIdx);
        /*stenley algorithm*/
        StanleyData stanleyControl(Vehicle vehicle, vector<vector<double>> points, int targetIdx);
};
} //end of stanley namespace
