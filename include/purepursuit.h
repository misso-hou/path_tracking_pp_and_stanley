#pragma once

#include <iostream>
#include <cmath>
#include <bits/stdc++.h>
#include "utilis.h"
#include "vehicle.h"

using namespace std; 

namespace PP{
struct PursuitData {
    double  radiusOrSteer;  // Radius of the Curve or Steer Angle
    int     targetIdx;      // Target Index of the Trajectory Points
    PursuitData(double r, int t) : radiusOrSteer(r), targetIdx(t) {} 
};
// Pure Pursuit Controller(Adaptive) //
class PurePursuit:public CommonUse {
    public:
    double lD, klD; // Look Ahead Distance and its Gain
    PurePursuit(double klD, double lD) : lD(lD), klD(klD) {} 
    int getTargetIndex(Vehicle vehicle, vector<vector<double>> points);
    PursuitData purePursuitControl(Vehicle vehicle, vector<vector<double>> points, int targetIdx);
};
}

/*
// Parameters - Pure Pursuit Controller
double klD  = 0.6;  // Look Ahead Gain
double lD   = 2.0;  // Look Ahead Distance
// Parameters - PID Controller
double kP   = 4.0;  // Proportional Gain
double kI   = 0.0;  // Proportional Gain
double kD   = 0.1;  // Proportional Gain
// Parameters - Vehicle
double x    = 0.0;  // Initial Position
double y    = 0.0;
double yaw  = 0.0;
double vel  = 1.0;  // Initial Target Velocity
double maxSpeed = 6.0; // m/s

// PID Controller
PID pid = PID(kP, kI, kD);
// Pure Pursuit Controller
PurePursuit pursuit = PurePursuit(klD, lD);
// Vehicle Simulation
Vehicle diffDrive(x, y, yaw);
int main() {
    double time = 0.0, maxTime = 60, dt = 0.3;    // seconds

    // Trajectory Points
    vector<vector<double>> path;
    for (double i=0; i<60; i+=1)
        path.push_back({i, sin(i / 5.0) * i / 1.5});

    // Velocity Profile (Manhattan Distance Based)
    vector<double> targetSpeed({vel});
    for (int i=0; i<path.size()-1; i++)
        targetSpeed.push_back(min(maxSpeed, (abs(path[i][0]-path[i+1][0]) + abs(path[i][1]-path[i+1][1])) * 2.0 ));

    double acceleration, currVelocity = 0.0;
    PursuitData result(0.0, pursuit.getTargetIndex(diffDrive, path));
    while ((time <= maxTime) & (result.targetIdx < path.size()-1)) {
        result = pursuit.purePursuitControl(diffDrive, path, result.targetIdx);
        acceleration = pid.compute(targetSpeed[result.targetIdx], diffDrive.linVel, dt);

        currVelocity += acceleration*dt;
        diffDrive.updateVelocity(currVelocity, currVelocity/result.radiusOrSteer, dt);

        cout << path[min(int(path.size()-1), int(time/dt))][0] << " " << path[min(int(path.size()-1), int(time/dt))][1] << " ";
        cout << diffDrive.x << " " << diffDrive.y << endl;
        time += dt;
    }
    // diffDrive.updateAcceleration(2.0, 2.0/4.0);
    // for (int i=0; i<100; i++) {
    //     diffDrive.updateVelocity(2.0, 2.0/4.0);
    //     cout << diffDrive.x << " " << diffDrive.y << endl;
    // }
    return 1;
}  
*/