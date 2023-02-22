#pragma once

#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

enum VehicleType { Differential,Ackermann };

class Vehicle {
    public:
        double x, y; // Position
        double yaw, maxSteer;  // Orientation
        double linVel, angVel;  // Linear & Angular Velocity
        double length; // Vehicle Dimension
        VehicleType type;

        Vehicle(double x, double y, double yaw, 
                VehicleType vehicleType=Ackermann,double maxSteer=M_PI/3,
                double bodyLength=1.0,double initLinVel=0.0, double initAngVel=0.0);

        void updateVelocity(double linVel, double angVelOrSteer, double dt=0.4);
        
        std::vector<std::vector<double> > drawVehicle(Vehicle vehicle,float L,float W);
};