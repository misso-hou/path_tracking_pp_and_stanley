#include <iostream>
#include <cmath>
#include "vehicle.h"

using namespace std;

Vehicle::Vehicle(double x, double y, double yaw, VehicleType vehicleType,double maxSteer,double bodyLength,double initLinVel, double initAngVel) {
    this->x     = x;
    this->y     = y;
    this->yaw   = yaw;
    this->linVel    = initLinVel; 
    this->angVel    = initAngVel;
    this->length    = bodyLength;
    this->maxSteer  = maxSteer;
    this->type      = vehicleType;
}

void Vehicle::updateVelocity(double linVel, double angVelOrSteer, double dt) {
    this->x = this->x + (this->linVel * cos(this->yaw) * dt);
    this->y = this->y + (this->linVel * sin(this->yaw) * dt);

    if (this->type == Ackermann)
        this->yaw = this->yaw + (this->linVel / this->length * tan(angVelOrSteer) * dt);    // angVelOrSteer - Steering Angle
    else { // Differential
        this->yaw = this->yaw + (this->angVel * dt);
        this->angVel = angVelOrSteer; // angVelOrSteer - Angular Velocity
    }
    this->linVel = linVel;    // linVel - Linear Velocity
}

vector<vector<double> > Vehicle::drawVehicle(Vehicle vehicle,float L,float W) {
    vector<vector<double>> body_points;
    body_points = {{-0.2*L,0.5*W},{0.8*L,0.5*W},{0.8*L,-0.5*W},{-0.2*L,-0.5*W},{-0.2*L,0.5*W}};
    vector<vector<double>> body_points_rotate;
    for(auto p:body_points){
        double local_x = vehicle.x+p[0]*cos(vehicle.yaw) - p[1]*sin(vehicle.yaw);
        double local_y = vehicle.y+p[0]*sin(vehicle.yaw) + p[1]*cos(vehicle.yaw);
        body_points_rotate.push_back({local_x,local_y});
    }
    return body_points_rotate;
}
