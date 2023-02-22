#include <iostream>
#include <cmath>
#include <bits/stdc++.h>
#include <boost/circular_buffer.hpp>
#include <stanley.h>

namespace Stanley{

using namespace std; 
// CommonUse common;
/*find the closet point on path(goal point)*/
int StanleyController::getTargetIndex(Vehicle vehicle, vector<vector<double>> points) {
    vector<double> dist;    // Distance between Vehicle and Trajectory Points
    double frontAxleX = vehicle.x + vehicle.length*cos(vehicle.yaw);
    double frontAxleY = vehicle.y + vehicle.length*sin(vehicle.yaw); 
    for (auto point:points){
        dist.push_back(sqrtf(pow(frontAxleX - point[0], 2) + pow(frontAxleY - point[1], 2)));
    }
    vector<double>::iterator it = find(dist.begin(), dist.end(), *min_element(dist.begin(), dist.end()));
    int index = distance(dist.begin(), it);
    return index;
}

double StanleyController::calculateEFA(Vehicle vehicle, vector<vector<double>> points, int targetIdx) {
    // Reference : https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    // Line defined by Point and Angle
    // /*
    double pX = vehicle.x;
    double pY = vehicle.y;
    double pTheta = vehicle.yaw;

    double x0 = points[targetIdx][0];
    double y0 = points[targetIdx][1];

    double errorFrontAxle = cos(pTheta)*(pY - y0) - sin(pTheta)*(pX - x0);
    return -errorFrontAxle;
    /*
    // Line defined by Two Points
    double x0 = points[targetIdx][0];
    double y0 = points[targetIdx][1];
    // Point1(P1)
    double x1 = vehicle.x;
    double y1 = vehicle.y;
    // Point2(P2)
    double x2 = vehicle.x + vehicle.length*cos(vehicle.yaw);
    double y2 = vehicle.y + vehicle.length*sin(vehicle.yaw);
    double errorFrontAxle = ( (x2-x1)*(y1-y0) - (x1-x0)*(y2-y1) ) / sqrtf(pow(x2-x1, 2) + pow(y2-y1, 2));
    return -errorFrontAxle;
    */
}

/*stenley algorithm*/
StanleyData StanleyController::stanleyControl(Vehicle vehicle, vector<vector<double>> points, int targetIdx) {
    /*step01->calculate the target point*/
    int index = max(targetIdx, getTargetIndex(vehicle, points));
    /*step02->calculate heading error:path angle - yaw angle*/
    double trajectYaw = ((index != 0) ? atan2(points[index][1]-points[index-1][1], points[index][0]-points[index-1][0]) : 
                        atan2(points[index+1][1]-points[index][1], points[index+1][0]-points[index][0]) );
    double thetaE = normalizeAngle(trajectYaw - vehicle.yaw);         // Heading Error
    /*step03->calculate the lateral error angle*/
    double efa = calculateEFA(vehicle, points, index);                //lateral error
    double thetaD = atan2(this->kE * efa, this->kS + vehicle.linVel); // Cross Track Error
    /*step04->steer angle calculate*/
    double delta = thetaE + thetaD;
    if (vehicle.type == Differential) {
        return StanleyData(delta, index);
    } else if (vehicle.type == Ackermann){
        delta = max(-vehicle.maxSteer, min(delta, vehicle.maxSteer));
        return StanleyData(delta, index);
    }
}
}

