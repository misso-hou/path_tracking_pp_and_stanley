#include <iostream>
#include <fstream>
#include <boost/circular_buffer.hpp>
#include "matplotlibcpp.h"
#include "pid.h"
#include "vehicle.h"
#include "stanley.h"
#include "purepursuit.h"

using namespace std;
namespace plt = matplotlibcpp;
using namespace Stanley;
using namespace PP;

#define STANLEY

/*get path points*/
string filePath = "/home/hou/work/workspace/path_tracking_test/data/trajectory_points.txt";
auto isPathExists = [](string path) {
  struct stat buffer;
  return (stat (path.c_str(), &buffer) == 0);
};

vector<vector<double>> loadPointsFromFile() {
  vector<vector<double>> Points;
  if (isPathExists(filePath))
    cout << "INFO : Saved points file exists!" << endl;
  else {
    cout << "INFO : Saved points file does not exists!" << endl;
    // Return empty vector points
    return Points;
  }

  ifstream inFile;
  string line;
  inFile.open(filePath);

  double pX, pY;
  while (std::getline(inFile, line)) {
    istringstream dataStream(line);

    dataStream >> pX;
    dataStream >> pY;

    Points.push_back(vector<double>{pX, pY});
    if ((pX==0) & (pY==15))     break;
  }
  return Points;
}

#if defined(STANLEY)
// Parameters - Stanley Controller
double kE   = 5.0;  // Control Gain
double kS   = 0.0;  // 1e-12; // Softening Constant
// Parameters - PID Controller
double kP   = 1.0;  // Proportional Gain
double kI   = 0.0;  // Integral Gain
double kD   = 0.0;  // Derivative Gain
// Parameters - Vehicle
double x    = 1.0;  // Initial Position
double y    = -10.0;
double yaw  = 0.6;
double vel  = 0.04;  // Initial Target Velocity
double maxSpeed = 2.0;      // m/s
double maxSteer = M_PI/3.0; // 60 deg
StanleyController stanley = StanleyController(kE, kS); // Stanley Controller

#elif defined(PP)
// Parameters - Pure Pursuit Controller
double klD  = 0.6;  // Look Ahead Gain
double lD   = 1.0;  // Look Ahead Distance
// Parameters - PID Controller
double kP   = 4.0;  // Proportional Gain
double kI   = 0.0;  // Proportional Gain
double kD   = 0.1;  // Proportional Gain
// Parameters - Vehicle
double x    = 0.0;  // Initial Position
double y    = -15.0;
double yaw  = 0.0;
double vel  = 0.5;  // Initial Target Velocity
double maxSpeed = 3.0; // m/s
vector<double> goalPose{0, 15};   // Goal Pose
double goalTolerance = 0.3;
PurePursuit pursuit = PurePursuit(klD, lD);
#endif

double L = 2;
double W = 1.4;
/*define the controller and vehicle*/
PID pid = PID(kP, kI, kD); // PID Controller
Vehicle vehicle(x,y,yaw,VehicleType::Differential);

int main() {
    double time = 0.0, maxTime = 60, dt = 0.1;    // seconds
    /*step01->create path*/
    vector<vector<double>> path = loadPointsFromFile();
    vector<double> x, y;
    for (auto p:path) {
        x.push_back(p[0]);
        y.push_back(p[1]);
    }
    /*step02->Velocity Profile (Manhattan Distance Based)*/
    vector<double> targetSpeed;
    for (int i=0; i<path.size()-1; i++){
        #if defined(STANLEY)
        targetSpeed.push_back(min(maxSpeed, (abs(path[i][0]-path[i+1][0]) + abs(path[i][1]-path[i+1][1] + 0.15)) * 2.2 ));
        #elif defined(PP)
        targetSpeed.push_back(min(maxSpeed, (abs(path[i][0]-path[i+1][0]) + abs(path[i][1]-path[i+1][1] + 0.15)) * 8.0 ));
        #endif
        // targetSpeed.push_back(2.5); 
    }
    
    /*step03->Calculate the control command*/
    double acceleration, currVelocity = 0.0;
    #if defined(STANLEY)
    StanleyData result(0.0, stanley.getTargetIndex(vehicle, path)); //give the first goal point on path
    #elif defined(PP)
    PursuitData result(0.0, pursuit.getTargetIndex(vehicle, path));  //steer angle and goal index on path
    #endif
    boost::circular_buffer<double> poseX(300), poseY(300),currVel(1000), targVel(1000), currOmega(1000),timeVel(1000);
    while ((time <= maxTime) & (result.targetIdx < path.size()-1)) {
        /*get steer command and velocity command*/
        #if defined(STANLEY)
        result = stanley.stanleyControl(vehicle, path, result.targetIdx);
        //!!!: find the resion of pp diff wheel not tracking well
        #elif defined(PP)
        result = pursuit.purePursuitControl(vehicle, path, result.targetIdx);
        #endif
        acceleration = pid.compute(targetSpeed[result.targetIdx], vehicle.linVel, dt);
        currVelocity += acceleration*dt;
        /*state update*/
        // vehicle.updateVelocity(currVelocity, currVelocity/result.radiusOrSteer, dt); //update vehicle state
        vehicle.updateVelocity(currVelocity, result.radiusOrSteer, dt); //update vehicle state
        time += dt;
        /*store the vehicle states*/
        poseX.push_back(vehicle.x);
        poseY.push_back(vehicle.y);     //pose of the vehicle
        currVel.push_back(currVelocity);
        targVel.push_back(targetSpeed[result.targetIdx]);   
        currOmega.push_back(result.radiusOrSteer);  //velocity of the vehicle 
        timeVel.push_back(time);
        /*vehicle body and direction display*/
        vector<vector<double>> vehicle_body = vehicle.drawVehicle(vehicle,L,W);
        vector<double> body_x, body_y;
        for (auto p:vehicle_body) {
            body_x.push_back(p[0]);
            body_y.push_back(p[1]);
        }
        vector<double> quiver_x,quiver_y,x_dir,y_dir;
        quiver_x = {L*0.8*cos(vehicle.yaw)+vehicle.x};
        quiver_y = {L*0.8*sin(vehicle.yaw)+vehicle.y};
        x_dir = {1.0*cos(vehicle.yaw)};
        y_dir = {1.0*sin(vehicle.yaw)};
        
        plt::clf();
        plt::title("Stanley : Path Tracking");  // Add Graph Title
        plt::subplot(1,2,1);
        plt::named_plot("Trajectory", x, y, "b:");
        plt::named_plot("Position", vector<double>(poseX.begin(), poseX.end()), 
                        vector<double>(poseY.begin(), poseY.end()), "r-");
        plt::plot({x[result.targetIdx]},{y[result.targetIdx]},"*g");
        plt::plot(body_x,body_y,"b-");
        plt::quiver(quiver_x,quiver_y,x_dir,y_dir,{{"color","r"}});
        plt::subplot(1,2,2);
        plt::plot(timeVel, targVel, "k:");
        plt::plot(timeVel, currVel, "r-");
        plt::plot(timeVel, currOmega, "y-");
        plt::legend();      // Enable legend
        plt::pause(0.03);   // Display plot continuously
    }
    return 1;
}