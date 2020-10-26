//
//  behaviorPlanning.h
//  
//
//  Created by Chandrasekar Sureshkumar on 5/3/18.
//

#ifndef behaviorPlanning_h
#define behaviorPlanning_h

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "stdio.h"
#include <math.h>
#include <algorithm>
#include "spline.h"
#include "FusionEKF.h"
#include <unordered_map>

using namespace std;
constexpr double pi() { return M_PI; }

using Eigen::MatrixXd;
using Eigen::VectorXd;
enum States { KL, PLCL, PLCR, LCL, LCR };

class PathPlanning {
public:
    double Ax_Lim = 9.5;
    double SpeedLimit = 49.5;
    double LaneWidth = 4.0;
    int totalLanes = 3;

    vector<double> maps_s;
    vector<double> maps_x;
    vector<double> maps_y;

    int closestWaypoint = 0;
    int lookAheadWP = 200;

    int nextWaypoint = 0;

    unordered_map<int,FusionEKF> obstacles;

     /**
     * Constructor
     */
    PathPlanning(vector<double> s_wp,vector<double> x_wp,vector<double> y_wp);

    /**
     * Destructor
     */
    virtual ~PathPlanning();

    double deg2rad(double x) { return x * pi() / 180; }
    double rad2deg(double x) { return x * 180 / pi(); }
    int getCurrentLane(double d) { return (int)(d/LaneWidth);}
    string getStateString(States state)
    {
        if (state == KL){return "KL";}
        else if (state == PLCL){return "PLCL";}
        else if (state == PLCR){return "PLCR";}
        else if (state == LCL){return "LCL";}
        else{return "LCR";}
    }
    double distance(double x1, double y1, double x2, double y2){return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));}
    double getSpeed(double vx, double vy){return sqrt((vx)*(vx)+(vy)*(vy));}
    int ClosestWaypoint(double x, double y)
    {
        double closestLen = 100000; //large number
        int closestWaypoint = 0;
        
        for(int i = 0; i < maps_x.size(); i++)
        {
            double map_x = maps_x[i];
            double map_y = maps_y[i];
            double dist = distance(x,y,map_x,map_y);
            if(dist < closestLen)
            {
                closestLen = dist;
                closestWaypoint = i;
            }
            
        }
        
        return closestWaypoint;
    }
    int NextWaypoint(double x, double y, double theta)
    {
        int closestWaypoint = ClosestWaypoint(x,y);
        
        double map_x = maps_x[closestWaypoint];
        double map_y = maps_y[closestWaypoint];
        
        double heading = atan2((map_y-y),(map_x-x));
        
        double angle = fabs(theta-heading);
        angle = min(2*pi() - angle, angle);
        
        if(angle > pi()/4)
        {
            // closestWaypoint++;
            // if (closestWaypoint == maps_x.size())
            // {
            //     closestWaypoint = 0;
            // }
            closestWaypoint = (closestWaypoint+1)%maps_s.size();
        }
        
        return closestWaypoint;
    }
    vector<double> getFrenet(double x, double y, double theta)
    {
        int next_wp = NextWaypoint(x,y, theta);
        
        int prev_wp;
        prev_wp = next_wp-1;
        if(next_wp == 0)
        {
            prev_wp  = maps_x.size()-1;
        }
        
        double n_x = maps_x[next_wp]-maps_x[prev_wp];
        double n_y = maps_y[next_wp]-maps_y[prev_wp];
        double x_x = x - maps_x[prev_wp];
        double x_y = y - maps_y[prev_wp];
        
        // find the projection of x onto n
        double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
        double proj_x = proj_norm*n_x;
        double proj_y = proj_norm*n_y;
        
        double frenet_d = distance(x_x,x_y,proj_x,proj_y);
        
        //see if d value is positive or negative by comparing it to a center point
        
        double center_x = 1000-maps_x[prev_wp];
        double center_y = 2000-maps_y[prev_wp];
        double centerToPos = distance(center_x,center_y,x_x,x_y);
        double centerToRef = distance(center_x,center_y,proj_x,proj_y);
        
        if(centerToPos <= centerToRef)
        {
            frenet_d *= -1;
        }
        
        // calculate s value
        double frenet_s = 0;
        for(int i = 0; i < prev_wp; i++)
        {
            frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
        }
        
        frenet_s += distance(0,0,proj_x,proj_y);
        
        return {frenet_s,frenet_d};
        
    }
    vector<double> getXY(double s, double d)
    {
        int prev_wp = -1;
        
        while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
        {
            prev_wp++;
        }
        
        int wp2 = (prev_wp+1)%maps_x.size();
        
        double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s-maps_s[prev_wp]);
        
        double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
        double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
        
        double perp_heading = heading-pi()/2;
        
        double x = seg_x + d*cos(perp_heading);
        double y = seg_y + d*sin(perp_heading);
        
        return {x,y};
        
    }
    
    double getSpeedDistanceCost(vector<vector<double>> predictions, vector<double> car_state, int deltaLane, double collision_safety_s);
    double getLaneChangeCost(vector<vector<double>> predictions, vector<double> car_state, int deltaLane);
    vector<vector<double>> getTrajectory(double ref_vel, vector<double> car_state_traj, int laneNum, vector<vector<double>> prev_path_XY);
    double getReferenceSpeed(States currentState, vector<vector<double>> predictions, vector<double> car_state, int currentLane, int finalLane, bool chkBehind, double ref_vel);
    vector<vector<double>> generate_trajectory(States state, vector<vector<vector<double>>> predictions, int finalLane, int nextLane, vector<double> car_state_traj, vector<vector<double>> prev_path_XY, double ref_vel, bool chkBehind);
    bool checkPLCStatus(vector<double> car_state, vector<vector<double>> predictions, double laneChangeCriteria, bool PLC);
    vector<bool> getLCStatus(int currentLane, vector<bool> plcStatus);
    vector<vector<vector<double>>> successor_states(vector<vector<vector<double>>> predictions, vector<double> car_state, States currentState, int nextLane, int prevLane, vector<vector<double>> prev_path_XY, double ref_vel);
    vector<double> generate_predictions(vector<double> car_state, int prev_path_size, double predictionHorizon);
    vector<double> generate_predictions_KF(vector<double> car_state, int prev_path_size, double predictionHorizon);
    vector<vector<double>> getClosestCars(vector<vector<double>> sensor_fusion, double car_s, vector<int> minMaxLanes, double range);
    vector<vector<double>> choose_next_state(vector<vector<double>> sensor_fusion, int &prevLane, int &nextLane, States &currentState, vector<double> car_state_current,vector<vector<double>> prev_path_XY, double ref_vel);
};

#endif /* behaviorPlanning_h */
