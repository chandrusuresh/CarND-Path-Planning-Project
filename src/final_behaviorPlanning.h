//
//  final_behaviorPlanning.h
//  
//
//  Created by Chandrasekar Sureshkumar on 5/3/18.
//

#ifndef final_behaviorPlanning_h
#define final_behaviorPlanning_h

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "stdio.h"
#include <math.h>
#include <algorithm>
#include "spline.h"

using namespace std;
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double Ax_Lim = 9.5;
double SpeedLimit = 49;
enum States { KL, PLCL, PLCR, LCL, LCR };
double LaneWidth = 4.0;
int totalLanes = 3;

int getCurrentLane(double d)
{
    return (int)(d/LaneWidth);
}

string getStateString(States state)
{
    if (state == KL)
    {
        return "KL";
    }
    else if (state == PLCL)
    {
        return "PLCL";
    }
    else if (state == PLCR)
    {
        return "PLCR";
    }
    else if (state == LCL)
    {
        return "LCL";
    }
    else
    {
        return "LCR";
    }
}
double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
double getSpeed(double vx, double vy)
{
    return sqrt((vx)*(vx)+(vy)*(vy));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    
    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
    
    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];
    
    double heading = atan2((map_y-y),(map_x-x));
    
    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);
    
    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }
    
    return closestWaypoint;
}

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
    
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

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

double getSpeedDistanceCost(vector<vector<double>> predictions, vector<double> car_state, int deltaLane, double SpeedLimit, double collision_safety_s)
{
    // // // Speed and Distance Cost
    double nCarsInFront = 0;
    double speedDistCost = 0.0;
    double minDist = 1000000;
    for (int i=0; i < predictions.size(); i++)
    {
//        if (predictions[i][1] == car_state[1] + (double)deltaLane)
//        if (predictions[i][1] == getCurrentLane(car_state[3]) + (double)deltaLane)
//        {
            double distDelta = predictions[i][2] - car_state[2];
            bool cond = ((distDelta >= 0) && (deltaLane == 0)) || ((deltaLane != 0) && (fabs(distDelta) >= 0));
            
            //            if (distDelta > 0)
            if (cond)
            {
                if (distDelta < minDist)
                {
                    if (deltaLane != 0)
                    {
                        minDist = fabs(distDelta);
                    }
                    else
                    {
                        minDist = distDelta;
                    }
                    
                }
                double check_speed = sqrt(predictions[i][4]*predictions[i][4] + predictions[i][5]*predictions[i][5]);
                nCarsInFront += 1.0;
                if (distDelta == 0)
                {
                    speedDistCost += 100;
                }
                else
                {
                    speedDistCost += (SpeedLimit - check_speed)/fabs(distDelta); // 10mph buffer
                }
            }
//        }
    }
    //    cout << "Number of cars: " << nCarsInFront << " with deltaLane = " << deltaLane  << " and deltaDist = " << minDist << endl;
    speedDistCost = speedDistCost/std::max(nCarsInFront,1.0);
    return speedDistCost;
}

double getLaneChangeCost(vector<vector<double>> predictions, vector<double> car_state, int deltaLane, double SpeedLimit)
{
    // // // Lane Change Cost. Default to 1.0 since its a factor.
    double laneChangeCost = 1.0;
    
    if (deltaLane != 0)
    {
        int minLane = 0;
        int maxLane = 0;
        if (deltaLane < 0)
        {
            minLane = car_state[1]+deltaLane;
            maxLane = car_state[1]-1;
        }
        else
        {
            minLane = car_state[1] + 1;
            maxLane = car_state[1] + deltaLane;
        }
        
        vector<double> defVector = {-10000,10000};
        vector<vector<double>> maxGap(abs(deltaLane),defVector);
        for (int i=0; i < predictions.size(); i++)
        {
            if (predictions[i][1] >= (double)minLane && predictions[i][1] <= (double)maxLane)
            {
                int index = predictions[i][1] - minLane;
                if (predictions[i][2] - car_state[2] <= 0 && predictions[i][2] - car_state[2] > maxGap[index][0])
                {
                    maxGap[index][0] = predictions[i][2] - car_state[2];
                }
                else if (predictions[i][2] - car_state[2] > 0 && predictions[i][2] - car_state[2] < maxGap[index][1])
                {
                    maxGap[index][1] = predictions[i][2] - car_state[2];
                }
            }
        }
        for (int i=0; i < maxGap.size(); i++)
        {
            if (maxGap[i][0] <= -20 && maxGap[i][1] >= 20)
            {
                laneChangeCost += 0.5; // 0.05 // // // 5% cost per lane change
            }
            else
            {
                laneChangeCost += 1000.0; // // // Not enough space to change lane
            }
        }
    }
    return laneChangeCost;
}

vector<vector<double>> getTrajectory(double ref_vel, vector<double> car_state_traj, int laneNum, vector<vector<double>> prev_path_XY, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y)
{
    vector<vector<double>> trajectory;
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    
    double lane = (double)laneNum;//car_state_traj[1];
    double car_s = car_state_traj[2];
    double car_x = car_state_traj[6];
    double car_y = car_state_traj[7];
    double car_yaw = car_state_traj[8];
    
    vector<double> ptsx;
    vector<double> ptsy;
    
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);
    
    vector<double> previous_path_x = prev_path_XY[0];
    vector<double> previous_path_y = prev_path_XY[1];
    int prev_path_size = previous_path_x.size();
    
    if (prev_path_size < 2)
    {
        double prev_car_x = ref_x - cos(car_yaw);
        double prev_car_y = ref_y - sin(car_yaw);
        
        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);
        
        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }
    else
    {
        ptsx.push_back(previous_path_x[prev_path_size-2]);
        ptsx.push_back(previous_path_x[prev_path_size-1]);
        
        ptsy.push_back(previous_path_y[prev_path_size-2]);
        ptsy.push_back(previous_path_y[prev_path_size-1]);
        
        ref_x = ptsx[1];
        ref_y = ptsy[1];
        ref_yaw = atan2(ptsy[1]-ptsy[0],ptsx[1]-ptsx[0]);
    }
    // Get 3 points forward in time to create the spline
    vector<double> wp1 = getXY(car_s+30,(2 + lane*4),map_waypoints_s, map_waypoints_x,map_waypoints_y);
    vector<double> wp2 = getXY(car_s+60,(2 + lane*4),map_waypoints_s, map_waypoints_x,map_waypoints_y);
    vector<double> wp3 = getXY(car_s+90,(2 + lane*4),map_waypoints_s, map_waypoints_x,map_waypoints_y);
    
    ptsx.push_back(wp1[0]);
    ptsx.push_back(wp2[0]);
    ptsx.push_back(wp3[0]);
    
    ptsy.push_back(wp1[1]);
    ptsy.push_back(wp2[1]);
    ptsy.push_back(wp3[1]);
    
    ////// Shift to vehicle reference frame
    for (int i=0; i < ptsx.size(); i++)
    {
        double shift_x = ptsx[i]-ref_x;
        double shift_y = ptsy[i]-ref_y;
        
        ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
        ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
    }
    
    ////// Create a spline
    tk::spline s;
    s.set_points(ptsx,ptsy);
    
    for (int i=0; i < prev_path_size; i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
    
    ////// Break up spline into equidistant points
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);
    
    double x_add_on = 0.0;
    double N = target_dist/(0.02*ref_vel/2.24);
    for (int i=0; i < 50-prev_path_size; i++)
    {
        double x_ref = x_add_on + target_dist/N;
        double y_ref = s(x_ref);
        
        x_add_on = x_ref;
        
        next_x_vals.push_back(ref_x + x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
        next_y_vals.push_back(ref_y + x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
        
    }
    trajectory.push_back(next_x_vals);
    trajectory.push_back(next_y_vals);
    return trajectory;
}

double getReferenceSpeed(States currentState, vector<vector<double>> predictions, double car_s, int currentLane, int finalLane, bool chkBehind, double ref_vel)
{
    double v_ref = 0.0;
    double SpeedOfClosestCarInLane = 10000;
    int indexOfClosestCarinLane = -1;
    double collision_safety_s = 20;
    double DistanceToClosestCarinLane = 10000;
    int deltaLane = finalLane - currentLane;
    for (int i=0; i < predictions.size(); i++)
    {
        ////// Check if car is in the same lane
        float d = predictions[i][3];
        double vx = predictions[i][4];
        double vy = predictions[i][5];
        double check_speed = getSpeed(vx,vy);
        double check_car_s = predictions[i][2];
        
        double distDelta = predictions[i][2] - car_s;
        bool cond = ((distDelta >= 0) && (deltaLane == 0)) || ((deltaLane != 0) && (fabs(distDelta) >= 0));
        
        if (cond)
        {
            if (distDelta < DistanceToClosestCarinLane)
            {
                if (deltaLane != 0)
                {
                    DistanceToClosestCarinLane = fabs(distDelta);
                }
                else
                {
                    DistanceToClosestCarinLane = distDelta;
                }
                indexOfClosestCarinLane = i;
                SpeedOfClosestCarInLane = min(SpeedLimit,check_speed);
            }
        }
    }
    double range4Speed = 30;
    double delSpeed = 0;
    double delVMax = Ax_Lim*0.02;
    if (indexOfClosestCarinLane != -1 && fabs(DistanceToClosestCarinLane) < range4Speed)
    {
        ////// Car is ahead in the same lane and within 50 meters
        if (SpeedOfClosestCarInLane > ref_vel)
        {
            v_ref = min(delVMax,fabs(ref_vel-SpeedOfClosestCarInLane));
        }
        else if (SpeedOfClosestCarInLane < ref_vel)
        {
//            v_ref = -min(delVMax,fabs(ref_vel-SpeedOfClosestCarInLane));
            double delV = fabs(ref_vel-SpeedOfClosestCarInLane);
            double decel = ((ref_vel*ref_vel)-(SpeedOfClosestCarInLane*SpeedOfClosestCarInLane))/max(0.01,fabs(DistanceToClosestCarinLane));
            v_ref = -min(delVMax,fabs(decel)*0.02);
        }
        else
        {
            v_ref = 0.0;
        }
    }
    else if (indexOfClosestCarinLane == -1 || fabs(DistanceToClosestCarinLane) >= range4Speed)
    {
        v_ref = min(delVMax,SpeedLimit-ref_vel);
    }
        
    return v_ref;
}

vector<vector<double>> generate_trajectory(States state, vector<vector<vector<double>>> predictions, int finalLane, int nextLane, vector<double> car_state_traj, vector<vector<double>> prev_path_XY, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y, double ref_vel, bool chkBehind)
{
    /*
     Given a possible next state, generate the appropriate trajectory to realize the next state.
     */
    int currentLane = getCurrentLane(car_state_traj[3]);
    double vel_ref_cL = getReferenceSpeed(state, predictions[currentLane],car_state_traj[2], currentLane, finalLane, chkBehind, ref_vel);
    double vel_ref_nL = getReferenceSpeed(state, predictions[currentLane],car_state_traj[2], currentLane, finalLane, chkBehind, ref_vel);
    double vel_ref = min(vel_ref_cL, vel_ref_nL);
    vector<double> misc = {vel_ref, (double)(state), (double)nextLane};
    vector<vector<double>> trajectory = getTrajectory(ref_vel+vel_ref, car_state_traj, finalLane, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    trajectory.push_back(misc);
    return trajectory;
}

bool checkPLCStatus(vector<double> car_state, vector<vector<double>> predictions, double laneChangeCriteria, bool PLC)
{
    // This function determines if a planning lane change maneuver is successful.
    // Finds the closest car ahead and behind of the ego car in a given lane.
    
    bool plc_complete = true;
    
    double minDistAhead = 100000;
    int indexClosestAhead = -1;
    double minDistBehind = -100000;
    int indexClosestBehind = -1;
    for (int i=0; i < predictions.size(); i++)
    {
        double deltaDist = predictions[i][2] - car_state[2];
        if (deltaDist > 0 && deltaDist < minDistAhead)
        {
            minDistAhead = deltaDist;
            indexClosestAhead = i;
        }
        if (deltaDist <= 0 && deltaDist > minDistBehind)
        {
            minDistBehind = deltaDist;
            indexClosestBehind = i;
        }
    }
    
    if (indexClosestBehind == -1 && indexClosestAhead == -1)
    {
        // No cars ahead and behind;
        plc_complete = plc_complete & true;
//        cout << "PLC Status: " << plc_complete << "because No cars ahead and behind" << endl;
    }
    if (indexClosestAhead != -1)
    {
        // Car Ahead
        if (minDistAhead >= laneChangeCriteria)
        {
            double egoCar_v = getSpeed(car_state[4], car_state[5]);
            double car_v = getSpeed(predictions[indexClosestAhead][4], predictions[indexClosestAhead][5]);
            
            double delta_s = predictions[indexClosestAhead][2] - car_state[2];
            double delta_v = egoCar_v - car_v;
//            if (((predictions[indexClosestAhead][2] - car_state[2])/max(0.01,egoCar_v - car_v)) >= 0.5)
            if (delta_s/max(0.01,delta_v) > delta_v/Ax_Lim || !PLC)
            {
                plc_complete = true;
//                cout << "PLC Status: " << plc_complete << "because Car ahead satisfies" << endl;
            }
            else
            {
                cout << "delS = " << delta_s << " : delta_v = " << delta_v << endl;
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    if (indexClosestBehind != -1)
    {
        // Car Behind
        if (minDistBehind <= -laneChangeCriteria)
        {
            double egoCar_v = getSpeed(car_state[4], car_state[5]);
            double car_v = getSpeed(predictions[indexClosestBehind][4], predictions[indexClosestBehind][5]);
            
            double delta_s = car_state[2] - predictions[indexClosestBehind][2];
            double delta_v = car_v - egoCar_v;
            
//            if (((car_state[2] - predictions[indexClosestBehind][2])/max(0.01,(car_v - egoCar_v))) >= 0.5)
            if (delta_s/max(0.01,delta_v) > delta_v/Ax_Lim || !PLC)
            {
                plc_complete = true;
//                cout << "PLC Status: " << plc_complete << " because Car behind satisfies" << endl;
            }
            else
            {
                cout << "delS = " << delta_s << " : delta_v = " << delta_v << endl;
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    return plc_complete;
}

vector<bool> getLCStatus(int currentLane, vector<bool> plcStatus)
{
    int laneDelta = 0;
    vector<bool> LCStatus = plcStatus;
    // LCL
    for (int i= totalLanes-1; i > currentLane; i--)
    {
        for (int j=i-1; j > currentLane; j--)
        {
            LCStatus[i] = LCStatus[i] & plcStatus[j];
            if (!LCStatus[i])
            {
                break;
            }
        }
    }
    // LCR
    for (int i= 0; i < currentLane; i++)
    {
        for (int j=i+1; j < currentLane; j++)
        {
            LCStatus[i] = LCStatus[i] & plcStatus[j];
            if (!LCStatus[i])
            {
                break;
            }
        }
    }
    return LCStatus;
}

vector<vector<vector<double>>> successor_states(vector<vector<vector<double>>> predictions, vector<double> car_state, States currentState, int nextLane, int prevLane, vector<vector<double>> prev_path_XY, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y, double ref_vel) {
    /*
     Provides the possible next states given the current state for the FSM
     discussed in the course, with the exception that lane changes happen
     instantaneously, so LCL and LCR can only transition back to KL.
     */
    vector<States> states;
    int currentLane = getCurrentLane(car_state[3]);
    vector<vector<vector<double>>> trajectories;
    
    vector<bool> plcStatus(3,true);
    double laneChangeCriteria = 20;
    
    bool PLC = currentState == PLCL || currentState == PLCR;
    
    for (int i=0; i < totalLanes; i++)
    {
        if (i!= currentLane)
        {
            if (currentState == PLCL || currentState == PLCR)
            {
                laneChangeCriteria = 15;
            }
            if (currentState == LCL || currentState == LCR)
            {
                laneChangeCriteria = 10;
            }
            plcStatus[i] = checkPLCStatus(car_state, predictions[i], laneChangeCriteria, PLC);
//            cout << "PLC Complete status = " << plcStatus[i] << " in lane: " << i << endl;
        }
    }
    
    vector<bool> LCStatus = getLCStatus(currentLane, plcStatus);
    for (int i=0; i < LCStatus.size(); i++)
    {
        cout << "LC Complete status = " << LCStatus[i] << " in lane: " << i << endl;
    }
    
    if (currentState == LCL || currentState == LCR)
    {
        if (LCStatus[nextLane] && (nextLane != currentLane))
        {
            // Continue Lane Change Maneuver
            states.push_back(currentState);
            // Lane change in progress
            int lane4Traj = nextLane;
            if (abs(nextLane - currentLane) > 1)
            {
                lane4Traj = currentLane+1;
                if (currentState == LCL)
                {
                    lane4Traj = currentLane - 1;
                }
            }
//            vector<vector<double>> trajectory = generate_trajectory(currentState, predictions[nextLane], lane4Traj, nextLane, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);
            vector<vector<double>> trajectory = generate_trajectory(currentState, predictions, lane4Traj, nextLane, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);
            trajectories.push_back(trajectory);
        }
        else
        {
            // Keep to Lane
            cout << "Lane Change Complete!!! or Aborting Lane Change!!!" << endl;
//            cout << "Current Lane is " << currentLane << prevLane+laneDelta << endl;
            states.push_back(KL);
            vector<vector<double>> trajectoryKL = generate_trajectory(KL, predictions, currentLane, currentLane, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);//false);
//            vector<vector<double>> trajectoryKL = generate_trajectory(KL, predictions[currentLane], currentLane, currentLane, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);//false);
            trajectories.push_back(trajectoryKL);
        }
    }
    else if (currentState == PLCL || currentState == PLCR)
    {
        // Keep to Lane
        states.push_back(KL);
//        vector<vector<double>> trajectory = generate_trajectory(KL, predictions[currentLane], currentLane, currentLane, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);
        vector<vector<double>> trajectory = generate_trajectory(KL, predictions, currentLane, currentLane, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);
        trajectories.push_back(trajectory);
        int lane4Traj = nextLane;
        if (LCStatus[nextLane])
        {
            if (currentState == PLCL)
            {
                states.push_back(LCL);
            }
            else
            {
                states.push_back(LCR);
            }
        }
        else
        {
            states.push_back(currentState);
            lane4Traj = currentLane;
        }
//        vector<vector<double>> trajectoryPLC = generate_trajectory(states[states.size()-1], predictions[nextLane], lane4Traj, nextLane, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);
        vector<vector<double>> trajectoryPLC = generate_trajectory(states[states.size()-1], predictions, lane4Traj, nextLane, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);
        trajectories.push_back(trajectoryPLC);
    }
    else
    {
        // Keep to Lane
        states.push_back(KL);
//        vector<vector<double>> trajectoryKL = generate_trajectory(KL, predictions[currentLane], currentLane, currentLane, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);//false);
        vector<vector<double>> trajectoryKL = generate_trajectory(KL, predictions, currentLane, currentLane, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);//false);
        trajectories.push_back(trajectoryKL);
        
        if (currentLane > 0)
        {
            int minLane = 0;
            int maxLane = max(1,currentLane);
            for (int k=minLane; k < maxLane; k++)
            {
                if (true)//LCStatus[k])
                {
                    states.push_back(PLCL);
//                    vector<vector<double>> trajectory = generate_trajectory(PLCL, predictions[k], currentLane, k,  car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);
                    vector<vector<double>> trajectory = generate_trajectory(PLCL, predictions, currentLane, k,  car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);
                    trajectories.push_back(trajectory);
                }
                else
                {
                    break;
                }
            }
        }
        if (currentLane < totalLanes - 1)
        {
            int minLane = max(0,currentLane+1);
            int maxLane = totalLanes;
            for (int k=minLane; k < maxLane; k++)
            {
                if (true)//LCStatus[k])
                {
                    states.push_back(PLCR);
//                    vector<vector<double>> trajectory = generate_trajectory(PLCR, predictions[k], currentLane, k, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);
                    vector<vector<double>> trajectory = generate_trajectory(PLCR, predictions, currentLane, k, car_state, prev_path_XY, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, true);
                    trajectories.push_back(trajectory);
                }
                else
                {
                    break;
                }
            }
        }
    }
    for (int i = 0; i < states.size(); i++)
    {
        cout << "Possible transition: " << getStateString(currentState) << " -> " << getStateString(states[i]) << endl;
    }
    return trajectories;
}

vector<double> generate_predictions(vector<double> car_state, int prev_path_size, double predictionHorizon, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y)
{
    /*
     Generates predictions for non-ego vehicles to be used
     in trajectory generation for the ego vehicle.
     */
    vector<double> predictions(6,-1);
    
    vector<double> currentXY = getXY(car_state[2], car_state[3], map_waypoints_s, map_waypoints_x, map_waypoints_y);
    double next_x = currentXY[0] + car_state[4]*predictionHorizon;
    double next_y = currentXY[1] + car_state[5]*predictionHorizon;
    double next_theta = atan2(next_y-currentXY[1],next_x-currentXY[0]);

    vector<double> nextSD = getFrenet(next_x, next_y, next_theta, map_waypoints_x, map_waypoints_y);

//    //    vector<double> predictions;
//    //    predictions = car_state;
//    //    predictions[2] += (double)(prev_path_size)*0.02*sqrt(car_state[4]*car_state[4]+car_state[5]*car_state[5]);
    
    predictions[0] = car_state[0];
    predictions[1] = (double)(getCurrentLane(nextSD[1]));//car_state[1];//
    predictions[2] = nextSD[0];//car_state[2] + getSpeed(car_state[4],car_state[5])*predictionHorizon;//
    predictions[3] = nextSD[1];//car_state[3];//
    predictions[4] = car_state[4];
    predictions[5] = car_state[5];
    
//    cout << "Before update: " << currentXY[0] << " : " << currentXY[1] << endl;//" : " << car_state[3] << endl;
//    cout << "After update: " << next_x << " : " << next_y << endl;//" : " << predictions[3] << endl;
//    cout << "Prediction Horizon: " << car_state[4] << " * " << predictionHorizon << " = " << car_state[4]*predictionHorizon << endl;
    
    return predictions;
    
}

vector<vector<double>> getClosestCars(vector<vector<double>> sensor_fusion, double car_s, vector<int> minMaxLanes, double range)
{
    // // // This function gets the closest (defined by the input argument range) cars to the ego car in each lane.
    // // // output format is a vector of states containing (double)car_id, car_lane, car_s, car_d, car_speed_x, car_speed_y for each closest car within the range in each lane.
    //double range = 70; // 70 meter range
    
    vector<vector<double>> closestCars;
    
    int count = 0;
    vector<int> countLanes(minMaxLanes[1]-minMaxLanes[0]+1,0);
    
    for (int i=0; i < sensor_fusion.size(); i++)
    {
        double check_car_s = sensor_fusion[i][5];
        if (fabs(check_car_s - car_s) <= range)
        {
            double d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = getSpeed(vx,vy);//sqrt(vx*vx + vy*vy);
            
            vector<double> state(6,-1);
            state[0] = (double)i;
            state[1] = getCurrentLane(d);
            state[2] = check_car_s;
            state[3] = d;
            state[4] = vx;
            state[5] = vy;
            
            closestCars.push_back(state);
        }
    }
    return closestCars;
}

vector<vector<double>> choose_next_state(vector<vector<double>> sensor_fusion, int &prevLane, int &nextLane, States &currentState, vector<double> car_state_current, double SpeedLimit, const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,vector<vector<double>> prev_path_XY, double ref_vel)
{
    /*
     Here you can implement the transition_function code from the Behavior Planning Pseudocode
     classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
     to the next state.
     
     INPUT: A predictions map. This is a map of vehicle id keys with predicted
     vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
     the vehicle at the current timestep and one timestep in the future.
     OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.
     
     */
    double collision_safety_s = 20;
    bool chkBehind = false;
    int laneNum_L = 0;//max(0, (int)car_state[1]-1); // Lane to the left  of current lane
    int laneNum_R = totalLanes + laneNum_L - 1;//min(2, (int)car_state[1]+1); // Lane to the right of current lane
    vector<int> minMaxLanes = {laneNum_L, laneNum_R};
    double range = 100;
    double predictionHorizon = 0.1;
    
    int currentLane = getCurrentLane(car_state_current[3]);
    
    // Get prediction for ego car
    vector<double> car_state = generate_predictions(car_state_current, prev_path_XY[0].size(), predictionHorizon, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    
    // Get other cars in all lanes that are within range of ego car. Rest of them are ignored.
    vector<vector<double>> closestCars = getClosestCars(sensor_fusion, car_state_current[2], minMaxLanes, range);
    
    vector<vector<vector<double>>> predictions;
    
    // Initialize predictions. Predictions is a vector of a vector of state vectors. It contains a list of vehicle states of cars in each lane.
    vector<double> defValues(6,-1);
    vector<vector<double>> defValuesVector;
    defValuesVector.push_back(defValues);

    vector<int> carsCount(totalLanes,0);
    for (int i=0; i < totalLanes; i++)
    {
        predictions.push_back(defValuesVector);
    }
    
    for (int j=0; j < closestCars.size(); j++)
    {
        bool chkBehind = true;
//        if (closestCars[j][1] == currentLane && (closestCars[j][2] - car_state_current[2]) < 0)
//        {
//            chkBehind = false;
//        }
        if (chkBehind)
        {
            vector<double> pred = generate_predictions(closestCars[j], prev_path_XY[0].size(), predictionHorizon, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            int cCarLane = getCurrentLane(pred[3]);
            if (cCarLane >= 0)
            {
                if (carsCount[cCarLane] > 0)
                {
                    predictions[cCarLane].push_back(pred);
                }
                else
                {
                    predictions[cCarLane][0] = pred;
                }
                carsCount[cCarLane]++;
            }
        }
    }
    
//    for (int i=0; i < totalLanes; i++)
//    {
//        cout << "Total cars after prediction in lane " << i << " is: " << carsCount[i] << endl;
//        if (i == currentLane && carsCount[i] > 0)
//        {
//            for (int j=0; j < carsCount[i]; j++)
//            {
//                cout << "Dist of car in lane#: " << currentLane << " is: " << predictions[i][j][2] - car_state_current[2] << endl;
//                cout << "D of closest car in lane is: " << predictions[i][j][3] << endl;
//                if (fabs(predictions[i][j][2] - car_state_current[2]) <= 7)
//                {
//                    cout << "**********************************************" << endl;
//                    cout << "**********************************************" << endl;
//                    cout << "*********Something WRONG *********" << endl;
//                    cout << "**********************************************" << endl;
//                    cout << "**********************************************" << endl;
//                }
//            }
//        }
//    }
//    cout << "Current s = " << car_state[2] << endl;
//    for (int i=0; i < totalLanes; i++)
//    {
//        for (int j=0; j < predictions[i].size(); j++)
//        {
//            cout << predictions[i][j][1] << " : " << predictions[i][j][2]  << " : " << predictions[i][j][3] << endl;
//        }
//    }
    
    vector<vector<vector<double>>> final_trajectories = successor_states(predictions, car_state_current, currentState, nextLane, prevLane, prev_path_XY,map_waypoints_s, map_waypoints_x, map_waypoints_y,ref_vel);
    
    float cost;
    vector<float> costs;
    vector<States> final_states;
    vector<int> final_lane;
    double laneChangeCriteria = 20;
    for (int i=0; i < final_trajectories.size();i++)
    {
        vector<vector<double>> trajectory = final_trajectories[i];
        if (trajectory[0].size() != 0)
        {
            int laneNum = (int)(trajectory[2][2]);
            States state = (States)(trajectory[2][1]);
            int deltaLane = laneNum - currentLane;
            double spdDistCost = getSpeedDistanceCost(predictions[laneNum], car_state, deltaLane, SpeedLimit, collision_safety_s);
            double laneChangeCost = getLaneChangeCost(predictions[laneNum], car_state, deltaLane, SpeedLimit);
            double cost = spdDistCost*laneChangeCost;
            cout << getStateString(state) << " : " << laneNum  << " : " << spdDistCost << " * " << laneChangeCost << endl;
            costs.push_back(cost);
            final_states.push_back(state);
            final_lane.push_back(laneNum);
        }
    }
    cout << "Best Transition is: " << getStateString(currentState) << " -> ";
    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    
    if (abs(final_lane[best_idx] - nextLane) > 1)
    {
        cout << "**********************************************" << endl;
        cout << "**********************************************" << endl;
        cout << "*********Double Lane Change Detected*********" << endl;
        cout << "**********************************************" << endl;
        cout << "**********************************************" << endl;
    }
    if (nextLane != final_lane[best_idx])
//    if (final_states[best_idx] == KL)
    {
        prevLane = currentLane;
    }
    
    currentState = final_states[best_idx];
    nextLane = final_lane[best_idx];
    
    cout << getStateString(currentState) << " and new lane is: " << nextLane << " while current lane is: " << currentLane << " and prev lane is " << prevLane << endl;
    cout << "Delta Speed: " << final_trajectories[best_idx][2][0] << endl;
    //    cout << "Best idx is: " << best_idx << " and best_idx1 is: " << best_idx1 << endl;
    cout << "*********************************" << endl;
    return final_trajectories[best_idx];
}

#endif /* final_behaviorPlanning_h */
