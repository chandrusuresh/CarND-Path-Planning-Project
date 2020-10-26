#include "behaviorPlanning.h"
// #include "Eigen/Dense"
#include <iostream>
#include <fstream>

// using namespace std;
// using Eigen::MatrixXd;
// using Eigen::VectorXd;
// using std::vector;

PathPlanning::PathPlanning(vector<double> s_wp,vector<double> x_wp,vector<double> y_wp)
{
    maps_s = s_wp;
    maps_x = x_wp;
    maps_y = y_wp;
    // SpeedLimit = maxSpeed;
}

PathPlanning::~PathPlanning() {}

    double PathPlanning::getSpeedDistanceCost(vector<vector<double>> predictions, vector<double> car_state, int deltaLane, double collision_safety_s)
    {
        // // // Speed and Distance Cost
        double nCarsInFront = 0;
        double speedDistCost = 0.0;
        double minDist = 1000000;
        double car_speed = distance(0,0,car_state[4],car_state[5]);
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
                        // speedDistCost += (SpeedLimit - check_speed)/fabs(distDelta); // 10mph buffer
                        speedDistCost += fabs(car_speed - min(SpeedLimit,check_speed))/fabs(distDelta); // 10mph buffer
                    }
                }
    //        }
        }
        //    cout << "Number of cars: " << nCarsInFront << " with deltaLane = " << deltaLane  << " and deltaDist = " << minDist << endl;
        speedDistCost = speedDistCost/std::max(nCarsInFront,1.0);
        return speedDistCost;
    }

    double PathPlanning::getLaneChangeCost(vector<vector<double>> predictions, vector<double> car_state, int deltaLane)
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

    vector<vector<double>> PathPlanning::getTrajectory(double ref_vel, vector<double> car_state_traj, int laneNum, vector<vector<double>> prev_path_XY)
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
        vector<double> wp1 = getXY(car_s+30,(2 + lane*4));
        vector<double> wp2 = getXY(car_s+60,(2 + lane*4));
        vector<double> wp3 = getXY(car_s+90,(2 + lane*4));
        
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

    double PathPlanning::getReferenceSpeed(States currentState, vector<vector<double>> predictions, vector<double> car_state, int currentLane, int finalLane, bool chkBehind, double ref_vel)
    {
        double v_ref = 0.0;
        double car_s = car_state[2];
        double car_spd = getSpeed(car_state[4],car_state[5]);
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

    vector<vector<double>> PathPlanning::generate_trajectory(States state, vector<vector<vector<double>>> predictions, int finalLane, int nextLane, vector<double> car_state_traj, vector<vector<double>> prev_path_XY, double ref_vel, bool chkBehind)
    {
        /*
        Given a possible next state, generate the appropriate trajectory to realize the next state.
        */
        int currentLane = getCurrentLane(car_state_traj[3]);
        double vel_ref_cL = getReferenceSpeed(state, predictions[currentLane],car_state_traj, currentLane, finalLane, chkBehind, ref_vel);
        double vel_ref_nL = getReferenceSpeed(state, predictions[currentLane],car_state_traj, currentLane, finalLane, chkBehind, ref_vel);
        double vel_ref = min(vel_ref_cL, vel_ref_nL);
        vector<double> misc = {vel_ref, (double)(state), (double)nextLane};
        vector<vector<double>> trajectory = getTrajectory(ref_vel+vel_ref, car_state_traj, finalLane, prev_path_XY);
        trajectory.push_back(misc);
        return trajectory;
    }

    bool PathPlanning::checkPLCStatus(vector<double> car_state, vector<vector<double>> predictions, double laneChangeCriteria, bool PLC)
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

    vector<bool> PathPlanning::getLCStatus(int currentLane, vector<bool> plcStatus)
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

    vector<vector<vector<double>>> PathPlanning::successor_states(vector<vector<vector<double>>> predictions, vector<double> car_state, States currentState, int nextLane, int prevLane, vector<vector<double>> prev_path_XY, double ref_vel)
    {
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
    //            vector<vector<double>> trajectory = generate_trajectory(currentState, predictions[nextLane], lane4Traj, nextLane, car_state, prev_path_XY, ref_vel, true);
                vector<vector<double>> trajectory = generate_trajectory(currentState, predictions, lane4Traj, nextLane, car_state, prev_path_XY, ref_vel, true);
                trajectories.push_back(trajectory);
            }
            else
            {
                // Keep to Lane
                cout << "Lane Change Complete!!! or Aborting Lane Change!!!" << endl;
    //            cout << "Current Lane is " << currentLane << prevLane+laneDelta << endl;
                states.push_back(KL);
                vector<vector<double>> trajectoryKL = generate_trajectory(KL, predictions, currentLane, currentLane, car_state, prev_path_XY, ref_vel, true);//false);
    //            vector<vector<double>> trajectoryKL = generate_trajectory(KL, predictions[currentLane], currentLane, currentLane, car_state, prev_path_XY, ref_vel, true);//false);
                trajectories.push_back(trajectoryKL);
            }
        }
        else if (currentState == PLCL || currentState == PLCR)
        {
            // Keep to Lane
            states.push_back(KL);
    //        vector<vector<double>> trajectory = generate_trajectory(KL, predictions[currentLane], currentLane, currentLane, car_state, prev_path_XY, ref_vel, true);
            vector<vector<double>> trajectory = generate_trajectory(KL, predictions, currentLane, currentLane, car_state, prev_path_XY, ref_vel, true);
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
    //        vector<vector<double>> trajectoryPLC = generate_trajectory(states[states.size()-1], predictions[nextLane], lane4Traj, nextLane, car_state, prev_path_XY, ref_vel, true);
            vector<vector<double>> trajectoryPLC = generate_trajectory(states[states.size()-1], predictions, lane4Traj, nextLane, car_state, prev_path_XY, ref_vel, true);
            trajectories.push_back(trajectoryPLC);
        }
        else
        {
            // Keep to Lane
            states.push_back(KL);
    //        vector<vector<double>> trajectoryKL = generate_trajectory(KL, predictions[currentLane], currentLane, currentLane, car_state, prev_path_XY, ref_vel, true);//false);
            vector<vector<double>> trajectoryKL = generate_trajectory(KL, predictions, currentLane, currentLane, car_state, prev_path_XY, ref_vel, true);//false);
            trajectories.push_back(trajectoryKL);
            
            if (currentLane > 0)
            {
                int minLane = 0;
                int maxLane = max(1,currentLane);
                for (int k=minLane; k < maxLane; k++)
                {
                    if (LCStatus[k])
                    {
                        states.push_back(PLCL);
    //                    vector<vector<double>> trajectory = generate_trajectory(PLCL, predictions[k], currentLane, k,  car_state, prev_path_XY, ref_vel, true);
                        vector<vector<double>> trajectory = generate_trajectory(PLCL, predictions, currentLane, k,  car_state, prev_path_XY, ref_vel, true);
                        trajectories.push_back(trajectory);
                    }
                    // else
                    // {
                    //     break;
                    // }
                }
            }
            if (currentLane < totalLanes - 1)
            {
                int minLane = max(0,currentLane+1);
                int maxLane = totalLanes;
                for (int k=minLane; k < maxLane; k++)
                {
                    if (LCStatus[k])
                    {
                        states.push_back(PLCR);
    //                    vector<vector<double>> trajectory = generate_trajectory(PLCR, predictions[k], currentLane, k, car_state, prev_path_XY, ref_vel, true);
                        vector<vector<double>> trajectory = generate_trajectory(PLCR, predictions, currentLane, k, car_state, prev_path_XY, ref_vel, true);
                        trajectories.push_back(trajectory);
                    }
                    // else
                    // {
                    //     break;
                    // }
                }
            }
        }
        for (int i = 0; i < states.size(); i++)
        {
            cout << "Possible transition: " << getStateString(currentState) << " -> " << getStateString(states[i]) << endl;
        }
        return trajectories;
    }

    vector<double> PathPlanning::generate_predictions(vector<double> car_state, int prev_path_size, double predictionHorizon)
    {
        /*
        Generates predictions for non-ego vehicles to be used
        in trajectory generation for the ego vehicle.
        */
        vector<double> predictions(6,-1);
        
        vector<double> currentXY = getXY(car_state[2], car_state[3]);
        double next_x = currentXY[0] + car_state[4]*predictionHorizon;
        double next_y = currentXY[1] + car_state[5]*predictionHorizon;
        double next_theta = atan2(next_y-currentXY[1],next_x-currentXY[0]);

        vector<double> nextSD = getFrenet(next_x, next_y, next_theta);

    //    //    vector<double> predictions;
    //    //    predictions = car_state;
    //    //    predictions[2] += (double)(prev_path_size)*0.02*sqrt(car_state[4]*car_state[4]+car_state[5]*car_state[5]);
        
        predictions[0] = car_state[0];
        predictions[1] = (double)(getCurrentLane(nextSD[1]));//car_state[1];//
        predictions[2] = nextSD[0];//car_state[2] + getSpeed(car_state[4],car_state[5])*predictionHorizon;//
        predictions[3] = nextSD[1];//car_state[3];//
        predictions[4] = car_state[4];
        predictions[5] = car_state[5];
        
        return predictions;
        
    }
    vector<double> PathPlanning::generate_predictions_KF(vector<double> car_state, int prev_path_size, double predictionHorizon)
    {
        /*
        Generates predictions for non-ego vehicles to be used
        in trajectory generation for the ego vehicle.
        */
       int car_id = (int) car_state[0];
       if (obstacles.count(car_id) == 0)
       {
           obstacles.insert({car_id,FusionEKF()});
       }
       vector<double> currentXY = getXY(car_state[2], car_state[3]);
       
       VectorXd meas = VectorXd(4);
       meas << currentXY[0], currentXY[1], car_state[4],car_state[5];

    //    cout << car_id << " : Processed" << endl;

       obstacles[car_id].ProcessMeasurement(meas);

        MatrixXd F_in = MatrixXd(4,4);
        F_in << 1, 0, predictionHorizon, 0,
                0, 1, 0, predictionHorizon,
                0, 0, 1, 0,
                0, 0, 0, 1;

        VectorXd x_pred = obstacles[car_id].ekf_.x_;
        
        int t = 0;
        while (t < 3)//(int)(predictionHorizon/obstacles[car_id].dt))
        {
            x_pred = F_in*x_pred;
            t++;
        }

       vector<double> predictions(6,-1);
    
       double next_x = x_pred[0];
       double next_y = x_pred[1];
       double next_speed  = getSpeed(x_pred[2],x_pred[3]);
       double next_theta = atan2(x_pred[3],x_pred[2]);

        vector<double> nextSD = getFrenet(next_x, next_y, next_theta);
        
        predictions[0] = car_state[0];
        predictions[1] = (double)(getCurrentLane(nextSD[1]));//car_state[1];//
        predictions[2] = nextSD[0];//car_state[2] + getSpeed(car_state[4],car_state[5])*predictionHorizon;//
        predictions[3] = nextSD[1];//car_state[3];//
        predictions[4] = x_pred[2];
        predictions[5] = x_pred[3];
        
        return predictions;
        
    }
    vector<vector<double>> PathPlanning::getClosestCars(vector<vector<double>> sensor_fusion, double car_s, vector<int> minMaxLanes, double range)
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

    vector<vector<double>> PathPlanning::choose_next_state(vector<vector<double>> sensor_fusion, int &prevLane, int &nextLane, States &currentState, vector<double> car_state_current,vector<vector<double>> prev_path_XY, double ref_vel)
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
        vector<double> car_state = generate_predictions(car_state_current, prev_path_XY[0].size(), predictionHorizon);
        
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
    
            if (chkBehind)
            {
                vector<double> pred = generate_predictions(closestCars[j], prev_path_XY[0].size(), predictionHorizon);
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
        
        vector<vector<vector<double>>> final_trajectories = successor_states(predictions, car_state_current, currentState, nextLane, prevLane, prev_path_XY,ref_vel);
        
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
                double spdDistCost = getSpeedDistanceCost(predictions[laneNum], car_state, deltaLane, collision_safety_s);
                double laneChangeCost = getLaneChangeCost(predictions[laneNum], car_state, deltaLane);
                double cost = spdDistCost*laneChangeCost;
                cout << getStateString(state) << " : " << laneNum  << " : " << spdDistCost << " * " << laneChangeCost << endl;
                costs.push_back(cost);
                final_states.push_back(state);
                final_lane.push_back(laneNum);
            }
        }
        cout << "Best Transition is: " << getStateString(currentState) << " -> ";
        vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
        int best_idx = std::distance(begin(costs), best_cost);
        
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