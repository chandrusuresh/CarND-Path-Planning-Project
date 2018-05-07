# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project, a path planning algorithm is developed to safely navigate a car around a virtual highway with other traffic. The Udacity Term3 Simulator is used for validation of the algorithm. The simulator provides the car's localization and sensor fusion data, along with a sparse map list of waypoints around the highway. The control aspect of the car is already taken care of by the simulator and is outside the scope of this project. The path planning algorithm allows the car to travel as fast as possible within the speed limit of 50 MPH with a max. acceleration of 10m/s^2 and max jerk of 10m/s^3 while still maintaining safe driving with other traffic. The path planning solution from this algorithm is sub-optimal, since safe and efficient driving and not optimality is the central goal of this exercise.  The algorithm computes the reference speed and appropriate lane for the car to travel to achieve safe and efficient driving by passing slower moving traffic when and where possible. The algorithm also allows the car to avoid collision with other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details of the simulator

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Implementation
Given that the highway to navigate has a fixed number of lanes (= 3), the following states are defined for the finite state machine.
*Keep Lane (KL) - This state essentially finds a trajectory that keeps the car within the current lane.
*Planning Lane Change Left/Right (PLCL/PLCR) - This state maintains the KL trajectory while adjusting the speed of the car to prepare the car for a lane change maneuver. The lane change is executed as soon as the safety and speed constraints are satisfied. Details of these conditions are described below.
*Lane Change Left/Right (LCL/LCR) - This state corresponds to when the car is executing the lane change maneuver. This is the only state where the car is allowed to cross between lanes.
*(Optional) Abort Lane Change- This state is critical for safety requirement, but is not an explicit part of this FSM. The safety and speed constraints mentioned above are constantly checked during the LCL/LCR states and when violated will change the state to KL. This state is specified here for the sake of completeness, but will not be discussed further. 

### Finite State Machine
The above states are connected as part of a finite state machine as shown below.
![alt text](https://github.com/chandrusuresh/CarND-Path-Planning-Project/blob/master/PathPlanning_FSM.png)

At initialization, the vehicle enters the Keep Lane (KL) state in lane#1 (middle lane) at zero speed. At each step, to update the speed, the speed of traffic within 100 meters of the current car location (in the Frenet co-ordinate system) is taken into account. A speed cost function is then computed for each lane that determines a metric that takes into account the difference of speed  from the speed limit and how close each car is to the ego car for each lane. This determines the cost for keeping lane or to plan a lane change maneuver. The details of the cost function are described in detail below.

To prevent constant switching between lanes, a lane change penalty of 0.5 if applied as a factor to the speed cost function value for each lane change. Suppose the speed cost value for a double lane change is c. The total cost of travelling in the new lane is therefore  c*(1+1) = 2c. If the speed cost is also c in the current lane, then the total cost for keeping in lane would be c*(1+0) = c. In this sitation, the best cost is in keeping with the current lane.

Once a lower cost for a different lane is detected, the algorithm then checks if it is safe to execute a lane change maneuver. This check is referred to as a planning lane change status (PLCstatus). This PLCstatus is checked to see if traffic in all the lanes from the current lane to the new lane are outside of a 20m radius of the ego car both ahead and behind. A second condition for speed delta is also checked to see if traffic ahead of the ego car in the new lane is too slow, in which case, the ego car may have to brake heavily to avoid collision or a car behind the ego car in the new lane is too fast, in which case, the ego car will have to accelerate beyond the acceleration/jerk threshold to avoid getting rear-ended. If these 2 conditions are satisfied then, the car is free to start the Lane Change manuerver. If any one of these conditions is not satisfied, then the car continues to remain in the planning lane change maneuver while adjusting its speed to the speed of traffic in the new lane to execute lane change when the above conditions are satisfied. The PLCstatus is checked at each time step the car is in the PLCL/PLCR states. While in PLCL/PLCR state, the car also checks for the total cost in the current lane, so that if the total cost of traffic in the current lane is better, the car can switch state to the KL state. The condition for the proximity and speed check that determine the  PLCstatus flag is explained below.  

If the PLCstatus is true, the car starts to execute the lane change maneuver. During the maneuver, the car continues to track and adjust its speed to the new lane. When the car gets into the new lane, it switches to the keep lane state in the new lane. To prevent collisions, the car continues to check for the proximity condition (as in the PLCstatus flag) to continue to monitor the safety of the lane change maneuver. The proximity condition during lane change is set to 10m. If at any time during lane change, the proximity condition is violated, the car switches back to the previous lane and changes its state to the keep lane state in the previous lane.

### Reference Speed Calculation
The goal of this project is to get the ego car to travel as close as possible to the speed limit without violating the acceleration and jerk conditions of 10m/s^2 and 10m/s^3 respectively. As a result, an acceleration limit of 9.5 m/s^2 is set as the acceleration limit. Since, the simulator runs at 50Hz, the maximum speed increase/decrease between time steps is 9.5*0.02 = 0.19 m/s.

For all the states (KL, PLCL/PLCR, LCL/LCR) the reference speed calculation is the same. The only difference is that for the KL state, the proximity of the closest car in the current lane is taken into account, while for the others, the closest car in the next lane is used.

Assuming the closest car in a given lane are travelling at a constant speed, the maximum acceleration/deceleration of the ego car is computed as the ratio of the difference in the squared speeds and twice the distance delta between the cars. speed delta between the 2 cars. i.e. suggested acc/dec = (Speed[j]^2 - egoCar_vel^2 /(2*(car_s[j] - egoCar_s)). The suggested speed increase/reduction of the ego car is the suggested acc/dec multiplied by the time step (0.02 sec).
 Since there is a hard constraint on the acceleration, the final speed delta from the current reference speed is the minimum of the suggested and the max speed (as discussed above).
 
 Another check is performed to make sure this final speed delta when applied does not violate the speed limit constraint. 

### Speed Cost Calculation
In each lane, a speed cost metric is computed for each vehicle within a 100m range as follows:
speed cost[i][j] = (Speed Limit - Speed[j])/(car_s[j] - egoCar_s)
where i refers to the lane and j refers to the traffic car in lane i. The numerator of the cost function gives an indication of the moving speed of traffic for that car, while the denominator gives an indication of the distance of the said traffic. The speed cost of the lane is computed as the sum of the speed costs of each car in that lane divided by the total number of cars in that lane. This gives an indication of the average traffic in each lane. This metric allows the car to be greedy and prefer instantaneous efficiency. For example, if the ego car is travelling at 15 m/s car and the car ahead in the current lane is travelling at 20 m/s at a distance of 50m. Another car in the next lane 100 m ahead is travelling at 20 m/s of the ego car. The speed cost of the traffic in the current lane is therefore (25-20)/50 = 0.1 while the speed cost in the next lane is (25-20)/100 = 0.05. The next lane is therefore more efficient for the ego car to switch lane to close the gap with the car in front. This gives the ego car the opportunity to change lane back to the current lane as soon as it passes the car that is currently ahead in the current lane.

### Lane Change Cost
This cost is applied as a factor the speed cost calculation. So, to keep lane the lane change cost is simply 1.0. For each lane change, the lane change cost increases by 50%. So for a double lane change maneuver, the lane change cost is a factor of (1+0.5*2) = 2.0. This additional cost prioritizes keeping to the current lane while also preventing zeno-type lane change behavior.

### Planning Lane Change Status (PLCstatus) Logic
To check if it is safe to execute the lane change maneuver, the PLCstatus logic is used. This logic condition is checked only in the PLCL/PLCR states. A modified version of this is also checked during the LCL/LCR states.

For a lane change to be safe, a 3 car length gap in all the subsequent (till the next) lanes is assumed to be necessary both in front and ahead of the vehicle. In addition, another logic is checked to ensure the ego car is not too fast/slow to keep up with the traffic in the next lane.

For this, the time taken to slow/speed the car from its current speed to the next lane traffic speed is calculated as ratio of the delta in distance and delta in speed between the ego car and the closest car in the next lane. This difference in time taken for the ego-car/traffic to catch up should be more than or equal to the minimum time taken to catch up. This minimum time taken is the ratio of the delta in speed and maximum acceleration/deceleration. If this speed check logic is violated, the ego car is deemed not ready for a lane change maneuver and continues to be in the PLCL/PLCR state.

This speed check logic is however not used during the LCL/LCR states, since it is assumed that that PLCL/PLCR state will bring the car to the necessary speed to satisfy this condition at the beginning of the maneuver. The proximity check is still in place to ensure no car comes dangerously close during the maneuver.

### Double Lane Change
Sometimes, the total cost in the next lane is higher than the keep lane cost, while the total cost 2 lanes away is less. In this case, a double lane change is required for efficient driving. Since the track is windy and there is a limit of 10m/s^2 on the total acceleration, the curvature of the track/highway adds a normal (centripetal) component to the total acceleration even if the speed remains the same. In such cases, to prevent violation of the acceleration limit, the double lane change is executed as two-single lane change maneuvers. This prevents the acceleration from exceeding the limit at speeds close to the speed limit. During this lane change manuever, once the first single lane change is complete, the car checks for the total cost once again, and the second single lane change is executed only if the total cost of the second lane is still the minimum of all lanes. Note that during the execution of this maneuver, the car switches states to PLCL/PLCR twice and checks the PLCstatus and proximity status for each sub-maneuver as explained above.


### Generating Trajectories
Once a lane to drive and reference speed are identified, the final task is to generate the trajectories. This is done with the same approach as done in the help video. the current untraversed part of the trajectory from the previous time step is used as the starting point and a spline is fitted to a point 30,60,90meters away in the next lane from the end of the previous trajectory. The construction of a spline ensures that the new trajectory passes through all these points and is smooth in the sense of the first and second derivatives.
Once a spline is fit, a trajectory is then interpolated from the current position to a point 30m away in the spline. Distance between each waypoint in this trajectory is determined by the reference speed and the time step of 0.02s. 

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
