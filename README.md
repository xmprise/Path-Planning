# Path Planning Project
Udacity Self-driving car term3 path planning project.

### Project Overview
To solve the problem of running smoothly on a track provided by a simulation, the following three must be solved.

- Smoothing given waypoint data
- Minimum jerk trajectory generation
- Create a model that minimizes costs and determine behavior.

### Smoothing exist waypoint data
The track is 6.9 km, but 181 waypoints are given. Linear interpolation of waypoints to calculate map coordinates in Frenet coordinates and apply them to the simulator. Simply restricting the d-coordinate of the Frenet and increasing the s-coordinate sequentially results in excessive acceleration and jerk. To solve this problem, interpolating the waypoints allows smooth estimation of the entire frenet coordinates. I have interpolated 8000 waypoints, but every 1 meter would have achieved the given goal.

### Applied to simulator
The simulator operates asynchronously with the path planning code, and as the calculated path segment transitions to the next path segment, the running speed temporarily changes due to the mismatch of the waypoints.
To solve this problem, save the last waypoint sent from the code and the waypoints observed in the simulator. For example, the last coordinate of the first segment is used as the first point of the next segment, and the last segment is used to send the last coordinate previously saved to the simulator.
The effect of this is to keep the previous path in the buffer and keep the point where the proper path is made.

### Compute Minimum Jerk
To create a smooth path, you must have a certain time interval. This is a long enough time to change the lane and to make time to calculate the cost function. The following assumes that the acceleration at the beginning and end of the minimum jerk frame is zero. This is helpful when switching from t frame to t + 1 frame. Acceleration and jerk are all associated with the constraint that velocity must match both at the beginning and end of the frame.
Use the standard jerk model to minimize the 5th order polynomial model as described in the lecture.

### Determine behavior
Given that the speed is relatively low and the gaps are relatively large, there is no need to use too many states to complicate the problem. I implemented the following behavior.

- keep going.
- Left lane change.
- Right lane change.

In the case of a lane change, the lane must not make the most left or right change action and should be acceptable in relation to the other vehicle. If both use the least cost and the lane change is not appropriate, the cost will be set high. The location of the nearest car in front of and behind the car is used to calculate the cost.

If go straight ahead, only need to consider the same lane. This is because other cars are quite good at beating in the back.
If going straight, one more thing should be considered. should not hit a car that is ahead of us to maximize speed with speed limitations. may not be able to change lanes, but the car will slow down and you will need to monitor car ahead of time and adjust the speed. The way I choose to achieve this is to modify the planned path final speed in proportion to the distance measurement from the nearest car ahead of me.

Once each cost is calculated and the lowest cost is determined, the action you can take is sent to the least jerk trajectory code.


### Conclusion
The resulting path planner works fine, but it is not perfect. If all other lanes are running at the same speed in all possible lanes, there is a chance that the planner will fall into a deadlock that can not be determined. However, this is a very rare phenomenon that can not occur in real world.

---
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
