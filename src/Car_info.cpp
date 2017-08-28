//
// Created by woojin.jung on 2017. 8. 26..
//

#include <iostream>
#include "include/Car_info.h"
#include "Eigen-3.3/Eigen/Dense"
#include <map>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


Car_info::Car_info(const int id)
{
    this->id = id;
}

void Car_info::car_infor_clen()
{
    near_car_data.clear();
}

void Car_info::set_near_carInfo(int lane, double s)
{
    this->sensingInfo.car_l = lane;
    this->sensingInfo.car_s = s;

    near_car_data.push_back(sensingInfo);
}

void determineStraightWaypoint(telemetry_info telemetry_data, waypoint_info *setpoint_data)
{
    double closest = 99999.0;
    for (int i=0; i<telemetry_data.other_cars.size(); i++)
    {
        if (telemetry_data.other_cars[i].car_l == telemetry_data.car_l)
        {
            double diff = telemetry_data.other_cars[i].car_s - telemetry_data.car_s;
            if (diff > 0.0 && diff < closest)
            {
                closest = diff;
            }
        }
    }

    double car_in_front_adj  = DISTANCE_MARGIN * (closest - DISTANCE_THRESH);
    if (car_in_front_adj > MAX_TRACKING_CHANGE_FACTOR)
    {
        car_in_front_adj = MAX_TRACKING_CHANGE_FACTOR;
    }
    if (car_in_front_adj < MIN_TRACKING_CHANGE_FACTOR)
    {
        car_in_front_adj = MIN_TRACKING_CHANGE_FACTOR;
    }
    double speed_start = telemetry_data.car_speed;
    if (speed_start > MAX_SPEED)
    {
        speed_start = MAX_SPEED;
    }
    double speed_end = speed_start + car_in_front_adj;
    if (speed_end > MAX_SPEED)
    {
        speed_end = MAX_SPEED;
    }
    if (speed_end < MIN_SPEED)
    {
        speed_end = MIN_SPEED;
    }

    setpoint_data->start_pos_s = telemetry_data.car_s;
    setpoint_data->start_vel_s = speed_start;
    setpoint_data->end_pos_s = telemetry_data.car_s + PATH_PLAN_TRAVERSE_SECONDS * 0.5 * (speed_start + speed_end);
    setpoint_data->end_vel_s = speed_end;
    setpoint_data->start_pos_l = telemetry_data.car_l;
    setpoint_data->end_pos_l = telemetry_data.car_l;
}

void Car_info::set_telemetryInfo(int car_l, double car_s, double car_speed)
{
   telemetryInfo.car_l = car_l;
   telemetryInfo.car_s = car_s;
   telemetryInfo.car_speed = car_speed;
   telemetryInfo.other_cars = near_car_data;

    determineStraightWaypoint(telemetryInfo, &waypointInfo);

}

// conversion of lane number to Frenet d-coordinate
double laneToFrenet(int lane)
{
    if (lane == 1)
    {
        return 2.0;
    }
    if (lane == 2)
    {
        return 6.0;
    }
    if (lane == 3)
    {
        return 10.0;
    }
    return 0;
}

std::vector<double> getXY(double s, double d, std::vector<double> maps_s, std::vector<double> maps_x,
                          std::vector<double> maps_y)
{
    int prev_wp = -1;
    while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1)))
    {
        prev_wp++;
    }
    int wp2             = (prev_wp+1) % maps_x.size();
    double heading      = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    double seg_s        = s - maps_s[prev_wp];
    double seg_x        = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y        = maps_y[prev_wp] + seg_s * sin(heading);
    double perp_heading = heading - M_PI / 2;
    double x            = seg_x + d * cos(perp_heading);
    double y            = seg_y + d * sin(perp_heading);
    return {x, y};
}

// Calculates jerk minimizing path
std::vector<double> computeMinimumJerk(std::vector<double> start, std::vector<double> end, double max_time, double time_inc)
{
    MatrixXd A = MatrixXd(3,3);
    VectorXd b = VectorXd(3);
    VectorXd x = VectorXd(3);

    double t  = max_time;
    double t2 = t * t;
    double t3 = t * t2;
    double t4 = t * t3;
    double t5 = t * t4;

    A <<   t3,    t4,    t5,
            3*t2,  4*t3,  5*t4,
            6*t,  12*t2, 20*t3;

    b << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
            end[1] - (start[1] + start[2] * t),
            end[2] - start[2];

    x = A.inverse() * b;

    double a0 = start[0];
    double a1 = start[1];
    double a2 = start[2] / 2.0;
    double a3 = x[0];
    double a4 = x[1];
    double a5 = x[2];

    std::vector<double> result;
    for (double t=time_inc; t<max_time+0.001; t+=time_inc)
    {
        double t2 = t * t;
        double t3 = t * t2;
        double t4 = t * t3;
        double t5 = t * t4;
        double r = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
        result.push_back(r);
    }
    return result;
}

void Car_info::update_setpoint(std::vector<double> map_waypoints_s, std::vector<double>map_waypoints_x, std::vector<double>map_waypoints_y)
{
    determineStraightWaypoint(telemetryInfo, &waypointInfo);

    // Conditions for minimum jerk in s (zero start/end acceleration)
    double start_pos_s = waypointInfo.start_pos_s;
    double start_vel_s = waypointInfo.start_vel_s;
    double end_pos_s = waypointInfo.end_pos_s;
    double end_vel_s = waypointInfo.end_vel_s;

    // Conditions for minimum jerk in d (zero start/end acceleration and velocity, indexing by lane)
    double start_pos_d = laneToFrenet(waypointInfo.start_pos_l);
    double end_pos_d = laneToFrenet(waypointInfo.end_pos_l);

    // Generate minimum jerk path in Frenet coordinates
    std::vector<double> next_s_vals = computeMinimumJerk({start_pos_s, start_vel_s, 0.0},
                                                         {end_pos_s, end_vel_s, 0.0},
                                                         PATH_PLAN_TRAVERSE_SECONDS,
                                                         PATH_PLAN_TIME_INCREMENT);
    std::vector<double> next_d_vals = computeMinimumJerk({start_pos_d, 0.0, 0.0},
                                                         {end_pos_d, 0.0, 0.0},
                                                         PATH_PLAN_TRAVERSE_SECONDS,
                                                         PATH_PLAN_TIME_INCREMENT);

    // Convert Frenet coordinates to map coordinates
    std::vector<double> next_x_vals = {};
    std::vector<double> next_y_vals = {};
    for (int i = 0; i < next_s_vals.size(); i++) {
        std::vector<double> xy = getXY(fmod(next_s_vals[i], TRACK_DISTANCE),
                                       next_d_vals[i],
                                       map_waypoints_s,
                                       map_waypoints_x,
                                       map_waypoints_y);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }

    jerkInfo.path_x = next_x_vals;
    jerkInfo.path_y = next_y_vals;
    jerkInfo.last_s = next_s_vals[next_s_vals.size() - 1];
    jerkInfo.last_d = next_d_vals[next_d_vals.size() - 1];

    stateSave.last_s = jerkInfo.last_s;
    stateSave.last_d = jerkInfo.last_d;
    stateSave.last_speed = waypointInfo.end_vel_s;
}

// Distance from the lane to the nearest car in front
double carFrontDistance(telemetry_info telemetry_data, int lane)
{
    double closest = 1000000.0;
    for (int i=0; i<telemetry_data.other_cars.size(); i++)
    {
        if (telemetry_data.other_cars[i].car_l == lane)
        {
            double diff = telemetry_data.other_cars[i].car_s - telemetry_data.car_s;
            if (diff > 0.0 && diff < closest)
            {
                closest = diff;
            }
        }
    }
    return closest;
}

// Distance from the lane to the nearest car in front in behind
double carBackDistance(telemetry_info telemetry_data, int lane)
{
    double closest = 99999.0;
    for (int i=0; i<telemetry_data.other_cars.size(); i++)
    {
        if (telemetry_data.other_cars[i].car_l == lane)
        {
            double diff = telemetry_data.car_s - telemetry_data.other_cars[i].car_s;
            if (diff > 0.0 && diff < closest)
            {
                closest = diff;
            }
        }
    }
    return closest;
}

// Cost left lane change
double leftLaneChangeCost(telemetry_info telemetry_data)
{
    if (telemetry_data.car_l == 1)
    {
        return MAX_COST;
    }
    double front_dist  = carFrontDistance(telemetry_data, telemetry_data.car_l - 1);
    double behind_dist = carBackDistance(telemetry_data, telemetry_data.car_l - 1);
    if (front_dist != 0.0 && behind_dist != 0.0)
    {
        return LANE_CHANGE_COST_SIDE_FRONT / front_dist + LANE_CHANGE_COST_SIDE_BACK / behind_dist;
    }
    return MAX_COST;
}

// Cost right lane change
double rightLaneChangeCost(telemetry_info telemetry_data)
{
    if (telemetry_data.car_l == 3)
    {
        return MAX_COST;
    }
    double front_dist  = carFrontDistance(telemetry_data, telemetry_data.car_l + 1);
    double behind_dist = carBackDistance(telemetry_data, telemetry_data.car_l + 1);
    if (front_dist != 0.0 && behind_dist != 0.0)
    {
        return LANE_CHANGE_COST_SIDE_FRONT / front_dist + LANE_CHANGE_COST_SIDE_BACK / behind_dist;
    }
    return MAX_COST;
}

// Cost keeping lane
double straightLaneCost(telemetry_info telemetry_data)
{
    double front_dist = carFrontDistance(telemetry_data, telemetry_data.car_l);
    if (front_dist != 0.0)
    {
        return LANE_CHANGE_COST_AHEAD / front_dist;
    }
    return MAX_COST;
}

// lowest cost action
string behaviorAction(telemetry_info telemetry_data)
{
    double left_cost  = leftLaneChangeCost(telemetry_data);
    double keep_cost  = straightLaneCost(telemetry_data);
    double right_cost = rightLaneChangeCost(telemetry_data);

    cout << "costs: " << left_cost << " - " << keep_cost << " - " << right_cost << endl;

    map<double, string> cost_map = { {left_cost,  "left"},
                                     {keep_cost,  "keep"},
                                     {right_cost, "right"} };

    // First value is the lowest cost since it is a priority queue on key
    map<double, string>::iterator cost_map_iterator;
    cost_map_iterator = cost_map.begin();
    string action = cost_map_iterator->second;
    return action;
}

void Car_info::update_behavior(std::vector<double> map_waypoints_s, std::vector<double>map_waypoints_x, std::vector<double>map_waypoints_y)
{
    string action = behaviorAction(telemetryInfo);
    if (action == "left")
    {
        waypointInfo.start_pos_s = telemetryInfo.car_s;
        waypointInfo.start_vel_s = telemetryInfo.car_speed;
        waypointInfo.end_pos_s = telemetryInfo.car_s + PATH_PLAN_TRAVERSE_SECONDS * telemetryInfo.car_speed;
        waypointInfo.end_vel_s = telemetryInfo.car_speed;
        waypointInfo.start_pos_l = telemetryInfo.car_l;
        waypointInfo.end_pos_l = telemetryInfo.car_l - 1;
    }
    else if (action == "keep")
    {
        determineStraightWaypoint(telemetryInfo, &waypointInfo);
    }
    else if (action == "right")
    {
        waypointInfo.start_pos_s = telemetryInfo.car_s;
        waypointInfo.start_vel_s = telemetryInfo.car_speed;
        waypointInfo.end_pos_s = telemetryInfo.car_s + PATH_PLAN_TRAVERSE_SECONDS * telemetryInfo.car_speed;
        waypointInfo.end_vel_s = telemetryInfo.car_speed;
        waypointInfo.start_pos_l = telemetryInfo.car_l;
        waypointInfo.end_pos_l = telemetryInfo.car_l + 1;
    }

    // Conditions for minimum jerk in s (zero start/end acceleration)
    double start_pos_s = waypointInfo.start_pos_s;
    double start_vel_s = waypointInfo.start_vel_s;
    double end_pos_s = waypointInfo.end_pos_s;
    double end_vel_s = waypointInfo.end_vel_s;

    // Conditions for minimum jerk in d (zero start/end acceleration and velocity, indexing by lane)
    double start_pos_d = laneToFrenet(waypointInfo.start_pos_l);
    double end_pos_d = laneToFrenet(waypointInfo.end_pos_l);

    // Generate minimum jerk path in Frenet coordinates
    std::vector<double> next_s_vals = computeMinimumJerk({start_pos_s, start_vel_s, 0.0},
                                                         {end_pos_s, end_vel_s, 0.0},
                                                         PATH_PLAN_TRAVERSE_SECONDS,
                                                         PATH_PLAN_TIME_INCREMENT);
    std::vector<double> next_d_vals = computeMinimumJerk({start_pos_d, 0.0, 0.0},
                                                         {end_pos_d, 0.0, 0.0},
                                                         PATH_PLAN_TRAVERSE_SECONDS,
                                                         PATH_PLAN_TIME_INCREMENT);

    // Convert Frenet coordinates to map coordinates
    std::vector<double> next_x_vals = {};
    std::vector<double> next_y_vals = {};
    for (int i = 0; i < next_s_vals.size(); i++) {
        std::vector<double> xy = getXY(fmod(next_s_vals[i], TRACK_DISTANCE),
                                       next_d_vals[i],
                                       map_waypoints_s,
                                       map_waypoints_x,
                                       map_waypoints_y);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }

    jerkInfo.path_x = next_x_vals;
    jerkInfo.path_y = next_y_vals;
    jerkInfo.last_s = next_s_vals[next_s_vals.size() - 1];
    jerkInfo.last_d = next_d_vals[next_d_vals.size() - 1];

    stateSave.last_s = jerkInfo.last_s;
    stateSave.last_d = jerkInfo.last_d;
    stateSave.last_speed = waypointInfo.end_vel_s;
}