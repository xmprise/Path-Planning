//
// Created by woojin.jung on 2017. 8. 26..
//
#include <vector>

#ifndef PATH_PLANNING_OTHERCAR_H
#define PATH_PLANNING_OTHERCAR_H

struct sensing_info
{
    int    car_l;
    double car_s;
};

struct state_save
{
    double last_s;
    double last_d;
    double last_speed;
};

// Telemetry type encapsulating the data we need for determining course
struct telemetry_info
{
    int    car_l;
    double car_s;
    double car_speed;
    std::vector<sensing_info> other_cars;
};

// Setpoint type for the controls we are returning
struct waypoint_info
{
    double start_pos_s;
    double start_vel_s;
    double end_pos_s;
    double end_vel_s;
    int    start_pos_l;
    int    end_pos_l;
};

// Type encapsulating x,y pairs and last s,d from jerk minimization
struct jerk_info
{
    std::vector<double> path_x;
    std::vector<double> path_y;
    double last_s;
    double last_d;
};

#endif //PATH_PLANNING_OTHERCAR_H
