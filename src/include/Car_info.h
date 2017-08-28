//
// Created by woojin.jung on 2017. 8. 26..
//
#include "State_info.h"
#include <vector>
#include <math.h>


#ifndef PATH_PLANNING_CAR_INFO_H
#define PATH_PLANNING_CAR_INFO_H

#define MAP_FILE                "../data/highway_map.csv"

#define PATH_PLAN_TRAVERSE_SECONDS       2.5
#define PATH_PLAN_TIME_INCREMENT     0.02

#define LANE_CHANGE_COST_SIDE_FRONT 1.0
#define LANE_CHANGE_COST_SIDE_BACK 0.5
#define LANE_CHANGE_COST_AHEAD  1.0

#define MAX_SPEED           20.5
#define MIN_SPEED           10.0

#define DISTANCE_MARGIN     4.0
#define DISTANCE_THRESH      20.0

#define MAX_COST                1000.0

#define MIN_TRACKING_CHANGE_FACTOR    -4.5
#define MAX_TRACKING_CHANGE_FACTOR     4.5

#define TRACK_DISTANCE             6945.554

class Car_info {

public:
    int id;
    sensing_info sensingInfo;
    telemetry_info telemetryInfo;
    waypoint_info waypointInfo;
    jerk_info jerkInfo;
    state_save stateSave;

    std::vector<sensing_info> near_car_data;

    Car_info(const int id);
    void car_infor_clen();
    void set_near_carInfo(int lane, double s);
    void set_telemetryInfo(int car_l, double car_s, double car_speed);
    void update_setpoint(std::vector<double> map_waypoints_s, std::vector<double>map_waypoints_x, std::vector<double>map_waypoints_y);
    void update_behavior(std::vector<double> map_waypoints_s, std::vector<double>map_waypoints_x, std::vector<double>map_waypoints_y);

};

#endif //PATH_PLANNING_CAR_INFO_H
