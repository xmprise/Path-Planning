#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <map>
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"
#include "include/Car_info.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::map;
using json = nlohmann::json;


constexpr double pi()    { return M_PI; }

string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1         = s.find_first_of("[");
    auto b2         = s.find_first_of("}");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1) * (x2-x1) + (y2-y1) * (y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
    double closestLen   = 100000.0;
    int closestWaypoint = 0;
    for (int i=0; i<maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist  = distance(x, y, map_x, map_y);
        if (dist < closestLen)
        {
            closestLen      = dist;
            closestWaypoint = i;
        }
    }
    return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
    double map_x        = maps_x[closestWaypoint];
    double map_y        = maps_y[closestWaypoint];
    double heading      = atan2((map_y-y), (map_x-x));
    double angle        = abs(theta-heading);
    if (angle > pi()/4)
    {
        closestWaypoint++;
    }
    return closestWaypoint;
}

vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
    int prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = maps_x.size() - 1;
    }
    double n_x         = maps_x[next_wp] - maps_x[prev_wp];
    double n_y         = maps_y[next_wp] - maps_y[prev_wp];
    double x_x         = x - maps_x[prev_wp];
    double x_y         = y - maps_y[prev_wp];
    double proj_norm   = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x      = proj_norm * n_x;
    double proj_y      = proj_norm * n_y;
    double frenet_d    = distance(x_x, x_y, proj_x, proj_y);
    double center_x    = 1000 - maps_x[prev_wp];
    double center_y    = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);
    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }
    double frenet_s = 0;
    for (int i=0; i<prev_wp; i++)
    {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
    }
    frenet_s += distance(0, 0, proj_x, proj_y);
    return {frenet_s, frenet_d};
}

// Frenet d-coordinate to the nearest enumerated lane
int convertDToLane(double d)
{
    if (d<4)
    {
        return 1;
    }
    if (d>=4.0 && d<8.0)
    {
        return 2;
    }
    if (d>=8.0)
    {
        return 3;
    }
    return 0;
}


int main() {
    uWS::Hub h;

    // Load waypoints
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    string line;
    ifstream in_map_(MAP_FILE, ifstream::in);
    while (getline(in_map_, line))
    {
        istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.emplace_back(x);
        map_waypoints_y.emplace_back(y);
        map_waypoints_s.emplace_back(s);
        map_waypoints_dx.emplace_back(d_x);
        map_waypoints_dy.emplace_back(d_y);
    }

    // Spline interpolate the map waypoints
    vector<double> waypoint_spline_t = {};
    for (int i=0; i<map_waypoints_x.size(); i++)
    {
        double t = (double)i / (double)map_waypoints_x.size();
        waypoint_spline_t.emplace_back(t);
    }

    tk::spline waypoint_spline_x;
    waypoint_spline_x.set_points(waypoint_spline_t, map_waypoints_x);
    tk::spline waypoint_spline_y;
    waypoint_spline_y.set_points(waypoint_spline_t, map_waypoints_y);
    tk::spline waypoint_spline_s;
    waypoint_spline_s.set_points(waypoint_spline_t, map_waypoints_s);
    tk::spline waypoint_spline_dx;
    waypoint_spline_dx.set_points(waypoint_spline_t, map_waypoints_dx);
    tk::spline waypoint_spline_dy;
    waypoint_spline_dy.set_points(waypoint_spline_t, map_waypoints_dy);

    vector<double> map_waypoints_x_new;
    vector<double> map_waypoints_y_new;
    vector<double> map_waypoints_s_new;
    vector<double> map_waypoints_dx_new;
    vector<double> map_waypoints_dy_new;

    map_waypoints_x.clear();
    map_waypoints_y.clear();
    map_waypoints_s.clear();
    map_waypoints_dx.clear();
    map_waypoints_dy.clear();

    for (int i=0; i<8000; i++)
    {
        double t = (double)i / (double)8000;
        map_waypoints_x.emplace_back(waypoint_spline_x(t));
        map_waypoints_y.emplace_back(waypoint_spline_y(t));
        map_waypoints_s.emplace_back(waypoint_spline_s(t));
        map_waypoints_dx.emplace_back(waypoint_spline_dx(t));
        map_waypoints_dy.emplace_back(waypoint_spline_dy(t));
    }
    Car_info cars(1);
    // Respond to simulator telemetry messages
    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                        &map_waypoints_dx, &map_waypoints_dy, &cars]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
                {
                    if (length && length > 2 && data[0] == '4' && data[1] == '2')
                    {
                        auto s = hasData(data);
                        if (s != "")
                        {
                            auto j       = json::parse(s);
                            string event = j[0].get<string>();
                            if (event == "telemetry")
                            {
                                double car_x         = j[1]["x"];
                                double car_y         = j[1]["y"];
                                double car_s         = j[1]["s"];
                                double car_d         = j[1]["d"];
                                double car_yaw       = j[1]["yaw"];
                                double car_speed     = j[1]["speed"];
                                auto previous_path_x = j[1]["previous_path_x"];
                                auto previous_path_y = j[1]["previous_path_y"];
                                double end_path_s    = j[1]["end_path_s"];
                                double end_path_d    = j[1]["end_path_d"];
                                auto sensor_fusion   = j[1]["sensor_fusion"];

                                json msgJson;
                                // First path
                                if (previous_path_x.size() == 0)
                                {
                                    cars.car_infor_clen();
                                    for (int i=0; i<sensor_fusion.size(); i++)
                                    {
                                        int id = sensor_fusion[i][0];
//                                        double x = sensor_fusion[i][1];
//                                        double y = sensor_fusion[i][2];
                                        double vx = sensor_fusion[i][3];
                                        double vy = sensor_fusion[i][4];
                                        double s = sensor_fusion[i][5];
                                        double d = sensor_fusion[i][6];
                                        int lane = convertDToLane(d);
                                        double speed = sqrt(vx*vx + vy*vy);

                                        cars.set_near_carInfo(lane, s);
                                    }
                                    int pos_l = convertDToLane(car_d);

                                    cars.set_telemetryInfo(pos_l, car_s, car_speed);
                                    cars.update_setpoint(map_waypoints_s, map_waypoints_x, map_waypoints_y);

                                    msgJson["next_x"]     = cars.jerkInfo.path_x;
                                    msgJson["next_y"]     = cars.jerkInfo.path_y;

                                }
                                    // Nearing the end of driven path
                                else if (previous_path_x.size() < 15)
                                {

                                    cars.car_infor_clen();
                                    for (int i=0; i<sensor_fusion.size(); i++)
                                    {
                                        int id = sensor_fusion[i][0];
//                                        double x = sensor_fusion[i][1];
//                                        double y = sensor_fusion[i][2];
                                        double vx = sensor_fusion[i][3];
                                        double vy = sensor_fusion[i][4];
                                        double s = sensor_fusion[i][5];
                                        double d = sensor_fusion[i][6];
                                        int lane = convertDToLane(d);
                                        double speed = sqrt(vx*vx + vy*vy);
                                        cars.set_near_carInfo(lane, s);
                                    }
                                    vector<double> path_x = previous_path_x;
                                    vector<double> path_y = previous_path_y;

                                    double car_s_ = cars.stateSave.last_s;
                                    double car_d_ = cars.stateSave.last_d;
                                    double car_speed_ = cars.stateSave.last_speed;
                                    int pos_l_ = convertDToLane(car_d_);
                                    cars.set_telemetryInfo(pos_l_, car_s_, car_speed_);
                                    cars.update_behavior(map_waypoints_s, map_waypoints_x, map_waypoints_y);

                                    cout << car_s_ << " " << car_d_ << " " << car_speed_ << " " << pos_l_ <<endl;
                                    for (int i=0; i<cars.jerkInfo.path_x.size(); i++)
                                    {
                                        path_x.push_back(cars.jerkInfo.path_x[i]);
                                        path_y.push_back(cars.jerkInfo.path_y[i]);
                                    }

                                    msgJson["next_x"] = path_x;
                                    msgJson["next_y"] = path_y;
                                }
                                else
                                {
                                    msgJson["next_x"] = previous_path_x;
                                    msgJson["next_y"] = previous_path_y;
                                }
                                auto msg = "42[\"control\","+ msgJson.dump()+"]";
                                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                            }
                        }
                        else
                        {
                            std::string msg = "42[\"manual\",{}]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }
                    }
                });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}