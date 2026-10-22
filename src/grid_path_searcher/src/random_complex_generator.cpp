#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <random>
#include <fstream>
#include <vector>

using namespace std;
using namespace Eigen;

ros::Publisher _all_map_pub;

int _obs_num, _cir_num;
double _x_size, _y_size, _z_size, _init_x, _init_y, _resolution, _sense_rate;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _w_c_l, _w_c_h;
std::string output_file_path;

bool _has_map = false;
bool randomMap;
bool randomSize;

std::vector<std::pair<double, double>> obstacles;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
vector<int> pointIdxSearch;
vector<float> pointSquaredDistance;

void writeObstaclesToCSV(const std::vector<std::pair<double, double>>& obstacles, const std::string& filename) {
    std::ofstream file;
    file.open(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing." << std::endl;
        return;
    }

    // Write headers
    file << "x,y" << std::endl;

    // Write data
    for (const auto& obstacle : obstacles) {
        file << obstacle.first << "," << obstacle.second << std::endl;
    }

    file.close();
}

std::vector<std::pair<double, double>> readObstaclesFromCSV(const std::string& filename) {
    std::vector<std::pair<double, double>> obstacles;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file for reading." << std::endl;
        return obstacles;
    }

    std::string line;
    std::getline(file, line);  // Skip header line
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string x_str, y_str;
        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            obstacles.emplace_back(x, y);
        }
    }

    file.close();
    return obstacles;
}

void RandomMapGenerate(const std::string& output_file_path) {
    random_device rd;
    default_random_engine eng(rd());

    uniform_real_distribution<double> rand_x = uniform_real_distribution<double>(_x_l, _x_h);
    uniform_real_distribution<double> rand_y = uniform_real_distribution<double>(_y_l, _y_h);
    uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(_w_l, _w_h);
    uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(_h_l, _h_h);

    // uniform_real_distribution<double> rand_x_circle = uniform_real_distribution<double>(_x_l + 1.0, _x_h - 1.0);
    // uniform_real_distribution<double> rand_y_circle = uniform_real_distribution<double>(_y_l + 1.0, _y_h - 1.0);
    // uniform_real_distribution<double> rand_r_circle = uniform_real_distribution<double>(_w_c_l    , _w_c_h    );

    // uniform_real_distribution<double> rand_roll      = uniform_real_distribution<double>(- M_PI,     + M_PI);
    // uniform_real_distribution<double> rand_pitch     = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
    // uniform_real_distribution<double> rand_yaw       = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
    // uniform_real_distribution<double> rand_ellipse_c = uniform_real_distribution<double>(0.5, 2.0);
    // uniform_real_distribution<double> rand_num       = uniform_real_distribution<double>(0.0, 1.0);

    pcl::PointXYZ pt_random;
    // Add distribution for obstacle size (in grid cells)
    uniform_int_distribution<int> rand_size = uniform_int_distribution<int>(1, 5);  // Adjust max size as needed

    // // firstly, we put some circles
    // for(int i = 0; i < _cir_num; i ++)
    // {
    //    double x0, y0, z0, R;
    //    std::vector<Vector3d> circle_set;

    //    x0   = rand_x_circle(eng);
    //    y0   = rand_y_circle(eng);
    //    z0   = rand_h(eng) / 2.0;
    //    R    = rand_r_circle(eng);

    //    if(sqrt( pow(x0-_init_x, 2) + pow(y0-_init_y, 2) ) < 2.0 )
    //       continue;

    //    double a, b;
    //    a = rand_ellipse_c(eng);
    //    b = rand_ellipse_c(eng);

    //    double x, y, z;
    //    Vector3d pt3, pt3_rot;
    //    for(double theta = -M_PI; theta < M_PI; theta += 0.025)
    //    {
    //       x = a * cos(theta) * R;
    //       y = b * sin(theta) * R;
    //       z = 0;
    //       pt3 << x, y, z;
    //       circle_set.push_back(pt3);
    //    }
    //    // Define a random 3d rotation matrix
    //    Matrix3d Rot;
    //    double roll,  pitch, yaw;
    //    double alpha, beta,  gama;
    //    roll  = rand_roll(eng); // alpha
    //    pitch = rand_pitch(eng); // beta
    //    yaw   = rand_yaw(eng); // gama

    //    alpha = roll;
    //    beta  = pitch;
    //    gama  = yaw;

    //    double p = rand_num(eng);
    //    if(p < 0.5)
    //    {
    //       beta = M_PI / 2.0;
    //       gama = M_PI / 2.0;
    //    }

    //    Rot << cos(alpha) * cos(gama)  - cos(beta) * sin(alpha) * sin(gama), - cos(beta) * cos(gama) * sin(alpha) - cos(alpha) * sin(gama),   sin(alpha) * sin(beta),
    //           cos(gama)  * sin(alpha) + cos(alpha) * cos(beta) * sin(gama),   cos(alpha) * cos(beta) * cos(gama) - sin(alpha) * sin(gama), - cos(alpha) * sin(beta),
    //           sin(beta)  * sin(gama),                                         cos(gama) * sin(beta),                                         cos(beta);

    //    for(auto pt: circle_set)
    //    {
    //       pt3_rot = Rot * pt;
    //       pt_random.x = pt3_rot(0) + x0 + 0.001;
    //       pt_random.y = pt3_rot(1) + y0 + 0.001;
    //       pt_random.z = pt3_rot(2) + z0 + 0.001;

    //       if(pt_random.z >= 0.0)
    //          cloudMap.points.push_back( pt_random );
    //    }
    // }

    bool is_kdtree_empty = false;
    if (cloudMap.points.size() > 0)
        kdtreeMap.setInputCloud(cloudMap.makeShared());
    else
        is_kdtree_empty = true;

    // Define the 8 neighboring directions
    const std::vector<std::pair<double, double>> neighbors = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
    // then, we put some pilar
    for (int i = 0; i < _obs_num; i++) {
        double x, y, w, h;
        int size;
        x = rand_x(eng);
        y = rand_y(eng);
        w = rand_w(eng);
        if (randomSize) {
            size = rand_size(eng);  // Generate random size for the obstacle
        } else {
            size = 2;
        }

        if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0)
            if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 0.8)
                continue;

        pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h) / 2.0);
        pointIdxSearch.clear();
        pointSquaredDistance.clear();

        if (is_kdtree_empty == false) {
            if (kdtreeMap.nearestKSearch(searchPoint, 1, pointIdxSearch, pointSquaredDistance) > 0) {
                if (sqrt(pointSquaredDistance[0]) < 1.0)
                    continue;
            }
        }

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;

        // // Generate points for a larger obstacle
        // for (int dx = -size / 2; dx <= size / 2; dx++) {
        //     for (int dy = -size / 2; dy <= size / 2; dy++) {
        //         double ox = x + dx * _resolution;
        //         double oy = y + dy * _resolution;
        //         obstacles.emplace_back(ox, oy);  // Store the x and y values

        //         h = rand_h(eng);
        //         int heiNum = 2.0 * ceil(h / _resolution);
        //         for (int t = 0; t < heiNum; t++) {
        //             pt_random.x = ox + 0.001;
        //             pt_random.y = oy + 0.001;
        //             pt_random.z = t * _resolution * 0.5 + 0.001;
        //             cloudMap.points.push_back(pt_random);
        //         }
        //     }
        // }

        // Generate points for a 4-grid obstacle
        for (int i = 0; i < 4; i++) {
            int dx, dy;
            // Define the 4 positions we want to occupy
            switch (i) {
            case 0:
                dx = 0;
                dy = 0;
                break;  // Center-right
            case 1:
                dx = 1;
                dy = 0;
                break;  // Right
            case 2:
                dx = 0;
                dy = 1;
                break;  // Top-center
            case 3:
                dx = 1;
                dy = 1;
                break;  // Top-right
            }

            double ox = x + dx * _resolution;
            double oy = y + dy * _resolution;
            obstacles.emplace_back(ox, oy);  // Store the x and y values

            h = rand_h(eng);
            int heiNum = 2.0 * ceil(h / _resolution);
            for (int t = 0; t < heiNum; t++) {
                pt_random.x = ox + 0.001;
                pt_random.y = oy + 0.001;
                pt_random.z = t * _resolution * 0.5 + 0.001;
                cloudMap.points.push_back(pt_random);
            }
        }
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    _has_map = true;

    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "map";

    writeObstaclesToCSV(obstacles, output_file_path);
}

void CsvMapGenerate(const std::string& input_file_path) {
    random_device rd;
    default_random_engine eng(rd());

    uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(_w_l, _w_h);
    uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(_h_l, _h_h);

    pcl::PointXYZ pt_random;

    // Read obstacles from CSV file
    obstacles = readObstaclesFromCSV(input_file_path);

    bool is_kdtree_empty = false;
    if (cloudMap.points.size() > 0)
        kdtreeMap.setInputCloud(cloudMap.makeShared());
    else
        is_kdtree_empty = true;

    for (std::vector<std::pair<double, double>>::size_type i = 0; i < obstacles.size(); i++) {
        double x, y, w, h;
        x = obstacles[i].first;
        y = obstacles[i].second;
        w = rand_w(eng);

        pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h) / 2.0);
        pointIdxSearch.clear();
        pointSquaredDistance.clear();

        if (is_kdtree_empty == false) {
            if (kdtreeMap.nearestKSearch(searchPoint, 1, pointIdxSearch, pointSquaredDistance) > 0) {
                if (sqrt(pointSquaredDistance[0]) < 1.0)
                    continue;
            }
        }

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil(w / _resolution);
        for (int r = -widNum / 2.0; r < widNum / 2.0; r++) {
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
                h = rand_h(eng);
                int heiNum = 2.0 * ceil(h / _resolution);
                for (int t = 0; t < heiNum; t++) {
                    pt_random.x = x + (r + 0.0) * _resolution + 0.001;
                    pt_random.y = y + (s + 0.0) * _resolution + 0.001;
                    pt_random.z = (t + 0.0) * _resolution * 0.5 + 0.001;
                    cloudMap.points.push_back(pt_random);
                }
            }
        }
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    _has_map = true;

    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "map";
}

void pubSensedPoints() {
    if (!_has_map)
        return;

    _all_map_pub.publish(globalMap_pcd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "random_complex_scene");
    ros::NodeHandle n("~");

    _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);

    n.param("init_state_x", _init_x, 0.0);
    n.param("init_state_y", _init_y, 0.0);

    n.param("map/x_size", _x_size, 50.0);
    n.param("map/y_size", _y_size, 50.0);
    n.param("map/z_size", _z_size, 5.0);

    n.param("map/obs_num", _obs_num, 30);
    n.param("map/circle_num", _cir_num, 30);
    n.param("map/resolution", _resolution, 0.2);

    n.param("ObstacleShape/lower_rad", _w_l, 0.3);
    n.param("ObstacleShape/upper_rad", _w_h, 0.8);
    n.param("ObstacleShape/lower_hei", _h_l, 3.0);
    n.param("ObstacleShape/upper_hei", _h_h, 7.0);

    n.param("CircleShape/lower_circle_rad", _w_c_l, 0.3);
    n.param("CircleShape/upper_circle_rad", _w_c_h, 0.8);
    n.param("output_file_path", output_file_path, std::string("obstacles.csv"));
    n.param("sensing/rate", _sense_rate, 1.0);
    n.param("randomGenerateMap", randomMap, true);
    n.param("randomObstacleSize", randomSize, true);

    _x_l = -_x_size / 2.0;
    _x_h = +_x_size / 2.0;

    _y_l = -_y_size / 2.0;
    _y_h = +_y_size / 2.0;

    if (randomMap)
        RandomMapGenerate(output_file_path);
    else
        CsvMapGenerate(output_file_path);

    ros::Rate loop_rate(_sense_rate);
    while (ros::ok()) {
        pubSensedPoints();
        ros::spinOnce();
        loop_rate.sleep();
    }
}