/**
 * This file is part of Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
 * Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
 * for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_pkg/graph_node.h>
#include <list>
using std::vector;
using namespace std;

class PlanningVisualization {
private:
    enum TRAJECTORY_PLANNING_ID { GOAL = 1, PATH = 200, BSPLINE = 300, BSPLINE_CTRL_PT = 400, POLY_TRAJ = 500 };

    enum TOPOLOGICAL_PATH_PLANNING_ID { GRAPH_NODE = 1, GRAPH_EDGE = 100, RAW_PATH = 200, FILTERED_PATH = 300, SELECT_PATH = 400 };

    /* data */
    /* visib_pub is seperated from previous ones for different info */
    ros::NodeHandle node;
    ros::Publisher traj_pub_;      // 0
    ros::Publisher topo_pub_;      // 1, shortcut topo paths (Phase2)
    ros::Publisher predict_pub_;   // 2
    ros::Publisher visib_pub_;     // 3, visibility constraints
    ros::Publisher frontier_pub_;  // 4, frontier searching
    ros::Publisher yaw_pub_;       // 5, yaw trajectory
    ros::Publisher sample_pub_;    // 6, sample_point
    ros::Publisher topo_raw_pub_;   // 7, raw (pre-shortcut) topo paths (Phase1)
    ros::Publisher fast_check_pub_;      // 8, path(s) currently being fast-checked
    ros::Publisher fast_check_grid_pub_; // 9, discretized fast_check grid points, per-path namespace
    ros::Publisher sample_region_pub_;   // 10, topo sampling rectangle
    vector<ros::Publisher> pubs_;        //

    // Marker builders -- return a marker instead of publishing it, so a whole path set can be
    // packed into a single MarkerArray. An empty point list yields a DELETE marker.
    visualization_msgs::Marker makeLineListMarker(
        const vector<Eigen::Vector2d>& list1, const vector<Eigen::Vector2d>& list2, double line_width, const Eigen::Vector4d& color, int id, const std::string& ns);
    visualization_msgs::Marker makeSphereListMarker(const vector<Eigen::Vector2d>& list, double resolution, const Eigen::Vector4d& color, int id, const std::string& ns);
    // per-point colored CUBE_LIST: is_red[i] true -> red (occupied/high-risk), false -> green (free/low-risk)
    visualization_msgs::Marker makeGridCubeListMarker(
        const vector<Eigen::Vector3d>& points, const vector<bool>& is_red, double resolution, int id, const std::string& ns);
    void publishMarker(const visualization_msgs::Marker& mk, int pub_id);

    // append one marker per path (namespace "path_<i>", id base_id + i) plus DELETE markers for the
    // last_num - paths.size() namespaces that are no longer in use
    void fillPathMarkers(
        const vector<vector<Eigen::Vector2d>>& paths, double line_width, int base_id, int last_num, double alpha, visualization_msgs::MarkerArray& arr);

    int last_topo_path1_num_;
    int last_topo_path2_num_;
    int last_bspline_phase1_num_;
    int last_bspline_phase2_num_;
    int last_frontier_num_;
    int last_fast_check_num_;
    int last_fast_check_grid_num_;

public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(ros::NodeHandle& nh);

    // draw basic shapes list
    void displaySphereList(const vector<Eigen::Vector2d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id = 0);
    void displayCubeList(const vector<Eigen::Vector2d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id = 0);
    void displayLineList(
        const vector<Eigen::Vector2d>& list1, const vector<Eigen::Vector2d>& list2, double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0, const std::string& ns = "",
        double lifetime = 15);

    // draw basic shapes

    void displayCylinder(const Eigen::Vector2d& pos, double resolution, const Eigen::Vector4d& color, int id, int pub_id = 0);

    void displayLineStrip(const std::vector<Eigen::Vector2d>& points, double resolution, const Eigen::Vector4d& color, int id, int pub_id);

    void displayCoordinateList(const vector<Eigen::Vector2d>& coordinate_positions, int id, int pub_id);
    // draw a piece-wise straight line path

    // draw a polynomial trajectory
    //   void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color,
    //                           int id = 0);

    // draw a bspline trajectory
    //   void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
    //                    bool show_ctrl_pts = false, double size2 = 0.1,
    //                    const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
    //                    int id2 = 0);

    // draw a set of bspline trajectories generated in different phases
    //   void drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size);
    //   void drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size);

    void drawTopoGraph(std::list<GraphNode::Ptr>& graph, double point_size, double line_width, const Eigen::Vector4d& color1, const Eigen::Vector4d& color2, const Eigen::Vector4d& color3, int id = 0);

    void drawTopoPathsPhase1(vector<vector<Eigen::Vector2d>>& paths, double line_width);
    void drawTopoPathsPhase2(vector<vector<Eigen::Vector2d>>& paths, double line_width);
    void drawFastCheckPath(vector<vector<Eigen::Vector2d>>& paths, double line_width);
    // discretized fast_check points as small grid cubes, one namespace per path;
    // is_red[path][point] true -> red (occupied/high-risk), false -> green (free/low-risk)
    void drawFastCheckGrid(const vector<vector<Eigen::Vector3d>>& paths, const vector<vector<bool>>& is_red, double resolution);

    void drawGoal(Eigen::Vector2d goal, double resolution, const Eigen::Vector4d& color, int id = 0);
    // outline of the world axis aligned rectangle the topo roadmap samples in
    void drawSampleRegion(const Eigen::Vector2d& center, double half_x, double half_y, double line_width, const Eigen::Vector4d& color, int id = 0);
    //   void drawPrediction(ObjPrediction pred, double resolution, const Eigen::Vector4d& color, int id = 0);
    void drawsamplePoints0(vector<Eigen::Vector2d> points, double resolution, const Eigen::Vector4d& color, int id = 0);
    void drawsamplePoints1(vector<Eigen::Vector2d> points, double resolution, const Eigen::Vector4d& color, int id = 0);
    Eigen::Vector4d getColor(double h, double alpha = 1.0);
    void drawPath(const Eigen::Vector2d start_pt, const Eigen::Vector2d target_pt, double resolution, const Eigen::Vector4d& color, int id);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    // SECTION developing
    //   void drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt);
    //   void drawYawPath(NonUniformBspline& pos, const vector<double>& yaw, const double& dt);
};
// namespace fast_planner
#endif