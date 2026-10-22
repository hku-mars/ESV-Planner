#ifndef _DEMO_NODE_H
#define _DEMO_NODE_H

#include <fstream>
#include <iostream>
#include <memory>

#include <ros/package.h>
#include <ros/ros.h>

#include <nav_msgs/Path.h>

#include "backward.hpp"
#include <fast_svsdf/planner_manager.hpp>

using namespace std;
using namespace Eigen;
namespace backward {
backward::SignalHandling sh;
}

class DemoNode {
public:
    DemoNode() {};   // Constructor
    ~DemoNode() {};  // Destructor

    typedef std::unique_ptr<DemoNode> Ptr;

    Config config;
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
    PlannerManager::Ptr planner_manager;

    // start pose comes from the launch file, target from the waypoint topic
    Eigen::Vector2d _start_pt = Vector2d(0, 0), _target_pt;

    ros::Subscriber _pts_sub;

    void rcvWaypointsCallback(const nav_msgs::Path& wp);
};
DemoNode::Ptr demo_node;
#endif
