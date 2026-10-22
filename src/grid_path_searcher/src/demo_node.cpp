#include "demo_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "demo_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::string package_path = ros::package::getPath("grid_path_searcher");
  std::string debug_file = package_path + "/debug.txt";

  // Open and truncate the file to clear its contents
  std::ofstream outFile(debug_file, std::ofstream::trunc);
  if (!outFile.is_open()) {
    std::cerr << "Failed to open file:" << debug_file << std::endl;
    return 1;
  }

  // Save the original buffer of std::cout and redirect std::cout to outFile
  std::streambuf *originalCoutBuffer =
      std::cout.rdbuf();            // Save original buffer
  std::cout.rdbuf(outFile.rdbuf()); // Redirect cout to outFile

  demo_node.reset(new DemoNode());
  demo_node->init(nh, nh_priv);

  ros::spin();

  // Destroy demo_node (and its ros::Timer/Publisher/Subscriber members) now,
  // while roscpp is still tearing down, instead of at global static
  // destruction time after main() returns. Doing it late races with
  // roscpp's internal shutdown thread and aborts with a boost::lock_error
  // inside ros::Timer's destructor.
  demo_node.reset();

  // Restore original cout buffer when the node shuts down
  std::cout.rdbuf(originalCoutBuffer);
  outFile.close(); // Close the file

  return 0;
}

void DemoNode::init(ros::NodeHandle &nh, ros::NodeHandle &nh_priv) {
  config.loadParameters(nh_priv);

  // Start pose comes from the launch file (planning/start_x, planning/start_y)
  _start_pt << config.start_x, config.start_y;
  ROS_INFO("Start point set from launch file: (%.2f, %.2f)", _start_pt.x(),
           _start_pt.y());

  cout << "robot_inner_radius in demonode: " << config.robot_inner_radius
       << endl;

  _pts_sub =
      nh.subscribe("waypoints", 1, &DemoNode::rcvWaypointsCallback, this);

  planner_manager.reset(new PlannerManager());
  planner_manager->init(nh, config);

  ROS_INFO("Ready. Set a target with 2D Nav Goal in RViz.");
}

void DemoNode::rcvWaypointsCallback(const nav_msgs::Path &wp) {
  // Planning reads the occupancy grid and map kernel, which only exist once
  // the global map has been received and processed.
  if (!planner_manager->mapReady()) {
    ROS_WARN("Global map is not ready yet, ignoring this goal.");
    return;
  }

  _target_pt.x() = wp.poses[0].pose.position.x;
  _target_pt.y() = wp.poses[0].pose.position.y;

  cout << "Start point: " << _start_pt.transpose()
       << " Target point: " << _target_pt.transpose() << endl;

  planner_manager->getGoal(_target_pt);
  planner_manager->systemFSM(_start_pt, _target_pt);
}
