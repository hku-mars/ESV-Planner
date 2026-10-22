#ifndef PLANNER_MANAGER_HPP
#define PLANNER_MANAGER_HPP

#include <fast_svsdf/planner_strategy.hpp>
#include <planner_algorithm/back_end_optimizer.hpp>
#include <planner_algorithm/mid_end.hpp>
#include <traj_utils/PolyTraj.h>
#include <utils/config.hpp>
using PathPoint = PlannerStrategy::PathPoint;

class PlannerManager {
private:
  // back end

  OriTraj::Ptr ori_traj_generator;
  TrajOptimizer::Ptr minco_traj_optimizer;

  ros::Subscriber rcvmap_signal_pub;
  ros::Publisher trajectory_pub_;

  /* data */

  Trajectory<TRAJ_ORDER> final_trajectory;

  Visualization::Ptr _visPtr_back_end;

  double _resolution;



  // Candidates are tried shortest first and the first one that survives
  // optimization is taken. Set false to optimize every candidate and let
  // trajSelectHandler pick the fastest one instead.
  bool stop_at_first_success{true};

  // Set once the global map has been gridded and the map kernel built; the
  // occupancy grid and kernel are uninitialized until then.
  bool map_ready_{false};

public:
  SweptVolumeManager::Ptr sv_manager;
  PCSmapManager::Ptr pcsmap_manager;
  TopoPath::Ptr _topo_path_finder;
  Config config;
  PlannerStrategy::Ptr planner_strategy;
  std::vector<Trajectory<TRAJ_ORDER>> backup_trajectories;
  PlanningVisualization::Ptr _visPtr_front_end;
  // back end
  PlannerManager(/* args */);
  ~PlannerManager();
  void init(ros::NodeHandle &nh, Config &config);
  void mapRcvCallBack(const std_msgs::Empty &msg);
  bool mapReady() const { return map_ready_; }
  vector<vector<Eigen::Vector2d>> generateTopoPath(Vector2d start,
                                                   Vector2d end);
  void systemFSM(Vector2d start, Vector2d end);
  Trajectory<TRAJ_ORDER> trajSelectHandler();
  void riskAwarePathHandler(
      const vector<vector<PathPoint>> &paths,
      const vector<bool> &path_is_fully_low_risk,
      const vector<vector<vector<PathPoint>>> &path_segments);
  void animationPlay();
  void publishTrajectory(const ros::Publisher &pub,
                         const Trajectory<5> &trajectory, int drone_id = 1,
                         int traj_id = 1);
  void getGoal(Vector2d end);

public:
  typedef shared_ptr<PlannerManager> Ptr;
};

#endif