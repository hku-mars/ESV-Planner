#include "map_manager/PCSmap_manager.h"
#include "swept_volume/sw_manager.hpp"

#include <Eigen/Dense>
#include <queue>
#include <random>
#include <set>
#include <unordered_set>
#include <utility>
#include <utils/config.hpp>
#include <utils/raycaster.h>
#include <vector>
#include <visualization_pkg/visualization_node.h>

using namespace std;
using namespace Eigen;

class TopoPath {
private:
  Config config;
  // sampling generator
  random_device rd_;
  default_random_engine eng_;
  uniform_real_distribution<double> rand_pos_;
  int kernel_size;
  BasicShape *robot_shape;
  int kernel_count;

  // parameter
  double max_sample_time_;
  int max_sample_num_;
  int max_raw_path_, max_raw_path2_;
  PCSmapManager::Ptr environment;

  Vector2d start_pt, end_pt;
  Vector3d boundary_xyzmin;
  Vector3d boundary_xyzmax;

  int counter = 0;

  // roadmap data structure, 0:start, 1:goal, 2-n: others
  list<GraphNode::Ptr> graph_;
  vector<vector<Eigen::Vector2d>> raw_paths_;
  vector<vector<Eigen::Vector2d>> short_paths_;
  Eigen::Vector2d start_pts_, end_pts_;

  PlanningVisualization::Ptr _vis_ptr;
  SweptVolumeManager::Ptr sv_manager;
  int GLX_SIZE, GLY_SIZE;
  double resolution_;
  double ratio_to_short_;
  int reserve_num_;
  bool test_mode;
  // Sampling rectangle: world axis aligned, centred between start and goal.
  // The configured margins are added to half the start/goal span on each
  // axis, so sample_half_*_ always covers both endpoints.
  Eigen::Vector2d sample_center_;
  double sample_margin_x;
  double sample_margin_y;
  double sample_width_x;
  double sample_width_y;
  double sample_half_x_;
  double sample_half_y_;
  int total_samples;
  int iterator_num;
  double push_extra_dist_;

  // raycasting
  vector<RayCaster> raycaster;

  list<GraphNode::Ptr> createGraph(Eigen::Vector2d start, Eigen::Vector2d end);
  Eigen::Vector2d getSample();
  vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector2d pt);
  bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector2d pt);
  double pathLength(const vector<Eigen::Vector2d> &path);
  vector<PathNode::Ptr> searchPaths();
  void depthFirstSearch(vector<GraphNode::Ptr> &vis);
  void pruneGraph();
  vector<PathNode::Ptr> pruneEquivalent(vector<PathNode::Ptr> &paths);
  vector<PathNode::Ptr> selectShortPaths(vector<PathNode::Ptr> &paths,
                                         int step);

  int shortestPath(const std::vector<PathNode::Ptr> &paths);

  // bool lineOfSight(const Eigen::Vector2d& start, const Eigen::Vector2d&
  // target, Eigen::Vector2d& collid_pt);

  /* Debug use only */
  std::vector<PathNode::Ptr> rawpathNodes_, shortpathNodes_;

  std::vector<Eigen::Vector2d>
  shortcutPath_fastPlanner(vector<Eigen::Vector2d> path, int path_id);
  vector<Eigen::Vector2d> discretizeLine_fastplanner(Eigen::Vector2d p1,
                                                     Eigen::Vector2d p2);

  bool lineOfSight(const Eigen::Vector2d &start, const Eigen::Vector2d &target);
  bool lineOfSight(const Eigen::Vector2d &start, const Eigen::Vector2d &target,
                   Eigen::Vector2d &collid_pt);

  std::string formatPoint(const Eigen::Vector2d &point) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << "(" << point.x() << ", "
       << point.y() << ")";
    return ss.str();
  }
  /* data */
public:
  bool sameTopoPath(const vector<Eigen::Vector2d> &path1,
                    const vector<Eigen::Vector2d> &path2);
  vector<PathNode::Ptr> shortcutPath(const vector<PathNode::Ptr> &paths);
  void setMapManager(PCSmapManager::Ptr map_manager,
                     SweptVolumeManager::Ptr sv);
  vector<Eigen::Vector2d>
  discretizePath_fastplanner(vector<Eigen::Vector2d> path);
  // std::vector<Eigen::Vector2d> generateInitPath(const Eigen::Vector2d& start,
  // const Eigen::Vector2d& target);
  TopoPath(/* args */);
  ~TopoPath();
  typedef shared_ptr<TopoPath> Ptr;
  vector<Eigen::Vector2d>
  discretizePath(const std::vector<Eigen::Vector2d> &path, int pt_num);

  void init(ros::NodeHandle &nh, Config &conf);
  // bool lineOfSight(Vector2d start, Vector2d end);
  // bool lineOfSight(const Eigen::Vector2d& start, const Eigen::Vector2d&
  // target);
  void findTopoPaths(Eigen::Vector2d start, Eigen::Vector2d end,
                     std::list<GraphNode::Ptr> &graph,
                     vector<PathNode::Ptr> &raw_paths,
                     vector<PathNode::Ptr> &short_paths_,
                     vector<PathNode::Ptr> &filtered_paths,
                     vector<PathNode::Ptr> &select_paths);
  void forestDebug(std::vector<std::vector<Eigen::Vector2d>> raw_paths,
                   std::vector<std::vector<Eigen::Vector2d>> short_paths);
  void findPathsWithWaypoints(
      const Eigen::Vector2d &start, const Eigen::Vector2d &end,
      const std::vector<std::vector<Eigen::Vector2d>>
          &path_waypoints, // Now takes a vector of waypoint sets
      std::vector<PathNode::Ptr> &raw_paths);
  void rcESDF(const Eigen::Vector2d &grid_point, const double yaw_idx,
              double &margin_dis, Eigen::Vector2d &gradient);

  vector<Eigen::Vector2i>
  extractCollisionPoints_fastcheck(const Eigen::Vector2d &grid_point,
                                   const int yaw_idx);
  // exclude_cells: grid cells already claimed by earlier pushes of the same
  // repair. Candidates there are skipped so neighboring samples converge
  // onto adjacent cells (forming a chain) instead of the same one -- a true
  // dead end then reports as "no candidate" rather than as an oscillation.
  bool searchValidatedPushPoint(
      const Eigen::Vector2d &colli_pt, double colli_yaw,
      const Eigen::Vector2d &prev_pt, const Eigen::Vector2d &next_pt,
      Eigen::Vector2d &pushed_pt, int &pushed_yaw_idx,
      const std::set<std::pair<int, int>> *exclude_cells = nullptr);
};
