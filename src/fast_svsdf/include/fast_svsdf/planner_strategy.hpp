#include <fstream>
#include <iostream>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <set>
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#include <utility>

#include "planner_algorithm/topo_path_search.h"
#include "utils/debug_publisher.hpp"
#include "visualization_pkg/visualization_node.h"
#include <map_manager/PCSmap_manager.h>
#include <planner_algorithm/back_end_optimizer.hpp>
#include <planner_algorithm/mid_end.hpp>
#include <utils/config.hpp>

class PlannerStrategy {
private:
  Config config;
  // back end

  OriTraj::Ptr ori_traj_generator;
  TrajOptimizer::Ptr minco_traj_optimizer;
  PlanningVisualization::Ptr _visPtr_front_end;
  Visualization::Ptr _visPtr_back_end;
  PCSmapManager::Ptr pcsmap_manager;

  int recent_ret;
  // AABB box size
  double bdx;
  double bdy;

  // fast check
  double _sta_threshold;
  double _resolution;

  int kernel_count;
  int kernel_size;

  int segTolerance;
  int MIN_FREE_SEGMENT_POINTS;
  double initial_yaw_offset;

  ros::Publisher shape_pub;
  std::string formatPoint(const Eigen::Vector2d &point) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << "(" << point.x() << ", "
       << point.y() << ")";
    return ss.str();
  }

  /* data */

public:
  SweptVolumeManager::Ptr sv_manager;
  TopoPath::Ptr _topo_path_finder;
  typedef shared_ptr<PlannerStrategy> Ptr;

  struct PathPoint {
    Eigen::Vector2d pos;    // Position in 2D
    double yaw;             // Yaw angle in radians
    int kernel_index;       // Corresponding kernel index
    bool is_collision_free; // Collision status
    int path_id;            // Which topology path this point belongs to
    bool is_high_risk;      // True if the region around this point needs full
                            // SE(2) optimization rather than R^2

    static int total_kernels; // Static member to store kernel count

    // Add default constructor
    PathPoint()
        : pos(Eigen::Vector2d::Zero()), yaw(0.0), kernel_index(0),
          is_collision_free(true), path_id(-1), is_high_risk(false) {}

    // Static method to initialize kernel count
    static void initializeKernelCount(int count) { total_kernels = count; }

    // Constructor with position and yaw
    PathPoint(const Eigen::Vector2d &position, double yaw_angle,
              bool collision_free = true)
        : pos(position), yaw(yaw_angle), is_collision_free(collision_free),
          path_id(-1), is_high_risk(false) {
      kernel_index =
          static_cast<int>((yaw + M_PI) * total_kernels / (2 * M_PI));
      kernel_index = std::max(0, std::min(total_kernels - 1, kernel_index));
    }

    // Constructor with position and kernel index
    PathPoint(const Eigen::Vector2d &position, int kernel_idx,
              bool collision_free = true)
        : pos(position), kernel_index(kernel_idx),
          is_collision_free(collision_free), path_id(-1), is_high_risk(false) {
      yaw = -M_PI + (2 * M_PI * kernel_index) / total_kernels;
    }

    Eigen::Vector3d toVector3d() const {
      return Eigen::Vector3d(pos.x(), pos.y(), yaw);
    }
  };

  // Updated PathSegment structure
  struct PathSegment {
    size_t pathIndex;    // Index of the original path
    size_t segmentIndex; // Index of the segment within the path
    bool isFree; // True if this is a free segment, false if it's a collision
                 // segment
    vector<Eigen::Vector3d> points; // The points in this segment (now 3D)

    // Updated getter functions for 3D points
    Eigen::Vector3d getFirstPoint() const { return points.front(); }
    Eigen::Vector3d getLastPoint() const { return points.back(); }
  };

public:
  PlannerStrategy();  // Declare the constructor
  ~PlannerStrategy(); // Declare the destructor
  void init(ros::NodeHandle &nh, Config &config);
  void fast_check(const vector<vector<Eigen::Vector2d>> &raw_paths_test_,
                  vector<vector<PathPoint>> &paths_out);
  void classifyPathsRisk(vector<vector<PathPoint>> &paths,
                         vector<bool> &path_is_fully_low_risk,
                         vector<vector<vector<PathPoint>>> &path_segments);
  void setEnvironment(TopoPath::Ptr topo_path,
                      PCSmapManager::Ptr pcsmap_manager,
                      TrajOptimizer::Ptr minco_traj_optimizer,
                      OriTraj::Ptr ori_traj_generator,
                      PlanningVisualization::Ptr _visPtr_front_end,
                      Visualization::Ptr _visPtr_back_end,
                      SweptVolumeManager::Ptr sv_manager);
  Trajectory<TRAJ_ORDER>
  generateTrajSE2(const std::vector<std::vector<PathSegment>> &segmentedPaths);
  void geneAABBPointsPath(const vector<Vector3d> &path);
  void generateTrajectoryFromPath(const vector<Vector3d> &path,
                                  Trajectory<TRAJ_ORDER> &traj);
  void segmentCollisionPaths(const vector<vector<Eigen::Vector3d>> &paths,
                             const vector<vector<bool>> &collisionStatuses,
                             vector<vector<PathSegment>> &segmentedPaths);
  bool trajSafe(const Trajectory<TRAJ_ORDER> &new_traj,
                std::vector<double> &collision_time_stamps,
                bool find_collision_time = false);
  std::vector<Trajectory<TRAJ_ORDER>>
  generateTrajR2(const vector<vector<Vector3d>> &path);
  void geneAABBPointsTrajectoryR2(Trajectory<TRAJ_ORDER> &traj);
  // Returns true only if every collision group was re-optimized into a safe
  // trajectory, so a caller can drop the candidate on failure.
  bool trajOptSVSDF(std::vector<double> &collision_time_stamps,
                    Trajectory<TRAJ_ORDER> &traj);

  // ---- risk-aware back end (consumes fast_check + classifyPathsRisk) ----
  static double pathLength(const vector<PathPoint> &path);
  void reportTrajectoryContinuity(const Trajectory<TRAJ_ORDER> &traj);
  static vector<Eigen::Vector3d> toVector3dPath(const vector<PathPoint> &path);
  // Whole-path R2 initialization for every risk class, followed by SE2 repair
  // of colliding pieces with fixed position/velocity/acceleration boundaries.
  bool optimizePathR2(const vector<PathPoint> &path,
                      Trajectory<TRAJ_ORDER> &traj_out);
  // Real-time playback of a finished trajectory, at the speed the trajectory
  // itself dictates.
  void playTrajectoryAnimation(const Trajectory<TRAJ_ORDER> &traj);
  void debugPrintOptX(const Trajectory<5> &traj, bool visualWayPoints);
  void decodeOptXFromTrajectory(const Trajectory<5> &traj,
                                Eigen::VectorXd &opt_x, int &N);
  void visulizeRobotKernel(const Eigen::Vector3i &grid_index, const int yaw_idx,
                           bool delete_old = true);
  vector<Eigen::Vector2i>
  extractCollisionPoints_fastcheck(const Eigen::Vector2d &grid_point,
                                   const int yaw_idx);
  Eigen::Vector2d
  calculatePushVector(const Eigen::Vector2d &curr_pt,
                      const vector<Eigen::Vector2i> &collision_obstacle_points,
                      double push_gain);
  vector<Eigen::Vector3d>
  formNewPath(const vector<Eigen::Vector2d> &pushed_points_2d);
  void trajOptSVSDF_collisonpath(std::vector<double> &collision_time_stamps,
                                 Trajectory<TRAJ_ORDER> &traj);

  bool verifyAndFixConnection_robot_running(
      const Eigen::Vector2d &pushed_point, int pushed_kernel_index,
      const vector<Eigen::Vector2d> &raw_path, size_t prev_waypoint,
      size_t next_waypoint, vector<PathPoint> &verified_path);
  bool checkAndAdjustSegment(const Eigen::Vector2d &start_pt,
                             const Eigen::Vector2d &end_pt, int &push_budget,
                             std::set<std::pair<int, int>> &pushed_cells,
                             vector<PathPoint> &out_path);
  // Optional output excludes start and includes end; empty on failure.
  bool segmentIsFree(const Eigen::Vector2d &start_pt,
                     const Eigen::Vector2d &end_pt,
                     vector<PathPoint> *checked_samples = nullptr);
  void pruneRepairedSegment(const Eigen::Vector2d &start_pt,
                            const Eigen::Vector2d &end_pt,
                            vector<PathPoint> &pts);
  void globalShortcutPath(vector<PathPoint> &path);
  bool classifySinglePathRisk(vector<PathPoint> &path,
                              vector<vector<PathPoint>> &segments);
  template <bool setInitialYaw>
  bool discretizedCollisionCheck(const Vector2d prev_pt, const Vector2d curr_pt,
                                 int &input_yaw_index, int &final_yaw_index,
                                 int maxdeepth = 10) {
    const double yaw_res = 2 * M_PI / kernel_count;
    const double yaw_start = -M_PI;
    int initial_kernel_index;

    // Get grid indices
    Eigen::Vector2i point_indx =
        pcsmap_manager->occupancy_map->getGridIndex2d(curr_pt);
    Eigen::Vector3i pt_indx(point_indx.x(), point_indx.y(), 0);

    if (setInitialYaw) {
      initial_kernel_index = input_yaw_index;

    } else {
      double yaw = atan2(curr_pt.y() - prev_pt.y(), curr_pt.x() - prev_pt.x());
      while (yaw > M_PI)
        yaw -= 2 * M_PI;
      while (yaw < -M_PI)
        yaw += 2 * M_PI;

      initial_kernel_index = std::round((yaw - yaw_start) / yaw_res);
      if (initial_kernel_index == kernel_count) {
        initial_kernel_index = 0;
      }
      input_yaw_index = initial_kernel_index;
    }
    // the final free robot yaw kernel index

    bool point_is_free = sv_manager->visit_kernels_by_distance(
        final_yaw_index, initial_kernel_index, pt_indx, maxdeepth);

    // visulizeRobotKernel(pt_indx, initial_kernel_index, true);

    // std::cout << "  [Collision Details]" << std::endl;
    // std::cout << "    Point: " << formatPoint(curr_pt) << std::endl;
    // std::cout << "    Grid Index: (" << point_indx.x() << ", " <<
    // point_indx.y() << ")" << std::endl; std::cout << "    Initial Kernel
    // Index: " << initial_kernel_index << std::endl; std::cout << "    Final
    // Kernel Index: " << final_yaw_index << std::endl;

    return point_is_free;
  }

  bool findNeighborWaypoints(const Eigen::Vector2d &curr_pt,
                             const vector<Eigen::Vector2d> &raw_path,
                             size_t &prev_waypoint, size_t &next_waypoint);

  void fast_check_collison_path(
      const vector<vector<Eigen::Vector2d>> &raw_paths_test_,
      vector<vector<Eigen::Vector3d>> &potentialCollisionPaths,
      vector<vector<Eigen::Vector3d>> &freePaths,
      vector<vector<bool>> &collisionStatus);

  bool verifyAndFixConnection_collison_path(
      const Eigen::Vector2d &pushed_point,
      const vector<Eigen::Vector2d> &raw_path, size_t prev_waypoint,
      size_t next_waypoint, double push_gain, int max_push_attempts);

  template <bool setInitialYaw>
  bool discretizedCollisionCheck_collison_path(const Vector2d prev_pt,
                                               const Vector2d curr_pt,
                                               int &input_yaw_index) {
    const double yaw_res = 2 * M_PI / kernel_count;
    const double yaw_start = -M_PI;
    int initial_kernel_index;

    // Get grid indices
    Eigen::Vector2i point_indx =
        pcsmap_manager->occupancy_map->getGridIndex2d(curr_pt);
    Eigen::Vector3i pt_indx(point_indx.x(), point_indx.y(), 0);

    if (setInitialYaw) {
      initial_kernel_index = input_yaw_index;

    } else {
      double yaw = atan2(curr_pt.y() - prev_pt.y(), curr_pt.x() - prev_pt.x());
      while (yaw > M_PI)
        yaw -= 2 * M_PI;
      while (yaw < -M_PI)
        yaw += 2 * M_PI;

      initial_kernel_index = std::round((yaw - yaw_start) / yaw_res);
      if (initial_kernel_index == kernel_count) {
        initial_kernel_index = 0;
      }
      input_yaw_index = initial_kernel_index;
    }

    int final_kernel_index;
    bool point_is_free = sv_manager->visit_kernels_by_distance(
        final_kernel_index, initial_kernel_index, pt_indx);

    // visulizeRobotKernel(pt_indx, initial_kernel_index, true);

    // if (!point_is_free && !setInitialYaw) {
    //     std::cout << "  [Collision Details]" << std::endl;
    //     std::cout << "    Point: " << formatPoint(curr_pt) << std::endl;
    //     std::cout << "    Grid Index: (" << point_indx.x() << ", " <<
    //     point_indx.y() << ")" << std::endl; std::cout << "    Yaw (rad): " <<
    //     std::fixed << std::setprecision(3) << atan2(curr_pt.y() -
    //     prev_pt.y(), curr_pt.x() - prev_pt.x()) << std::endl; std::cout << "
    //     Initial Kernel Index: " << initial_kernel_index << std::endl;
    //     std::cout << "    Final Kernel Index: " << final_kernel_index <<
    //     std::endl;
    // }

    return point_is_free;
  }
};