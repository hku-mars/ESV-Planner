#include "fast_svsdf/planner_strategy.hpp"
#include <set>
#include <utility>
int PlannerStrategy::PathPoint::total_kernels = 1;

namespace {

// Bound travel between uniform time samples using the piece's peak planar
// speed. Average arc length cannot bound spacing when speed varies within a
// piece. Remove yaw from the speed calculation: it is not a spatial axis.
int aabbSampleCount(const Piece<TRAJ_ORDER> &piece, double half_extent) {
  auto planar_coeffs = piece.getCoeffMat();
  planar_coeffs.row(2).setZero();
  const Piece<TRAJ_ORDER> planar_piece(piece.getDuration(), planar_coeffs);
  const double max_travel = planar_piece.getMaxVelRate() * piece.getDuration();
  const double step = 0.5 * half_extent;
  return std::max(1, static_cast<int>(std::ceil(max_travel / step)));
}

} // namespace


void PlannerStrategy::init(ros::NodeHandle &nh, Config &config) {
  kernel_count = config.kernel_yaw_num;
  std::vector<double> para;

  // Keep the loaded configuration. The parameter name shadows the member, so
  // `config = config` was only a self-assignment and left this->config at its
  // defaults (notably using the wrong initial duration in the R2/SE2 helpers).
  this->config = config;
  para = config.poly_params;
  initial_yaw_offset = (para[2] * PI / 180.0);
  PathPoint::initializeKernelCount(kernel_count);

  bdx = config.kernel_size * config.occupancy_resolution;
  bdy = config.kernel_size * config.occupancy_resolution;
  _resolution = config.occupancy_resolution;
  MIN_FREE_SEGMENT_POINTS = config.min_free_segmentPoints;
  segTolerance = config.segmentTolerance;
  kernel_size = config.kernel_size;
  _sta_threshold = config.safety_hor_fastCheck;

  shape_pub = nh.advertise<visualization_msgs::MarkerArray>("robot_shape", 10);
}

void PlannerStrategy::setEnvironment(TopoPath::Ptr topo_path,
                                     PCSmapManager::Ptr pcsmap,
                                     TrajOptimizer::Ptr minco_traj,
                                     OriTraj::Ptr ori_traj,
                                     PlanningVisualization::Ptr _visPtr_front,
                                     Visualization::Ptr _visPtr_back,
                                     SweptVolumeManager::Ptr sv) {
  _topo_path_finder = topo_path;
  pcsmap_manager = pcsmap;
  minco_traj_optimizer = minco_traj;
  ori_traj_generator = ori_traj;
  _visPtr_front_end = _visPtr_front;
  _visPtr_back_end = _visPtr_back;
  sv_manager = sv;
}
PlannerStrategy::PlannerStrategy() {
  // Constructor implementation
}

PlannerStrategy::~PlannerStrategy() {
  // Destructor implementation
}

void PlannerStrategy::geneAABBPointsPath(const vector<Vector3d> &path) {
  double temp_traj_parlength = 1.0;
  int index_gap = ceil(temp_traj_parlength /
                       ((pcsmap_manager->occupancy_map)->grid_resolution));
  int path_size = path.size();

  while (index_gap >= path_size - 1) {
    temp_traj_parlength /= 1.5;
    index_gap = ceil(temp_traj_parlength /
                     ((pcsmap_manager->occupancy_map)->grid_resolution));
  }

  index_gap = 2; // You might want to adjust this based on your requirements

  Vector3d tmp_pos{999, 999, 0};

  for (int ind = index_gap; ind < path_size - 1; ind += index_gap) {
    Vector3d wp = path[ind];
    // Third channel is yaw, not height -- zero it before querying the map.
    wp(2) = 0.0;
    minco_traj_optimizer->pcsmap_manager->getPointsInAABBOutOfLastOne(
        wp, tmp_pos, bdx / 1.5, bdy / 1.5, 0);
    tmp_pos = wp;
  }

  // Add the collected points to parallel_points
  for (const auto &pair : minco_traj_optimizer->pcsmap_manager->aabb_points) {
    minco_traj_optimizer->parallel_points.push_back(pair.second);
  }

  minco_traj_optimizer->parallel_points_num =
      minco_traj_optimizer->parallel_points.size();
  cout << "\033[32m=========parallel_points_num:==========\033[0m] ="
       << minco_traj_optimizer->parallel_points_num << endl;
  minco_traj_optimizer->lastTstar =
      vector<double>(minco_traj_optimizer->parallel_points_num, 0.0);

  // Visualize the aabb points
  minco_traj_optimizer->renderAABBpoints("se2_path");
}

void PlannerStrategy::generateTrajectoryFromPath(const vector<Vector3d> &path,
                                                 Trajectory<TRAJ_ORDER> &traj) {
  int path_size = path.size();
  vector<Vector3d> Q;
  vector<Vector3d> acc_list;
  vector<Matrix3d> rotatelist;
  int index_gap = 3; // You might want to adjust this

  for (int ind = index_gap; ind < path_size - 1; ind += index_gap) {
    Q.push_back(path[ind]);
    acc_list.push_back(Vector3d::Zero());
    rotatelist.push_back(Matrix3d::Identity());
  }

  int N = Q.size() + 1;

  Matrix3d initState = Matrix3d::Zero();
  Matrix3d finalState = Matrix3d::Zero();
  initState.col(0) = path.front();
  finalState.col(0) = path.back();

  VectorXd ts = config.inittime * VectorXd::Ones(N);

  Eigen::VectorXd opt_x;

  // recent_ret is the only success flag this function exports; reset it so a
  // failed getOriTraj cannot be read as the previous call's success.
  recent_ret = false;

  // R2 planner
  bool mid_ret = ori_traj_generator->getOriTraj(
      initState, finalState, Q, ts, acc_list, rotatelist, N, traj, opt_x);

  cout << "after original trajectory generator : " << sv_manager->traj_duration
       << endl;
  std::cout << "[planner strategy] Ori trajectory generated successfully!"
            << std::endl;

  if (mid_ret) {
    const int optimizer_status = minco_traj_optimizer->optimize_traj_lmbm(
        initState, finalState, opt_x, N, traj);
    // LMBM returns an integer status and still materializes its best trajectory
    // for some negative termination codes. Export whether there is a candidate
    // to validate; collision safety, below in the callers, is authoritative.
    recent_ret = traj.getPieceNum() > 0;
    if (optimizer_status < 0 && recent_ret) {
      ROS_WARN("SE2 optimizer stopped with status %d; validating its returned "
               "trajectory before deciding whether to keep it",
               optimizer_status);
    }
    std::cout << "==========mid_ret:" << mid_ret
              << " ret:=============== " << optimizer_status << std::endl;
  }
}

bool PlannerStrategy::trajSafe(const Trajectory<TRAJ_ORDER> &new_traj,
                               std::vector<double> &collision_time_stamps,
                               bool update_collision_time) {
  Eigen::Vector3d xt;
  Eigen::Matrix3d Rt;
  std::vector<Eigen::Vector3d> collision_positions;
  std::vector<double> collision_sdf_values;
  std::vector<double> collision_time_stamps_display;
  // update the input trajectory to swpet volume manager for collision check
  sv_manager->updateTraj(new_traj);
  sv_manager->swept_cal->sweptvolumeclear();

  // std::cout << "Number of points in final trajectory: " <<
  // minco_traj_optimizer->parallel_points_num << std::endl;

  // Check if the final trajectory is collision-free
  for (int pointindex = 0;
       pointindex < minco_traj_optimizer->parallel_points_num; ++pointindex) {
    // Get the evaluation position and set z-coordinate to 0
    Eigen::Vector3d pos_eva = minco_traj_optimizer->parallel_points[pointindex];

    pos_eva(2) = 0;

    // cout << "current checking point: " << pos_eva.transpose() << endl;

    Eigen::Vector3d gradp_rel;
    double time_star;

    // Evaluate the signed distance field (SDF) value at the position
    double sdf_value = sv_manager->getSDFofSweptVolume<false, false>(
        pos_eva, time_star, gradp_rel);
    // cout << "sdf_value: " << sdf_value << " time_star: " << time_star <<
    // endl;

    // Get grid index for debugging (optional)
    Eigen::Vector3i grid_index =
        pcsmap_manager->occupancy_map->getGridIndex3d(pos_eva);

    if (sdf_value < _sta_threshold) {
      // Collision detected at this point; store the information
      collision_positions.push_back(pos_eva);
      collision_sdf_values.push_back(sdf_value);
      if (update_collision_time) {
        collision_time_stamps.push_back(time_star);
        collision_time_stamps_display.push_back(time_star);
      } else {
        collision_time_stamps_display.push_back(time_star);
      }
    }
  }

  // If any collisions were detected
  if (!collision_positions.empty()) {
    // Summary in the ROS log: the per-point dump below only reaches stdout,
    // and the worst sdf is what says whether this is a marginal miss against
    // the threshold or the trajectory sitting inside an obstacle.
    size_t worst = 0;
    for (size_t i = 1; i < collision_sdf_values.size(); ++i) {
      if (collision_sdf_values[i] < collision_sdf_values[worst]) {
        worst = i;
      }
    }
    ROS_ERROR(
        "trajSafe: %zu/%d points violate the margin, worst sdf %.4f at "
        "(%.2f, %.2f) at t = %.3f of %.3f, threshold %.4f",
        collision_positions.size(), minco_traj_optimizer->parallel_points_num,
        collision_sdf_values[worst], collision_positions[worst].x(),
        collision_positions[worst].y(), collision_time_stamps_display[worst],
        new_traj.getTotalDuration(), _sta_threshold);

    // Publish the evidence: the obstacle cells the check objected to, and the
    // robot pose at the worst one. A stretch that looks clear in RViz and is
    // still rejected is settled by looking at these two together -- either the
    // red cells sit under the robot at that pose (the rejection is right), or
    // they do not (the check is looking at the wrong geometry).
    Eigen::Matrix3Xd violation_points(3, collision_positions.size());
    for (size_t i = 0; i < collision_positions.size(); ++i) {
      violation_points.col(i) = collision_positions[i];
    }
    _visPtr_back_end->visBalls(violation_points, "traj_safe/violations", false,
                               red, 0.9, _resolution);

    sv_manager->getStateOnTrajStamp(collision_time_stamps_display[worst], xt,
                                    Rt);
    Eigen::Vector3d worst_pose = xt;
    worst_pose(2) = 0.0;
    sv_manager->current_robot_shape->Transform(Rt, worst_pose);
    _visPtr_back_end->visPolytope(
        sv_manager->current_robot_shape->mesh_var, "traj_safe/worst_mesh",
        "traj_safe/worst_edge", false, 1, 7, red, 0.6);

    std::cout << "Collisions detected at the following points:" << std::endl;

    for (size_t i = 0; i < collision_positions.size(); ++i) {
      // Get the state on the trajectory at the collision time

      sv_manager->getStateOnTrajStamp(collision_time_stamps_display[i], xt, Rt);

      // Output collision details
      std::cout << "Collision " << i + 1 << ": Position = ("
                << collision_positions[i].x() << ", "
                << collision_positions[i].y() << ", "
                << collision_positions[i].z()
                << "), sdf vaule is: " << collision_sdf_values[i]
                << " Time = " << collision_time_stamps_display[i] << std::endl;
    }
    return false; // Trajectory is not safe
  }

  // No collisions detected; trajectory is safe
  return true;
}
std::vector<Trajectory<TRAJ_ORDER>> PlannerStrategy::generateTrajR2(
    const std::vector<std::vector<Eigen::Vector3d>> &paths) {
  std::cout << "Entering generateTrajectoryLBFGS function" << std::endl;
  std::cout << "Number of paths: " << paths.size() << std::endl;

  std::vector<double> collision_time_stamps;
  std::vector<Trajectory<TRAJ_ORDER>> trajectories_list;
  Trajectory<TRAJ_ORDER> trajectoryR2;

  for (size_t path_index = 0; path_index < paths.size(); ++path_index) {
    collision_time_stamps.clear();
    const int index_gap = 7;
    const auto &path = paths[path_index];
    int path_size = path.size();

    std::cout << "Processing path " << path_index + 1 << " of " << paths.size()
              << std::endl;
    std::cout << "Path size: " << path_size << std::endl;

    if (path_size < 2) {
      std::cout << "Path " << path_index + 1 << " is too short. Skipping."
                << std::endl;
      continue;
    }

    std::vector<Eigen::Vector3d> Q;
    std::vector<Eigen::Vector3d> acc_list;
    std::vector<Eigen::Matrix3d> rotatelist;

    // Clear previous data
    minco_traj_optimizer->pcsmap_manager->aabb_points.clear();
    minco_traj_optimizer->parallel_points.clear();

    for (int ind = index_gap; ind < path_size - 1; ind += index_gap) {
      Q.push_back(path[ind]);
      acc_list.push_back(Eigen::Vector3d::Zero());
      rotatelist.push_back(Eigen::Matrix3d::Identity());
    }

    int N = Q.size() + 1;
    ROS_INFO("[R2] %d checked path points -> %zu interior waypoints -> %d pieces",
             path_size, Q.size(), N);

    Eigen::Matrix3d initState = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d finalState = Eigen::Matrix3d::Zero();
    initState.col(0) = path.front();
    finalState.col(0) = path.back();

    Eigen::VectorXd ts = config.inittime * Eigen::VectorXd::Ones(N);

    Eigen::VectorXd opt_x;

    ros::Time t1 = ros::Time::now();

    bool mid_ret =
        ori_traj_generator->getOriTraj(initState, finalState, Q, ts, acc_list,
                                       rotatelist, N, trajectoryR2, opt_x);

    ros::Time t2 = ros::Time::now();
    double duration = (t2 - t1).toSec() * 1000;
    std::cout << "R2 optimization duration: " << duration << " ms" << std::endl;

    // Waypoint-parameterised guess handed to the R2 optimizer, and the
    // trajectory it returned. Both are published before any collision check or
    // SE2 repair touches the result.
    _visPtr_back_end->visMultipleTraj(
        "init_traj/R2",
        std::vector<Trajectory<TRAJ_ORDER>>{ori_traj_generator->init_traj},
        false, orange);

    if (mid_ret) {
      _visPtr_back_end->visMultipleTraj(
          "opt_traj/R2", std::vector<Trajectory<TRAJ_ORDER>>{trajectoryR2},
          false, blue);
      trajectories_list.push_back(trajectoryR2);
    } else {
      std::cout << "Failed to generate trajectory for path " << path_index + 1
                << std::endl;
    }
  }
  return trajectories_list;
  std::cout << "Exiting generateTrajectoryLBFGS function" << std::endl;
}

void PlannerStrategy::geneAABBPointsTrajectoryR2(
    Trajectory<TRAJ_ORDER> &trajR2) {
  // Clear previous data
  minco_traj_optimizer->pcsmap_manager->aabb_points.clear();
  minco_traj_optimizer->parallel_points.clear();
  minco_traj_optimizer->parallel_points_num = 0;
  minco_traj_optimizer->lastTstar.clear();

  const int piece_num = trajR2.getPieceNum();
  if (piece_num == 0) {
    ROS_ERROR("geneAABBPointsTrajectoryR2 called on an empty trajectory");
    return;
  }

  // Sample the curve itself, not its MINCO junctions. The junctions are the
  // N-1 spatial variables of the trajectory: a single piece trajectory has
  // none of them, so the old decode collected zero obstacle points and
  // trajSafe -- which only ever looks at parallel_points -- then declared
  // every one piece trajectory collision free without testing anything. Even
  // with several pieces the junctions are just the piece endpoints, so the
  // bulge in the middle of a long piece was covered only by whatever the
  // neighbouring boxes happened to reach.
  Vector3d tmp_pos{Vector3d::Zero()};
  bool have_last_box = false;

  for (int idx = 0; idx < piece_num; ++idx) {
    const auto &piece = trajR2[idx];
    const double piece_duration = piece.getDuration();
    const int num_samples = aabbSampleCount(piece, std::min(bdx, bdy));

    // j == 0 is the piece start, which is the previous piece's end; only the
    // very first piece needs it. j == num_samples closes the piece so the
    // trajectory's own end point is covered too.
    for (int j = (idx == 0 ? 0 : 1); j <= num_samples; ++j) {
      Vector3d pos = piece.getPos(j * piece_duration / num_samples);
      // The third channel of a trajectory sample is yaw, not height.
      pos(2) = 0.0;
      if (have_last_box) {
        minco_traj_optimizer->pcsmap_manager->getPointsInAABBOutOfLastOne(
            pos, tmp_pos, bdx, bdy, 0);
      } else {
        minco_traj_optimizer->pcsmap_manager->getPointsInAABBIntoSet(
            pos, bdx, bdy, 0);
        have_last_box = true;
      }
      tmp_pos = pos;
    }
  }
  // Add the collected points to parallel_points
  for (const auto &pair : minco_traj_optimizer->pcsmap_manager->aabb_points) {
    minco_traj_optimizer->parallel_points.push_back(pair.second);
  }

  minco_traj_optimizer->parallel_points_num =
      minco_traj_optimizer->parallel_points.size();
  cout << "\033[32m=========geneAABBPointsTrajectoryR2:==========\033[0m] ="
       << minco_traj_optimizer->parallel_points_num << endl;
  minco_traj_optimizer->lastTstar =
      vector<double>(minco_traj_optimizer->parallel_points_num, 0.0);
}

/**
 * @brief Optimizes trajectory segments containing collisions using SVSDF
 * optimization
 * @details This function performs the following steps:
 *          1. Identifies pieces with collisions using provided timestamps
 *          2. Groups adjacent collision pieces together for joint optimization
 *          3. Optimizes each group with one additional piece before and after
 * as buffer
 *          4. Validates optimization results for safety
 *          5. Either updates the trajectory with optimized pieces or clears the
 * entire trajectory if optimization fails
 *
 * @param collision_time_stamps Vector of time stamps where collisions occur in
 * the trajectory
 * @param traj Reference to the trajectory to be optimized, will be cleared if
 * optimization fails or produces unsafe results
 *
 * @note The function processes collision pieces in groups rather than
 * individually to handle adjacent collisions more effectively
 * @note If optimization fails or produces unsafe results, the entire trajectory
 * will be cleared using traj.clear()
 *
 * @return void The original trajectory is modified in place: either optimized
 * or cleared
 */

bool PlannerStrategy::trajOptSVSDF(std::vector<double> &collision_time_stamps,
                                   Trajectory<TRAJ_ORDER> &traj) {
  std::cout << "\n============= Starting trajOptLBFGS =============\n";

  // Vector to store free trajectory pieces and optimized pieces separately
  std::vector<Trajectory<TRAJ_ORDER>>
      free_trajectories; // For collision-free pieces
  std::vector<Trajectory<TRAJ_ORDER>>
      optimized_trajectories; // For optimized pieces

  if (collision_time_stamps.empty()) {
    // Reached only when the caller saw a collision it could not localize in
    // time; there is nothing to repair, so the candidate is unusable.
    std::cout << "No collision timestamps provided. Exiting.\n";
    return false;
  }

  // Cleared by any group that fails to optimize or fails its safety check.
  bool all_groups_safe = true;

  // Track which pieces have collisions
  std::set<int> collision_pieces;
  int trajectory_buffer =
      2; // Number of pieces before and after a collision piece to optimize

  // Identify unique pieces with collisions
  std::cout << "\n--- Identifying Collision Pieces ---\n";
  for (double t : collision_time_stamps) {
    int currentIdx;
    double prevStart, nextEnd;
    std::cout << "Processing collision at time: " << t << std::endl;

    traj.getGlobalPieceTimes(t, currentIdx, prevStart, nextEnd);

    if (currentIdx == -1) {
      std::cout << "Error: Collision time stamp " << t
                << " not found in trajectory" << std::endl;
      continue;
    }

    std::cout << "Found collision in piece " << currentIdx
              << " (PieceStartTime: " << traj.getPieceStartTime(currentIdx)
              << ", Duration: " << traj[currentIdx].getDuration() << ")\n";
    collision_pieces.insert(currentIdx);
  }

  std::cout << "\nUnique pieces with collisions: " << collision_pieces.size()
            << std::endl;
  std::cout << "Collision pieces indices: ";
  for (int idx : collision_pieces) {
    std::cout << idx << " ";
  }
  std::cout << std::endl;

  // Group adjacent collision pieces
  std::vector<std::pair<int, int>> optimization_groups;
  if (!collision_pieces.empty()) {
    const int MERGE_THRESHOLD =
        4; // Maximum gap between collision pieces to merge them

    auto it = collision_pieces.begin();
    int group_start = *it;
    int group_end = group_start;

    for (++it; it != collision_pieces.end(); ++it) {
      if (*it - group_end <= MERGE_THRESHOLD) {
        // Pieces are close enough to merge
        group_end = *it;
      } else {
        // Gap is too large, create new group
        optimization_groups.emplace_back(group_start, group_end);
        group_start = *it;
        group_end = group_start;
      }
    }
    // Add the last group
    optimization_groups.emplace_back(group_start, group_end);
  }

  // Mark pieces that will be optimized (including buffers)
  std::set<int> optimized_with_buffer;
  for (const auto &group : optimization_groups) {
    int min_idx =
        std::max(0, group.first - trajectory_buffer); // Include buffer
    int max_idx =
        std::min(group.second + trajectory_buffer,
                 static_cast<int>(traj.getPieceNum()) - 1); // Include buffer
    for (int i = min_idx; i <= max_idx; ++i) {
      optimized_with_buffer.insert(i);
    }
  }

  // Extract and store free trajectory pieces
  // (pieces not being optimized and not in buffer zones)
  int current_free_start = 0;
  for (int i = 0; i <= traj.getPieceNum(); ++i) {
    if (i == traj.getPieceNum() ||
        optimized_with_buffer.find(i) != optimized_with_buffer.end()) {
      // If we've found free pieces to store
      if (i > current_free_start) {
        // Only store if this range isn't part of a buffer
        bool is_valid_free_segment = true;
        for (int j = current_free_start; j < i; ++j) {
          if (optimized_with_buffer.find(j) != optimized_with_buffer.end()) {
            is_valid_free_segment = false;
            break;
          }
        }

        if (is_valid_free_segment) {
          Trajectory<TRAJ_ORDER> free_segment;
          for (int j = current_free_start; j < i; ++j) {
            free_segment.emplace_back(traj[j].getDuration(),
                                      traj[j].getCoeffMat());
          }
          if (free_segment.getPieceNum() > 0) {
            free_trajectories.push_back(free_segment);
          }
        }
      }
      // Update the next potential starting point for free pieces
      if (i < traj.getPieceNum()) {
        current_free_start = i + 1;
        // Skip ahead if we're in a buffer zone
        while (current_free_start < traj.getPieceNum() &&
               optimized_with_buffer.find(current_free_start) !=
                   optimized_with_buffer.end()) {
          current_free_start++;
        }
      }
    }
  }

  std::cout << "\nOptimization groups formed: " << optimization_groups.size()
            << std::endl;
  for (const auto &group : optimization_groups) {
    std::cout << "Group: pieces " << group.first << " to " << group.second
              << std::endl;
  }

  // Process each group
  int group_count = 0;
  for (const auto &group : optimization_groups) {
    /* code */

    std::cout << "\n=== Processing Group " << ++group_count << "/"
              << optimization_groups.size() << " (Pieces " << group.first
              << " to " << group.second << ") ===\n";

    Eigen::VectorXd opt_x;
    int N;
    decodeOptXFromTrajectory(traj, opt_x, N);
    std::cout << "Decoded trajectory with N = " << N << std::endl;

    // Determine the range of pieces to optimize (including one piece before and
    // after the group)
    int min_idx = std::max(0, group.first - trajectory_buffer);
    int max_idx = std::min(group.second + trajectory_buffer,
                           static_cast<int>(traj.getPieceNum()) - 1);
    int num_pieces = max_idx - min_idx + 1;

    std::cout << "Optimization range:\n"
              << "- Start piece: " << min_idx << "\n"
              << "- End piece: " << max_idx << "\n"
              << "- Number of pieces: " << num_pieces << std::endl;

    // Get time range for optimization
    double group_start_time = traj.getPieceStartTime(min_idx);
    double group_end_time =
        traj.getPieceStartTime(max_idx) + traj[max_idx].getDuration();

    std::cout << "Time range for optimization:\n"
              << "- Start time: " << group_start_time << "\n"
              << "- End time: " << group_end_time << "\n"
              << "- Total duration: " << (group_end_time - group_start_time)
              << std::endl;

    // Extract states for optimization
    Eigen::Vector3d xt_start, xt_end;
    Eigen::Matrix3d Rt_start, Rt_end;
    sv_manager->getStateOnTrajStamp(group_start_time, xt_start, Rt_start);
    sv_manager->getStateOnTrajStamp(group_end_time, xt_end, Rt_end);

    // Prepare initial and final states
    Matrix3d initState, finalState;
    initState.col(0) = xt_start;
    initState.col(1) = traj[min_idx].getVel(0);
    initState.col(2) = traj[min_idx].getAcc(0);
    finalState.col(0) = xt_end;
    finalState.col(1) = traj[max_idx].getVel(traj[max_idx].getDuration());
    finalState.col(2) = traj[max_idx].getAcc(traj[max_idx].getDuration());

    // Extract optimization variables
    std::cout << "\nExtracting optimization variables..." << std::endl;
    // temporalDim (num_pieces taus) + spatialDim (3 per junction), matching
    // what optimize_traj_lmbm copies out of it.
    Eigen::VectorXd pieces_opt_x(num_pieces + 3 * (num_pieces - 1));

    // Extract tau values
    pieces_opt_x.segment(0, num_pieces) = opt_x.segment(min_idx, num_pieces);

    // Extract control points
    for (int i = 0; i < num_pieces - 1; ++i) {
      pieces_opt_x.segment(num_pieces + i * 3, 3) =
          opt_x.segment(N + (min_idx + i) * 3, 3);
    }

    std::cout << "\nSetting up collision checking..." << std::endl;
    // Clear and update collision checking data
    minco_traj_optimizer->pcsmap_manager->aabb_points.clear();
    minco_traj_optimizer->parallel_points.clear();

    // Sample points for collision checking
    Vector3d tmp_pos{Vector3d::Zero()};
    bool have_last_box = false;
    for (int curr_idx = min_idx; curr_idx <= max_idx; ++curr_idx) {
      const auto &piece = traj[curr_idx];
      double piece_duration = piece.getDuration();

      int num_samples = aabbSampleCount(piece, std::min(bdx, bdy));

      // Include both piece ends: j == 0 only for the first piece of the group
      // (afterwards it repeats the previous piece's end), j == num_samples so
      // the group's final pose is covered as well.
      for (int j = (curr_idx == min_idx ? 0 : 1); j <= num_samples; ++j) {
        Vector3d pos = piece.getPos(j * piece_duration / num_samples);
        // The third channel of a trajectory sample is yaw, not height. Left in
        // place it centred the obstacle box at z = yaw (radians read as
        // metres), so the query ran in an empty map layer and the optimizer
        // saw no obstacles at all for any heading away from zero.
        pos(2) = 0.0;
        // Full bdx/bdy box, matching what trajSafe gathers when it judges the
        // result. The old bdx/3 window was 0.7 m at this map scale -- narrower
        // than the robot's own 1.05 m footprint radius -- so the optimizer was
        // blind to the very obstacles its body was hitting.
        // A sentinel "last" centre cannot seed the chain: projInMap clamps
        // it onto the map corner, which would mask real cells out of the
        // first box.
        if (have_last_box) {
          minco_traj_optimizer->pcsmap_manager->getPointsInAABBOutOfLastOne(
              pos, tmp_pos, bdx, bdy, 0);
        } else {
          minco_traj_optimizer->pcsmap_manager->getPointsInAABBIntoSet(
              pos, bdx, bdy, 0);
          have_last_box = true;
        }
        tmp_pos = pos;
      }
    }

    for (const auto &pair : minco_traj_optimizer->pcsmap_manager->aabb_points) {
      minco_traj_optimizer->parallel_points.push_back(pair.second);
    }

    // The cost function walks parallel_points[0 .. parallel_points_num), so
    // the count has to be re-published with the points. Left stale it still
    // held the count from the last whole-trajectory geneAABBPoints call, which
    // is far larger than this group's set: the optimizer then read past the
    // end of the vector and pushed the trajectory away from garbage positions,
    // "converging" straight into the real obstacles.
    minco_traj_optimizer->parallel_points_num =
        minco_traj_optimizer->parallel_points.size();
    minco_traj_optimizer->lastTstar =
        vector<double>(minco_traj_optimizer->parallel_points_num, 0.0);
    minco_traj_optimizer->renderAABBpoints(
        "se2_pieces_" + std::to_string(min_idx) + "_" + std::to_string(max_idx));

    ROS_WARN("Group pieces %d-%d of %d: optimizing against %d obstacle points",
             min_idx, max_idx, traj.getPieceNum(),
             minco_traj_optimizer->parallel_points_num);

    std::cout << "\nStarting trajectory optimization..." << std::endl;
    // Optimize the affected pieces
    Trajectory<TRAJ_ORDER> optimized_pieces;
    ros::Time t1 = ros::Time::now();

    const int optimizer_status = minco_traj_optimizer->optimize_traj_lmbm(
        initState, finalState, pieces_opt_x, num_pieces, optimized_pieces);

    ros::Time t2 = ros::Time::now();
    double duration = (t2 - t1).toSec() * 1000;
    std::cout << "SE2 piece optimization duration: " << duration << " ms"
              << std::endl;

    // LMBM's integer status describes how it terminated, but it also returns
    // the best trajectory for negative statuses. Do not discard a physically
    // safe result because of that status; validate every structurally complete
    // candidate and let the swept-volume check make the acceptance decision.
    if (optimized_pieces.getPieceNum() != num_pieces) {
      ROS_ERROR("SE2 optimizer status %d returned %d pieces for a %d piece "
                "group, cannot splice it back",
                optimizer_status, optimized_pieces.getPieceNum(), num_pieces);
      all_groups_safe = false;
      break;
    }

    if (optimizer_status < 0) {
      ROS_WARN("SE2 optimizer stopped with status %d for pieces %d-%d; "
               "checking the returned trajectory for safety",
               optimizer_status, min_idx, max_idx);
    } else {
      std::cout << "\nOptimization successful!" << std::endl;
    }

    // Generate obstacle points around the returned trajectory and validate it
    // before changing the original R2 trajectory.
    geneAABBPointsTrajectoryR2(optimized_pieces);
    std::cout << "Checking if optimized trajectory is safe..." << std::endl;
    if (!trajSafe(optimized_pieces, collision_time_stamps, false)) {
      // Do not add this result to the green SE2 visualization. Previously it
      // was visualized as an optimized trajectory even on this failure path,
      // which made a rejected/colliding segment appear collision-free in RViz.
      ROS_ERROR("SE2 trajectory for pieces %d-%d is still unsafe; dropping "
                "candidate",
                min_idx, max_idx);
      all_groups_safe = false;
      break;
    }

    std::cout << "Optimized trajectory is safe. Updating pieces..."
              << std::endl;
    optimized_trajectories.push_back(optimized_pieces);
    for (int j = 0; j < num_pieces; ++j) {
      traj[min_idx + j] = optimized_pieces[j];
    }
    // Publish the complete repaired trajectory once the entire group has been
    // spliced, rather than exposing partially updated group state.
    sv_manager->updateTraj(traj);
    std::cout << "Trajectory segment updated successfully" << std::endl;
  }
  double length2 = 0, length1 = 0;
  for (size_t i = 0; i < free_trajectories.size(); i++) {
    length2 = TrajectoryLength<5>::getLength(free_trajectories[i]);
    cout << "Total trajectory length for R2 number " << i << " is: " << length2
         << endl;
  }
  for (size_t i = 0; i < optimized_trajectories.size(); i++) {
    length1 = TrajectoryLength<5>::getLength(optimized_trajectories[i]);
    cout << "Total trajectory length for SE2 number " << i << " is: " << length1
         << endl;
  }

  // cout << "Calculation Trajectory length is: " << length1 + length2 << endl;

  _visPtr_back_end->visMultipleTraj("final_traj/R2", free_trajectories, false,
                                    red);
  _visPtr_back_end->visMultipleTraj("final_traj/SE2", optimized_trajectories,
                                    false, green);
  cout << "collision free trajectory" << free_trajectories.size() << endl;
  cout << "optmizied trajectory" << optimized_trajectories.size() << endl;

  std::cout << "\n============= Completed trajOptLBFGS =============\n";
  return all_groups_safe;
}

void PlannerStrategy::playTrajectoryAnimation(
    const Trajectory<TRAJ_ORDER> &traj) {
  const double total = traj.getTotalDuration();
  if (traj.getPieceNum() == 0 || total <= 1e-2) {
    ROS_WARN("[animation] nothing to play (%d pieces, %.3f s)",
             traj.getPieceNum(), total);
    return;
  }

  // sw_manager::process reads the pose off its own copy of the trajectory.
  sv_manager->updateTraj(traj);

  std::cout << "[animation] playing " << total << " s of trajectory"
            << std::endl;

  // Redraw cadence only -- it decides how smooth the animation looks, never
  // how fast the robot travels. Every frame is placed by the elapsed wall
  // clock, so a slow frame is skipped over rather than stretching the motion.
  const ros::Duration frame_period(0.02);
  const ros::Time wall_start = ros::Time::now();
  // process() places the robot at (now - trajStamp) on the trajectory, so the
  // stamp is the only thing it needs to walk the whole path in real time.
  sv_manager->setTrajStamp(wall_start.toSec());

  // process() only draws while its own delta is strictly inside the
  // trajectory, so the loop stops just short of the end.
  while (ros::ok() && (ros::Time::now() - wall_start).toSec() < total) {
    sv_manager->process(traj);
    frame_period.sleep();
  }

  std::cout << "[animation] done" << std::endl;
}

void PlannerStrategy::decodeOptXFromTrajectory(const Trajectory<5> &traj,
                                               Eigen::VectorXd &opt_x, int &N) {
  // Update N with the number of pieces in the trajectory
  N = traj.getPieceNum();

  int temporalDim = N;
  int spatialDim = 3 * (N - 1);

  // Resize opt_x to match the trajectory size
  opt_x.resize(temporalDim + spatialDim);

  // Extract and transform time intervals (tau)
  Eigen::VectorXd T = traj.getDurations();
  Eigen::VectorXd tau(temporalDim);
  ori_traj_generator->backwardT(T, tau);
  opt_x.segment(0, temporalDim) = tau;

  // Extract waypoints (xi). MINCO's N-1 spatial variables are the *junctions*
  // between pieces, i.e. the end of piece i == the start of piece i+1, so the
  // constant term to read is the one of piece i+1. Reading piece i instead
  // shifted every waypoint back by one piece: the first variable became the
  // trajectory's own start point (already fixed by initState) and the last
  // junction was dropped, so every warm start handed to the SE(2) optimizer
  // described a different trajectory than the one being repaired.
  Eigen::Matrix3Xd P(3, N - 1);
  for (int i = 0; i < N - 1; ++i) {
    // Column 5 of a 5th order coefficient matrix is the constant term, i.e.
    // the position at the start of that piece.
    P.col(i) = traj[i + 1].getCoeffMat().col(5);
  }

  Eigen::VectorXd xi(spatialDim);
  ori_traj_generator->backwardP(P, xi);
  opt_x.segment(temporalDim, spatialDim) = xi;
}

void PlannerStrategy::visulizeRobotKernel(const Eigen::Vector3i &grid_index,
                                          const int yaw_idx, bool delete_old) {
  // Get robot shape from swept volume manager
  BasicShape *robot_shape = sv_manager->getRobotShape();

  visualization_msgs::MarkerArray marker_array;

  if (delete_old) {
    // Add a deletion marker first
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    marker_array.markers.push_back(delete_marker);
  }

  // Setup base marker properties
  visualization_msgs::Marker cell_marker;
  cell_marker.header.frame_id = "map";
  cell_marker.header.stamp = ros::Time::now();
  cell_marker.ns = "robot_shape";
  cell_marker.type = visualization_msgs::Marker::CUBE;
  cell_marker.action = visualization_msgs::Marker::ADD;

  cell_marker.scale.x = _resolution;
  cell_marker.scale.y = _resolution;
  cell_marker.scale.z = 0.01; // Thin height for 2D visualization
  // cell_marker.lifetime = ros::Duration(4.0);

  int marker_id = 0;
  int half_size = (kernel_size - 1) / 2;

  // Iterate through the kernel and place cubes
  for (int i = 0; i < kernel_size; i++) {
    for (int j = 0; j < kernel_size; j++) {
      if (robot_shape->byte_shape_kernels[yaw_idx].getOccupied(i, j)) {
        // Calculate relative grid position
        Eigen::Vector3i rel_grid_pos(grid_index(0) + (i - half_size),
                                     grid_index(1) + (j - half_size),
                                     grid_index(2));

        // Get world coordinates for this grid cell
        Eigen::Vector3d world_pos =
            pcsmap_manager->occupancy_map->getGridCubeCenter3d(rel_grid_pos);

        cell_marker.pose.position.x = world_pos(0);
        cell_marker.pose.position.y = world_pos(1);
        cell_marker.pose.position.z = world_pos(2);

        cell_marker.pose.orientation.w = 1.0;
        cell_marker.pose.orientation.x = 0.0;
        cell_marker.pose.orientation.y = 0.0;
        cell_marker.pose.orientation.z = 0.0;
        // Set color (steel blue with transparency)
        cell_marker.color.r = 0.4;
        cell_marker.color.g = 0.7;
        cell_marker.color.b = 1.0;
        cell_marker.color.a = 0.5;

        if (pcsmap_manager->occupancy_map->isIndexOccupied3d(rel_grid_pos)) {
          cell_marker.color.r = 1.0;
          cell_marker.color.g = 0.0;
          cell_marker.color.b = 0.0;
        }

        cell_marker.id = marker_id++;
        marker_array.markers.push_back(cell_marker);
      }
    }

    shape_pub.publish(marker_array);
  }
}

vector<Eigen::Vector2i> PlannerStrategy::extractCollisionPoints_fastcheck(
    const Eigen::Vector2d &grid_point, const int yaw_idx) {
  vector<Eigen::Vector2i> collision_points;
  BasicShape *robot_shape = sv_manager->getRobotShape();
  Eigen::Vector2i grid_index =
      pcsmap_manager->occupancy_map->getGridIndex2d(grid_point);
  int half_size = (kernel_size - 1) / 2;

  bool first_collision =
      true; // Track if this is the first collision point found

  for (int i = 0; i < kernel_size; i++) {
    for (int j = 0; j < kernel_size; j++) {
      if (robot_shape->byte_shape_kernels[yaw_idx].getOccupied(i, j)) {
        Eigen::Vector2i rel_grid_pos(grid_index(0) + (i - half_size),
                                     grid_index(1) + (j - half_size));

        if (pcsmap_manager->occupancy_map->isIndexOccupied2d(rel_grid_pos)) {
          //   if (first_collision) {
          //     std::cout
          //         << "\n[COLLISION] First collision detected at world point "
          //         << formatPoint(grid_point) << std::endl;
          //     first_collision = false;
          //   }

          Eigen::Vector2d world_pos =
              pcsmap_manager->occupancy_map->getGridCubeCenter2d(rel_grid_pos);
          //   std::cout << "  → Collision point at grid: (" << rel_grid_pos.x()
          //             << ", " << rel_grid_pos.y()
          //             << "), world: " << formatPoint(world_pos) << std::endl;

          collision_points.push_back(rel_grid_pos);
        }
      }
    }
  }

  if (!collision_points.empty()) {
    std::cout << "  Total collision points: " << collision_points.size()
              << " at yaw index: " << yaw_idx << std::endl;
  }

  return collision_points;
}

/**
 * @brief Forms a new path from pushed points and converts to 3D with yaw
 * @param pushed_points_2d Vector of pushed 2D points
 * @return Vector of 3D points with yaw angles
 */
vector<Eigen::Vector3d>
PlannerStrategy::formNewPath(const vector<Eigen::Vector2d> &pushed_points_2d) {
  vector<Eigen::Vector3d> new_path_3d;

  // Handle first point (zero yaw)
  if (!pushed_points_2d.empty()) {
    new_path_3d.push_back(
        Eigen::Vector3d(pushed_points_2d[0].x(), pushed_points_2d[0].y(), 0.0));
  }

  // Process remaining points
  for (size_t i = 1; i < pushed_points_2d.size(); ++i) {
    const Eigen::Vector2d &curr_pt = pushed_points_2d[i];
    const Eigen::Vector2d &prev_pt = pushed_points_2d[i - 1];

    // Calculate yaw from previous to current point
    double dx = curr_pt.x() - prev_pt.x();
    double dy = curr_pt.y() - prev_pt.y();
    double yaw = atan2(dy, dx);
    yaw = yaw + initial_yaw_offset;
    while (yaw > M_PI)
      yaw -= 2 * M_PI;
    while (yaw < -M_PI)
      yaw += 2 * M_PI;

    new_path_3d.push_back(Eigen::Vector3d(curr_pt.x(), curr_pt.y(), yaw));
  }

  return new_path_3d;
}

/**
 * @brief Calculate push vector for a point based on nearby obstacles
 * @param curr_pt Current point position
 * @param collision_obstacle_points List of obstacle points causing collision
 * @param push_gain Scaling factor for push magnitude
 * @return Vector indicating direction and magnitude to push
 */
Eigen::Vector2d PlannerStrategy::calculatePushVector(
    const Eigen::Vector2d &curr_pt,
    const vector<Eigen::Vector2i> &collision_obstacle_points,
    double push_gain) {
  Eigen::Vector2d push_direction = Eigen::Vector2d::Zero();

  // Calculate accumulated push direction from all obstacle points
  for (size_t i = 0; i < collision_obstacle_points.size(); i++) {
    const auto &obstacle_point = collision_obstacle_points[i];

    Eigen::Vector2d obstacle_pos =
        pcsmap_manager->occupancy_map->getGridCubeCenter2d(
            Eigen::Vector2i(obstacle_point.x(), obstacle_point.y()));

    Eigen::Vector2d diff = curr_pt - obstacle_pos;

    if (diff.norm() > 1e-6) {
      Eigen::Vector2d normalized_diff = diff.normalized();
      push_direction += normalized_diff;
    } else {
      std::cout << "Debug: Skipping obstacle - too close to current point"
                << std::endl;
    }
  }

  // Normalize and scale the push direction
  if (push_direction.norm() > 1e-6) {
    push_direction.normalize();
    push_direction *= push_gain;

  } else {
    std::cout
        << "Debug: Push direction magnitude too small - returning zero vector"
        << std::endl;
  }

  return push_direction;
}

// Add this function right before or after your existing fast_check function
bool PlannerStrategy::findNeighborWaypoints(
    const Eigen::Vector2d &curr_pt, const vector<Eigen::Vector2d> &raw_path,
    size_t &prev_waypoint, size_t &next_waypoint) {
  double min_distance = std::numeric_limits<double>::max();
  bool found = false;
  constexpr double DISTANCE_THRESHOLD = 0.1; // Adjust if needed

  for (size_t j = 0; j < raw_path.size() - 1; j++) {
    // Vector from waypoint j to j+1
    Eigen::Vector2d segment = raw_path[j + 1] - raw_path[j];
    double segment_length_sq = segment.squaredNorm();

    if (segment_length_sq < 1e-6)
      continue; // Skip tiny segments

    // Vector from waypoint j to current point
    Eigen::Vector2d to_point = curr_pt - raw_path[j];

    // Calculate projection parameter
    double t = to_point.dot(segment) / segment_length_sq;

    // Skip if projection is outside segment
    if (t < 0.0 || t > 1.0)
      continue;

    // Calculate closest point on segment
    Eigen::Vector2d projection = raw_path[j] + t * segment;
    double distance = (curr_pt - projection).norm();

    if (distance < min_distance - 1e-9) {
      min_distance = distance;
      prev_waypoint = j;
      next_waypoint = j + 1;
      found = true;
    } else if (found && distance <= min_distance + 1e-9) {
      // Tie: the point sits on a shared waypoint, equidistant to the
      // segment behind and the segment ahead. The collision belongs to
      // where travel continues, so repair the segment ahead -- attributing
      // it to the segment behind patches ground already passed and leaves
      // the actual offending segment untouched.
      prev_waypoint = j;
      next_waypoint = j + 1;
    }
  }

  std::cout << "  [NEIGHBOR SEARCH] Found waypoints: " << prev_waypoint
            << " -> " << next_waypoint << " (distance: " << min_distance << ")"
            << std::endl;

  return found && min_distance < DISTANCE_THRESHOLD;
}

void PlannerStrategy::fast_check(
    const vector<vector<Eigen::Vector2d>> &raw_paths_test_,
    vector<vector<PathPoint>> &paths_out) {
  std::cout << "\n=== Starting Fast Check for " << raw_paths_test_.size()
            << " paths ===" << std::endl;

  paths_out.clear();

  vector<Eigen::Vector3d> visCollisionPoints;
  vector<vector<bool>> visCollisionStatus;
  vector<vector<Eigen::Vector2d>> new_path_2d;
  vector<vector<Eigen::Vector3d>> grid_points_per_path;
  vector<vector<bool>> grid_is_red_per_path;

  ros::Time t1 = ros::Time::now();
  // Calculate yaw from vector direction
  auto calculateForwardYaw = [](const Eigen::Vector2d &from,
                                const Eigen::Vector2d &to) -> double {
    double yaw = atan2(to.y() - from.y(), to.x() - from.x());
    while (yaw > M_PI)
      yaw -= 2 * M_PI;
    while (yaw < -M_PI)
      yaw += 2 * M_PI;
    return yaw;
  };

  // Convert yaw to kernel index
  auto yawToKernelIndex = [this](double yaw) -> int {
    const double yaw_res = 2 * M_PI / kernel_count;
    const double yaw_start = -M_PI;
    int index = std::round((yaw - yaw_start) / yaw_res);
    return (index == kernel_count) ? 0 : index;
  };

  for (size_t path_idx = 0; path_idx < raw_paths_test_.size(); path_idx++) {
    std::cout << "\n[PATH " << path_idx + 1 << "/" << raw_paths_test_.size()
              << "]" << std::endl;
    vector<Eigen::Vector2d> raw_path = raw_paths_test_[path_idx];
    auto discrete_path =
        _topo_path_finder->discretizePath_fastplanner(raw_path);
    vector<PathPoint> current_path;
    bool path_has_collision = false;

    // Raw segments whose repair already failed: every further discretized
    // sample on such a segment would fail the identical push/verify attempt,
    // so it is marked unresolved directly. Indexed by waypoint pair, and
    // invalidated whenever a successful splice renumbers raw_path.
    std::set<std::pair<size_t, size_t>> failed_segments;

    // Initialize first point with forward-facing yaw
    if (discrete_path.size() >= 2) {
      double initial_yaw =
          calculateForwardYaw(discrete_path[0], discrete_path[1]);
      int initial_kernel = yawToKernelIndex(initial_yaw);
      current_path.emplace_back(discrete_path[0], initial_kernel);
    } else if (!discrete_path.empty()) {
      current_path.emplace_back(discrete_path[0], initial_yaw_offset);
    }

    size_t i = 1;
    while (i < discrete_path.size()) {
      Eigen::Vector2d curr_pt = discrete_path[i];
      bool final_push_success = false;
      int initial_kernel_index, final_kernel_index;
      // cout << "current check point: " << curr_pt.transpose() << endl;

      // Calculate forward-facing yaw first
      double forward_yaw =
          (i < discrete_path.size() - 1)
              ? calculateForwardYaw(curr_pt, discrete_path[i + 1])
              : calculateForwardYaw(discrete_path[i - 1], curr_pt);
      // cout << "forward yaw: " << forward_yaw << endl;

      initial_kernel_index = yawToKernelIndex(forward_yaw);
      // cout << "initial kernel index: " << initial_kernel_index << endl;
      // Try forward-facing yaw first
      bool point_is_free = discretizedCollisionCheck<true>(
          discrete_path[i - 1], curr_pt, initial_kernel_index,
          final_kernel_index, 2);

      if (point_is_free) {
        // cout << "initial kernel index check success, point is free" << endl;
        // cout << "final stored kernel index: " << final_kernel_index << endl;
        // cout << "==============================" << endl;
        current_path.emplace_back(curr_pt, final_kernel_index, true);
        i++;
        continue;
      }

      // If forward yaw fails, try full collision check with previous yaw
      if (!current_path.empty()) {
        initial_kernel_index = current_path.back().kernel_index;
        // cout << "initial kernel index check failed, trying previous kernel
        // index: " << initial_kernel_index << endl;
        point_is_free = discretizedCollisionCheck<true>(
            discrete_path[i - 1], curr_pt, initial_kernel_index,
            final_kernel_index);

        if (point_is_free) {
          // cout << "previous kernel index check success, point is free" <<
          // endl; cout << "final stored kernel index: " << final_kernel_index
          // << endl; cout << "==============================" << endl;
          current_path.emplace_back(curr_pt, final_kernel_index, true);
          i++;
          continue;
        }
      }

      // If both attempts fail, handle collision
      path_has_collision = true;
      std::cout << "\n[COLLISION Found at] Point " << i << "/"
                << discrete_path.size() - 1 << " at " << formatPoint(curr_pt)
                << std::endl;

      // Rest of collision handling remains the same
      bool found_neighbors = false;
      size_t prev_waypoint = 0, next_waypoint = 0;
      found_neighbors = findNeighborWaypoints(curr_pt, raw_path, prev_waypoint,
                                              next_waypoint);

      if (!found_neighbors) {
        std::cout
            << "  ⚠️ Failed to find neighboring waypoints for collision point"
            << std::endl;
        current_path.emplace_back(curr_pt, initial_kernel_index, false);
        i++;
        continue;
      }

      if (failed_segments.count({prev_waypoint, next_waypoint})) {
        current_path.emplace_back(curr_pt, initial_kernel_index, false);
        i++;
        continue;
      }

      // PushAway, validated against occupancy, robot shape, and line of
      // sight back to the last accepted point and forward to the next
      // discretized point (mirrors the topology's shortcutPath_fastPlanner --
      // the LOS target must be a nearby point, not the far raw waypoint,
      // since an obstacle sitting between here and that waypoint is exactly
      // why this collision exists and would make the LOS check unsatisfiable).
      const Eigen::Vector2d &los_target =
          (i + 1 < discrete_path.size()) ? discrete_path[i + 1] : curr_pt;
      Eigen::Vector2d new_pushed_point;
      bool push_ok = _topo_path_finder->searchValidatedPushPoint(
          curr_pt, forward_yaw, discrete_path[i - 1], los_target,
          new_pushed_point, final_kernel_index);

      if (push_ok) {
        std::cout << "    ✓ Push successful - point is now collision-free "
                     "with kernel index: "
                  << final_kernel_index << std::endl;

        vector<PathPoint> verified_path;

        bool verify_ok = verifyAndFixConnection_robot_running(
            new_pushed_point, final_kernel_index, raw_path, prev_waypoint,
            next_waypoint, verified_path);

        // Halving fallback: the full-segment verification is all-or-nothing,
        // so one unresolvable spot anywhere in the segment (typically its
        // far end, e.g. a tight corner) discards meters of repair that
        // succeeded -- everything behind the stuck spot goes red. Retry
        // toward a pushed virtual waypoint partway between the collision
        // and the segment end, halving the span until the repair fits.
        // Success splices the salvaged part and inserts the virtual point
        // as a real waypoint; the stuck remainder is handled (and possibly
        // marked high-risk) on later iterations, isolated to its true size.
        if (!verify_ok) {
          const Eigen::Vector2d seg_a = raw_path[prev_waypoint];
          const Eigen::Vector2d seg_b = raw_path[next_waypoint];
          const double seg_len_full = (seg_b - seg_a).norm();
          if (seg_len_full > 1e-6) {
            double t_coll = (curr_pt - seg_a).dot(seg_b - seg_a) /
                            (seg_b - seg_a).squaredNorm();
            t_coll = std::min(std::max(t_coll, 0.0), 1.0);
            const Eigen::Vector2d dir = (seg_b - seg_a) / seg_len_full;
            const double seg_yaw = atan2(dir.y(), dir.x());
            const double min_len = 6.0 * _resolution;
            double frac = 0.5;
            while (!verify_ok &&
                   (1.0 - t_coll) * frac * seg_len_full > min_len) {
              const double t_v = t_coll + (1.0 - t_coll) * frac;
              const Eigen::Vector2d mid = seg_a + t_v * (seg_b - seg_a);
              Eigen::Vector2d virtual_wp;
              int virtual_kernel;
              // The midpoint sits on the colliding line, so it must be
              // pushed free before it can serve as a segment endpoint.
              if (_topo_path_finder->searchValidatedPushPoint(
                      mid, seg_yaw, mid - _resolution * dir,
                      mid + _resolution * dir, virtual_wp, virtual_kernel)) {
                raw_path.insert(raw_path.begin() + next_waypoint, virtual_wp);
                if (verifyAndFixConnection_robot_running(
                        new_pushed_point, final_kernel_index, raw_path,
                        prev_waypoint, next_waypoint, verified_path)) {
                  std::cout << "  ↳ Partial repair: segment split at virtual "
                               "waypoint "
                            << formatPoint(virtual_wp)
                            << ", remainder handled separately" << std::endl;
                  verify_ok = true;
                  break;
                }
                raw_path.erase(raw_path.begin() + next_waypoint);
              }
              frac *= 0.5;
            }
          }
        }

        if (verify_ok) {
          // Reconnection target: forcing the repaired chain back through
          // raw_path[next_waypoint] leaves a kink when the pushed points
          // have shifted the corridor sideways -- the chain detours to a
          // waypoint the obstacle no longer requires. Try to connect the
          // chain tail directly to a later waypoint instead; the straight
          // connection is never longer than the detour through the skipped
          // waypoints, so it only needs to be footprint-free and stay in
          // the same homotopy class.
          size_t reconnect_waypoint = next_waypoint;
          {
            const size_t max_skip = 3;
            const size_t cand_end =
                std::min(next_waypoint + max_skip, raw_path.size() - 1);
            // The connection may start from ANY point of the repaired chain
            // (earliest first), not only its tail: a pushed point only
            // exists to clear the collision at its own sample, so once a
            // direct free connection bypasses it, it is a detour and gets
            // dropped along with the skipped waypoints. Farthest candidate
            // first, earliest chain point first = maximum removal.
            bool done = false;
            for (size_t cand = cand_end; cand > next_waypoint && !done;
                 --cand) {
              for (size_t s = 0; s <= verified_path.size() && !done; ++s) {
                const Eigen::Vector2d chain_pt = (s == 0)
                                                     ? raw_path[prev_waypoint]
                                                     : verified_path[s - 1].pos;
                vector<PathPoint> checked_connection;
                if (!segmentIsFree(chain_pt, raw_path[cand], &checked_connection)) {
                  continue;
                }
                vector<Eigen::Vector2d> detour;
                detour.push_back(chain_pt);
                for (size_t t = s; t < verified_path.size(); ++t) {
                  detour.push_back(verified_path[t].pos);
                }
                for (size_t t = next_waypoint; t <= cand; ++t) {
                  detour.push_back(raw_path[t]);
                }
                vector<Eigen::Vector2d> direct{chain_pt, raw_path[cand]};
                if (!_topo_path_finder->sameTopoPath(detour, direct)) {
                  continue;
                }
                std::cout << "    ↷ Reconnected from chain point " << s << "/"
                          << verified_path.size() << " past waypoint "
                          << next_waypoint << " directly to waypoint " << cand
                          << std::endl;
                verified_path.resize(s);
                verified_path.insert(verified_path.end(), checked_connection.begin(),
                                     checked_connection.end());
                reconnect_waypoint = cand;
                done = true;
              }
            }
          }

          // Capture the reconnect waypoint's position before raw_path is
          // mutated below.
          Eigen::Vector2d next_waypoint_pos = raw_path[reconnect_waypoint];
          size_t reconnect_idx = i;
          double reconnect_distance = std::numeric_limits<double>::max();
          for (size_t k = i; k < discrete_path.size(); ++k) {
            const double distance = (discrete_path[k] - next_waypoint_pos).norm();
            if (distance < reconnect_distance) {
              reconnect_distance = distance;
              reconnect_idx = k;
            }
          }
          const vector<Eigen::Vector2d> unchecked_tail(
              discrete_path.begin() + reconnect_idx + 1, discrete_path.end());

          // Drop the waypoints the reconnection skips (no-op when
          // reconnect_waypoint == next_waypoint == prev_waypoint + 1), then
          // splice every configuration the verification step needed (the
          // pushed point plus any further recursive splits) into the raw
          // path, in path order, so the fix is not silently dropped.
          raw_path.erase(raw_path.begin() + prev_waypoint + 1,
                         raw_path.begin() + reconnect_waypoint);
          for (size_t k = 0; k + 1 < verified_path.size(); k++) {
            raw_path.insert(raw_path.begin() + prev_waypoint + 1 + k,
                            verified_path[k].pos);
          }

          // verified_path reconstructs the segment fresh from
          // raw_path[prev_waypoint] onward, so current_path must be
          // truncated back to that same waypoint -- not to i, which is
          // partway between prev_waypoint and the collision -- or the
          // splice doubles back on itself instead of connecting cleanly.
          Eigen::Vector2d prev_waypoint_pos = raw_path[prev_waypoint];
          auto prev_it =
              std::find_if(current_path.rbegin(), current_path.rend(),
                           [&prev_waypoint_pos](const PathPoint &p) {
                             return (p.pos - prev_waypoint_pos).norm() < 1e-6;
                           });
          if (prev_it != current_path.rend()) {
            current_path.resize(
                std::distance(current_path.begin(), prev_it.base()));
          } else {
            std::cout << "  ⚠️ prev_waypoint not found in current_path, "
                         "falling back to truncating at the collision point"
                      << std::endl;
            current_path.resize(i);
          }
          for (const auto &vp : verified_path) {
            current_path.push_back(vp);
          }
          final_push_success = true;

          // The splice renumbered raw_path, so cached failure indices no
          // longer refer to the same segments.
          failed_segments.clear();

          // Reuse the checked prefix and repair samples. Only the untouched
          // suffix remains to be checked; the reconnect endpoint is included
          // in verified_path and must not be processed a second time.
          discrete_path.clear();
          for (const auto &point : current_path) discrete_path.push_back(point.pos);
          i = discrete_path.size();
          discrete_path.insert(discrete_path.end(), unchecked_tail.begin(), unchecked_tail.end());
        }
      }

      if (!final_push_success) {
        std::cout << "  ❌ All push attempts failed for point " << i
                  << std::endl;
        failed_segments.insert({prev_waypoint, next_waypoint});
        current_path.emplace_back(curr_pt, initial_kernel_index, false);
        i++;
      }
    }

    // Final pass over the assembled path: drop connector points (original
    // waypoints and pushed points) that a direct footprint-free, same-
    // topology connection makes unnecessary.
    globalShortcutPath(current_path);

    // Stamp which topology path every point in this run came from.
    for (auto &point : current_path) {
      point.path_id = path_idx;
    }

    for (const auto &point : current_path) {
      cout << "point: " << point.pos.transpose()
           << " is free: " << point.is_collision_free << endl;
      cout << "kernel index: " << point.kernel_index << endl;
    }

    // Counts may differ after geometry-changing shortcuts; repaired and
    // unchanged sections both retain their checked samples and yaw kernels.
    size_t unresolved_count =
        std::count_if(current_path.begin(), current_path.end(),
                      [](const PathPoint &p) { return !p.is_collision_free; });
    std::cout << "[PATH " << path_idx + 1
              << " SUMMARY] discretized points: " << discrete_path.size()
              << ", points in output: " << current_path.size()
              << ", unresolved (red): " << unresolved_count << std::endl;

    paths_out.push_back(current_path);

    ros::Time t2 = ros::Time::now();
    double duration = (t2 - t1).toSec() * 1000;
    std::cout << "SE2 path processing duration in our method: " << duration
              << " ms" << std::endl;

    // Visualization data collection remains the same
    vector<bool> vis_status;
    for (const auto &point : current_path) {
      visCollisionPoints.push_back(point.toVector3d());
      vis_status.push_back(point.is_collision_free);
    }
    visCollisionStatus.push_back(vis_status);

    vector<Eigen::Vector2d> path_2d;
    for (const auto &point : current_path) {
      path_2d.push_back(point.pos);
    }
    new_path_2d.push_back(path_2d);

    // Discretized grid points for this path, colored red if occupied
    // (unresolved) or high-risk, green otherwise.
    vector<Eigen::Vector3d> grid_points;
    vector<bool> grid_is_red;
    for (const auto &point : current_path) {
      grid_points.push_back(point.toVector3d());
      grid_is_red.push_back(!point.is_collision_free || point.is_high_risk);
    }
    grid_points_per_path.push_back(grid_points);
    grid_is_red_per_path.push_back(grid_is_red);
  }

  // Visualization calls remain the same
  _visPtr_back_end->visPathNode("path_vis", visCollisionPoints,
                                visCollisionStatus, _resolution);
  _visPtr_front_end->drawFastCheckPath(new_path_2d, _resolution / 10);
  _visPtr_front_end->drawFastCheckGrid(grid_points_per_path,
                                       grid_is_red_per_path, _resolution);

  std::cout << "\n=== Fast Check Complete ===" << std::endl;
  std::cout << "Paths processed: " << paths_out.size() << std::endl;
}

// Classifies a single path's points into low-risk (R^2 optimization
// suffices) vs high-risk (needs full SVSDF SE(2) optimization) regions.
//
// A point is high-risk if fast_check left it unresolved (is_collision_free
// == false), plus a segTolerance-point buffer on either side. Any low-risk
// gap shorter than MIN_FREE_SEGMENT_POINTS that borders a high-risk region
// isn't worth its own optimization segment, so it's absorbed into that
// region too.
//
// Returns true if the whole path ends up low-risk (segments holds a single
// entry: the whole path). Otherwise segments holds the alternating
// low/high-risk stretches in path order; every boundary point is duplicated
// across the split so adjacent segments share an exact handoff
// configuration (same x, y, yaw) for downstream trajectory continuity.
bool PlannerStrategy::classifySinglePathRisk(
    vector<PathPoint> &path, vector<vector<PathPoint>> &segments) {
  segments.clear();
  if (path.empty()) {
    return true;
  }

  for (size_t i = 0; i < path.size(); i++) {
    if (path[i].is_collision_free) {
      continue;
    }
    size_t lo = (i >= (size_t)segTolerance) ? i - segTolerance : 0;
    size_t hi = std::min(path.size() - 1, i + (size_t)segTolerance);
    for (size_t j = lo; j <= hi; j++) {
      path[j].is_high_risk = true;
    }
  }

  size_t i = 0;
  while (i < path.size()) {
    if (path[i].is_high_risk) {
      i++;
      continue;
    }
    size_t run_start = i;
    while (i < path.size() && !path[i].is_high_risk) {
      i++;
    }
    bool touches_high_risk = run_start > 0 || i < path.size();
    if (touches_high_risk && i - run_start < (size_t)MIN_FREE_SEGMENT_POINTS) {
      for (size_t j = run_start; j < i; j++) {
        path[j].is_high_risk = true;
      }
    }
  }

  bool fully_low_risk =
      std::none_of(path.begin(), path.end(),
                   [](const PathPoint &p) { return p.is_high_risk; });
  if (fully_low_risk) {
    segments.push_back(path);
    return true;
  }

  size_t start = 0;
  while (start < path.size()) {
    size_t end = start;
    while (end + 1 < path.size() &&
           path[end + 1].is_high_risk == path[start].is_high_risk) {
      end++;
    }
    vector<PathPoint> segment(path.begin() + start, path.begin() + end + 1);
    if (!segments.empty()) {
      segment.insert(segment.begin(), segments.back().back());
    }
    segments.push_back(segment);
    start = end + 1;
  }

  return false;
}

void PlannerStrategy::classifyPathsRisk(
    vector<vector<PathPoint>> &paths, vector<bool> &path_is_fully_low_risk,
    vector<vector<vector<PathPoint>>> &path_segments) {
  path_is_fully_low_risk.clear();
  path_segments.clear();

  for (auto &path : paths) {
    vector<vector<PathPoint>> segments;
    bool fully_low_risk = classifySinglePathRisk(path, segments);
    path_is_fully_low_risk.push_back(fully_low_risk);
    path_segments.push_back(segments);
  }
}

// ===================== risk-aware trajectory back end =====================
//
// Every candidate starts as one whole-path R2 trajectory. Risk labels do not
// introduce trajectory boundaries or zero-velocity constraints. Only pieces
// found to collide by the swept-volume check are repaired in SE2.

// Junction-by-junction continuity report for a finished trajectory. MINCO
// pins each piece's boundary state, so a jump here means the splice itself is
// wrong -- pieces written back with mismatched boundary conditions, or two
// independently optimized stretches concatenated at different endpoints.
void PlannerStrategy::reportTrajectoryContinuity(
    const Trajectory<TRAJ_ORDER> &traj) {
  int reported = 0;
  for (int i = 0; i + 1 < traj.getPieceNum(); ++i) {
    const double dur = traj[i].getDuration();
    const Eigen::Vector3d end_state = traj[i].getPos(dur);
    const Eigen::Vector3d start_state = traj[i + 1].getPos(0.0);

    const double pos_gap = (end_state.head(2) - start_state.head(2)).norm();
    const double yaw_gap = std::abs(end_state(2) - start_state(2));
    const double vel_gap =
        (traj[i].getVel(dur) - traj[i + 1].getVel(0.0)).norm();

    if (pos_gap > 1e-3 || yaw_gap > 1e-2 || vel_gap > 1e-2) {
      ROS_ERROR("Junction %d/%d: position jump %.4f m, yaw jump %.4f rad, "
                "velocity jump %.4f m/s",
                i, traj.getPieceNum() - 1, pos_gap, yaw_gap, vel_gap);
      reported++;
    }
  }
  if (reported == 0) {
    std::cout << "Trajectory is continuous across all "
              << traj.getPieceNum() - 1 << " junctions" << std::endl;
  }
}

double PlannerStrategy::pathLength(const vector<PathPoint> &path) {
  double len = 0.0;
  for (size_t i = 1; i < path.size(); i++) {
    len += (path[i].pos - path[i - 1].pos).norm();
  }
  return len;
}

vector<Eigen::Vector3d>
PlannerStrategy::toVector3dPath(const vector<PathPoint> &path) {
  vector<Eigen::Vector3d> pts;
  pts.reserve(path.size());
  for (const auto &p : path) {
    Eigen::Vector3d pt = p.toVector3d();
    // PathPoint yaw is wrapped to [-pi, pi), but MINCO treats yaw as a plain
    // coordinate. Unwrap it so neighbouring points never differ by ~2*pi,
    // which would make the robot spin a full turn between them.
    if (!pts.empty()) {
      double dyaw = pt.z() - pts.back().z();
      while (dyaw > M_PI)
        dyaw -= 2 * M_PI;
      while (dyaw < -M_PI)
        dyaw += 2 * M_PI;
      pt.z() = pts.back().z() + dyaw;
    }
    pts.push_back(pt);
  }
  return pts;
}

// Optimize the complete path, then repair colliding piece groups while
// retaining their original endpoint position, velocity and acceleration.
bool PlannerStrategy::optimizePathR2(const vector<PathPoint> &path,
                                     Trajectory<TRAJ_ORDER> &traj_out) {
  const vector<Eigen::Vector3d> pts = toVector3dPath(path);
  traj_out.clear();
  if (pts.size() < 2) {
    return false;
  }

  vector<vector<Eigen::Vector3d>> single_path{pts};
  std::vector<Trajectory<TRAJ_ORDER>> trajectories =
      generateTrajR2(single_path);
  if (trajectories.empty() || trajectories.front().getPieceNum() == 0) {
    ROS_ERROR("[R2] optimizer failed on a %zu sample path", pts.size());
    return false;
  }

  Trajectory<TRAJ_ORDER> traj = trajectories.front();
  std::vector<double> collision_time_stamps;
  geneAABBPointsTrajectoryR2(traj);

  if (trajSafe(traj, collision_time_stamps, true)) {
    traj_out = traj;
    return true;
  }

  ROS_WARN("[R2] path collides at %zu time stamps, repairing those pieces "
           "in SE(2)",
           collision_time_stamps.size());
  if (!trajOptSVSDF(collision_time_stamps, traj)) {
    return false;
  }

  // Validate the complete splice, including unchanged pieces and junctions,
  // using obstacle points gathered around the final trajectory.
  geneAABBPointsTrajectoryR2(traj);
  collision_time_stamps.clear();
  if (!trajSafe(traj, collision_time_stamps, false)) {
    ROS_ERROR("[R2] complete trajectory is still unsafe after SE2 repair");
    return false;
  }

  traj_out = traj;
  return true;
}

/**
 * @brief When a collision is detected, we push that specific point
Based on where the collision occurred (first or second half of path), we insert
the new middle point in the appropriate position Creates a new 4-point sub-path
that includes both the original pushed point and the new middle point Checks if
the entire new sub-path is collision free
 * @param pushed_point new pushed point
 * @param raw_path raw single path
 * @param prev_waypoint previous waypoint index of this new pushed point
 * @param next_waypoint next waypoint index of this new pushed point
 */

// Footprint-aware feasibility of the straight connection start_pt -> end_pt:
// discretizes it and requires a collision-free yaw kernel at every sample.
bool PlannerStrategy::segmentIsFree(const Eigen::Vector2d &start_pt,
                                    const Eigen::Vector2d &end_pt,
                                    vector<PathPoint> *checked_samples) {
  if (checked_samples) checked_samples->clear();
  auto discrete_path =
      _topo_path_finder->discretizePath_fastplanner({start_pt, end_pt});
  vector<PathPoint> checked;
  for (size_t i = 1; i < discrete_path.size(); i++) {
    int point_yaw_index = 0;
    int final_kernel_index;
    if (!discretizedCollisionCheck<false>(discrete_path[i - 1],
                                          discrete_path[i], point_yaw_index,
                                          final_kernel_index)) {
      return false;
    }
    checked.emplace_back(discrete_path[i], final_kernel_index, true);
  }
  if (checked_samples) checked_samples->swap(checked);
  return true;
}

// Shortcut pass over a repaired segment (start_pt -> pts... -> end_pt).
// SegAdjust only guarantees each push clears its own sample, so the chain it
// leaves behind can jog laterally cell-by-cell (staircase artifacts) even
// where a direct footprint-free connection between farther-apart points
// exists. Greedily keep the farthest point reachable by a free straight
// connection and drop everything in between.
void PlannerStrategy::pruneRepairedSegment(const Eigen::Vector2d &start_pt,
                                           const Eigen::Vector2d &end_pt,
                                           vector<PathPoint> &pts) {
  if (pts.size() < 2) {
    return;
  }

  // pts excludes start_pt and already includes end_pt.
  const size_t last = pts.size();
  auto posOf = [&](size_t idx) -> Eigen::Vector2d {
    return idx == 0 ? start_pt : pts[idx - 1].pos;
  };
  vector<PathPoint> pruned;
  size_t cur = 0;
  while (cur < last) {
    size_t nxt = cur + 1;
    vector<PathPoint> selected{pts[nxt - 1]};
    for (size_t j = last; j > cur + 1; --j) {
      vector<PathPoint> checked;
      if (segmentIsFree(posOf(cur), posOf(j), &checked) && !checked.empty()) {
        nxt = j;
        // Preserve the anchor's already validated adjusted yaw.
        checked.back() = pts[j - 1];
        selected.swap(checked);
        break;
      }
    }
    pruned.insert(pruned.end(), selected.begin(), selected.end());
    cur = nxt;
  }
  pts.swap(pruned);
}

// Final smoothing pass over one fully processed path: collapse the
// connector points (original waypoints and pushed points alike) that the
// finished geometry no longer needs. Per-repair reconnection cannot see
// these -- each repair only reasons about its own segment, so independently
// valid fixes can chain into a dogleg no single obstacle forces. Operates on
// the polyline's turning points only (dense samples on straight runs are
// collinear), keeps every connection footprint-
// free and homotopy-equivalent to what it replaces, and never shortcuts
// across unresolved (red) points, whose surroundings must reach the
// back-end untouched.
void PlannerStrategy::globalShortcutPath(vector<PathPoint> &path) {
  if (path.size() < 3) {
    return;
  }

  vector<PathPoint> result;
  size_t run_start = 0;
  while (run_start < path.size()) {
    if (!path[run_start].is_collision_free) {
      result.push_back(path[run_start]);
      run_start++;
      continue;
    }
    size_t run_end = run_start;
    while (run_end + 1 < path.size() && path[run_end + 1].is_collision_free) {
      run_end++;
    }

    if (run_end - run_start + 1 < 3) {
      for (size_t k = run_start; k <= run_end; ++k) {
        result.push_back(path[k]);
      }
      run_start = run_end + 1;
      continue;
    }

    // Turning points of this collision-free run.
    vector<size_t> verts;
    verts.push_back(run_start);
    for (size_t k = run_start + 1; k < run_end; ++k) {
      Eigen::Vector2d d1 = path[k].pos - path[k - 1].pos;
      Eigen::Vector2d d2 = path[k + 1].pos - path[k].pos;
      double cross = d1.x() * d2.y() - d1.y() * d2.x();
      if (std::abs(cross) > 1e-3 * d1.norm() * d2.norm()) {
        verts.push_back(k);
      }
    }
    verts.push_back(run_end);

    vector<PathPoint> rebuilt{path[run_start]};
    size_t vi = 0;
    while (vi + 1 < verts.size()) {
      size_t chosen = vi + 1;
      vector<PathPoint> selected;
      for (size_t vj = verts.size() - 1; vj > vi + 1; --vj) {
        vector<Eigen::Vector2d> detour;
        for (size_t t = vi; t <= vj; ++t) detour.push_back(path[verts[t]].pos);
        vector<Eigen::Vector2d> direct{path[verts[vi]].pos, path[verts[vj]].pos};
        if (!_topo_path_finder->sameTopoPath(detour, direct)) continue;
        vector<PathPoint> checked;
        if (!segmentIsFree(direct.front(), direct.back(), &checked) || checked.empty()) continue;
        checked.back() = path[verts[vj]];
        selected.swap(checked);
        chosen = vj;
        break;
      }
      if (selected.empty()) {
        // Unchanged geometry: keep its original checked yaw samples verbatim.
        selected.insert(selected.end(), path.begin() + verts[vi] + 1,
                        path.begin() + verts[chosen] + 1);
      }
      rebuilt.insert(rebuilt.end(), selected.begin(), selected.end());
      vi = chosen;
    }
    result.insert(result.end(), rebuilt.begin(), rebuilt.end());
    run_start = run_end + 1;
  }
  path.swap(result);
}

// Helper function to verify and fix connection
// Recursively verifies the segment start_pt -> end_pt: finds the first
// collision along it, steers it away with the validated PushAway search
// (searchValidatedPushPoint, shared with the topology's own shortcutting),
// then connects the pushed point directly to start_pt/end_pt and recurses on
// the two resulting child segments. If either child cannot be resolved, the
// whole segment is reported unresolved so the caller can discard the
// adjustment (mirrors SegAdjust in the paper).
//
// push_budget bounds the TOTAL number of pushed points across the whole
// recursion tree, not the tree depth: a segment running parallel to a wall
// needs one push per collision cluster in sequence, so a depth cap fails it
// even though every individual push succeeds. The natural terminator is
// PushAway itself running out of free space (as in the paper); the budget is
// only a runaway guard.
bool PlannerStrategy::checkAndAdjustSegment(
    const Eigen::Vector2d &start_pt, const Eigen::Vector2d &end_pt,
    int &push_budget, std::set<std::pair<int, int>> &pushed_cells,
    vector<PathPoint> &out_path) {
  out_path.clear();

  auto discrete_path =
      _topo_path_finder->discretizePath_fastplanner({start_pt, end_pt});
  if (discrete_path.size() < 2) {
    return true; // segment too short to discretize, nothing to check
  }

  vector<PathPoint> checked;
  size_t collision_idx = 0;
  bool found_collision = false;
  for (size_t i = 1; i < discrete_path.size(); i++) {
    int point_yaw_index = 0;
    int final_kernel_index;
    bool segment_is_free =
        discretizedCollisionCheck<false>(discrete_path[i - 1], discrete_path[i],
                                         point_yaw_index, final_kernel_index);
    if (!segment_is_free) {
      collision_idx = i;
      found_collision = true;
      break;
    }
    checked.emplace_back(discrete_path[i], final_kernel_index, true);
  }

  if (!found_collision) {
    out_path.swap(checked);
    return true;
  }

  if (push_budget <= 0) {
    std::cout << "  ⚠️ SegAdjust push budget exhausted" << std::endl;
    return false;
  }

  const Eigen::Vector2d collision_pt = discrete_path[collision_idx];
  const double colli_yaw =
      atan2(end_pt.y() - start_pt.y(), end_pt.x() - start_pt.x());

  // LOS target must be a nearby point, not end_pt itself: end_pt can be the
  // far boundary of the whole segment being repaired (it's inherited
  // unchanged down one whole side of the recursion), and an obstacle sitting
  // between here and that far point is exactly why this collision exists --
  // requiring LOS all the way there would make the search unsatisfiable.
  const Eigen::Vector2d &los_target = (collision_idx + 1 < discrete_path.size())
                                          ? discrete_path[collision_idx + 1]
                                          : end_pt;

  Eigen::Vector2d pushed_point;
  int pushed_kernel_index;
  // pushed_cells is handed to the search as an exclusion set: neighboring
  // samples legitimately want to push into the same grid cell (their ideal
  // candidates are one sample spacing apart, which quantizes to one cell),
  // and treating that as an oscillation killed repairs that were
  // converging. Excluding used cells makes neighbors take adjacent cells
  // and turns a true dead end into a clean "no candidate" failure.
  bool push_ok = _topo_path_finder->searchValidatedPushPoint(
      collision_pt, colli_yaw, discrete_path[collision_idx - 1], los_target,
      pushed_point, pushed_kernel_index, &pushed_cells);
  if (!push_ok) {
    return false;
  }
  --push_budget;

  const Eigen::Vector2i pushed_cell =
      pcsmap_manager->occupancy_map->getGridIndex2d(pushed_point);
  pushed_cells.insert({pushed_cell.x(), pushed_cell.y()});

  // Connect the pushed point directly to its two immediate neighbors and
  // recheck each new sub-segment independently.
  vector<PathPoint> left_path, right_path;
  if (!checkAndAdjustSegment(start_pt, pushed_point, push_budget, pushed_cells,
                             left_path)) {
    return false;
  }
  if (!checkAndAdjustSegment(pushed_point, end_pt, push_budget, pushed_cells,
                             right_path)) {
    return false;
  }

  out_path = left_path;
  if (!out_path.empty()) {
    out_path.back() = PathPoint(pushed_point, pushed_kernel_index, true);
  }
  out_path.insert(out_path.end(), right_path.begin(), right_path.end());
  return true;
}

bool PlannerStrategy::verifyAndFixConnection_robot_running(
    const Eigen::Vector2d &pushed_point, int pushed_kernel_index,
    const vector<Eigen::Vector2d> &raw_path, size_t prev_waypoint,
    size_t next_waypoint, vector<PathPoint> &verified_path) {
  verified_path.clear();
  std::cout << "\n[VERIFY CONNECTION] Starting verification between waypoints "
            << prev_waypoint << " and " << next_waypoint << std::endl;

  // Total pushed points allowed across both sides, scaled to the segment
  // length: a wall-parallel segment legitimately needs on the order of one
  // push per discretized sample, so a small fixed cap would fail repairs
  // that are actually converging (the real terminator is PushAway running
  // out of free space).
  const double seg_len =
      (raw_path[next_waypoint] - raw_path[prev_waypoint]).norm();
  int push_budget =
      std::max(32, static_cast<int>(std::ceil(4.0 * seg_len / _resolution)));

  // Cells this repair has already pushed a point onto, for oscillation
  // detection across the whole recursion (both sides share it, as they
  // share the budget). Seeded with the initial pushed point: a recursion
  // push landing back on it is the same cycle.
  std::set<std::pair<int, int>> pushed_cells;
  const Eigen::Vector2i seed_cell =
      pcsmap_manager->occupancy_map->getGridIndex2d(pushed_point);
  pushed_cells.insert({seed_cell.x(), seed_cell.y()});

  vector<PathPoint> left_path, right_path;
  if (!checkAndAdjustSegment(raw_path[prev_waypoint], pushed_point, push_budget,
                             pushed_cells, left_path)) {
    std::cout << "  ⚠️ Left sub-segment could not be resolved" << std::endl;
    return false;
  }
  if (!checkAndAdjustSegment(pushed_point, raw_path[next_waypoint], push_budget,
                             pushed_cells, right_path)) {
    std::cout << "  ⚠️ Right sub-segment could not be resolved" << std::endl;
    return false;
  }

  verified_path = left_path;
  if (!verified_path.empty()) {
    verified_path.back() = PathPoint(pushed_point, pushed_kernel_index, true);
  }
  verified_path.insert(verified_path.end(), right_path.begin(),
                       right_path.end());

  // Remove staircase detours while retaining the checked samples of each
  // replacement connection. Output excludes start and includes end.
  pruneRepairedSegment(raw_path[prev_waypoint], raw_path[next_waypoint],
                       verified_path);
  return true;
}
