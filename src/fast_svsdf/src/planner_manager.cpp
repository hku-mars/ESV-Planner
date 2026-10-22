#include "fast_svsdf/planner_manager.hpp"
#include <algorithm>
#include <numeric>

void PlannerManager::init(ros::NodeHandle &nh, Config &conf) {
  config = conf;

  _resolution = config.occupancy_resolution;

  // map pointer initialization
  pcsmap_manager.reset(new PCSmapManager(config));
  pcsmap_manager->init(nh);

  // swept volume manager initialization
  sv_manager.reset(new SweptVolumeManager(config));
  sv_manager->init(nh, config);

  // topo path finder initialization
  _topo_path_finder.reset(new TopoPath());
  _topo_path_finder->init(nh, config);
  _topo_path_finder->setMapManager(pcsmap_manager, sv_manager);

  // visualization pointer initialization front(topo path search) and
  // back(trajectory optimization)
  _visPtr_front_end.reset(new PlanningVisualization(nh));
  _visPtr_back_end.reset(new Visualization(nh));

  // R2 trajectory optimization initialization
  ori_traj_generator.reset(new OriTraj);
  ori_traj_generator->setParam(nh, config);

  // SE2 trajectory optimization initialization
  minco_traj_optimizer.reset(new TrajOptimizer);
  minco_traj_optimizer->setParam(nh, config);
  minco_traj_optimizer->setEnvironment(sv_manager);

  // trajectory slicing strategy initialization
  planner_strategy.reset(new PlannerStrategy);
  planner_strategy->init(nh, config);
  planner_strategy->setEnvironment(
      _topo_path_finder, pcsmap_manager, minco_traj_optimizer,
      ori_traj_generator, _visPtr_front_end, _visPtr_back_end, sv_manager);

  rcvmap_signal_pub =
      nh.subscribe("/rcvmap_signal", 1, &PlannerManager::mapRcvCallBack, this);
  trajectory_pub_ =
      nh.advertise<traj_utils::PolyTraj>("/planning/trajectory", 10);
  // heartbet_pub_ = nh.advertise<std_msgs::Empty>("/planning/heartbeat", 1);
}

PlannerManager::PlannerManager() {
  // Constructor definition
}

PlannerManager::~PlannerManager() {
  // Destructor definition
}

void PlannerManager::mapRcvCallBack(const std_msgs::Empty &msg) {
  uint8_t *map_kernel = pcsmap_manager->generateMapKernel2D(config.kernel_size);

  int sizeX = pcsmap_manager->occupancy_map->X_size;
  int sizeY = pcsmap_manager->occupancy_map->Y_size;
  int sizeZ = pcsmap_manager->occupancy_map->Z_size;
  sv_manager->setMapKernel(map_kernel, sizeX, sizeY, sizeZ);

  minco_traj_optimizer->setGridMap(pcsmap_manager);
  map_ready_ = true;
}

// final state machine
void PlannerManager::systemFSM(Vector2d start, Vector2d end) {
  bool play_animation = false;

  // Candidates from the previous goal must not survive into this plan --
  // trajSelectHandler picks by duration, so a shorter trajectory left over
  // from an earlier goal would win and the robot would animate to the wrong
  // destination.
  backup_trajectories.clear();
  final_trajectory.clear();
  minco_traj_optimizer->clearvisAABBpoints();

  // Clear RViz results even if this run finds no path or needs no SE2 repair.
  const std::vector<Trajectory<TRAJ_ORDER>> empty_trajectories;
  _visPtr_back_end->visMultipleTraj("init_traj/R2", empty_trajectories);
  _visPtr_back_end->visMultipleTraj("opt_traj/R2", empty_trajectories);
  _visPtr_back_end->visMultipleTraj("final_traj/R2", empty_trajectories);
  _visPtr_back_end->visMultipleTraj("final_traj/SE2", empty_trajectories);

  //   std::string filePath = ros::package::getPath("grid_path_searcher") +
  //                          "/data/trajectories/trajectory.txt";
  //   TrajectoryIO<5>::loadTrajectory(final_trajectory,
  //                                   filePath);
  //   backup_trajectories.push_back(final_trajectory);
  //   find topo paths
  vector<vector<Eigen::Vector2d>> raw_paths = generateTopoPath(start, end);

  // fast check and adjust collision path points
  vector<vector<PathPoint>> fast_check_paths;
  planner_strategy->fast_check(raw_paths, fast_check_paths);

  vector<bool> path_is_fully_low_risk;
  vector<vector<vector<PathPoint>>> path_segments;
  //
  planner_strategy->classifyPathsRisk(fast_check_paths, path_is_fully_low_risk,
                                      path_segments);

  riskAwarePathHandler(fast_check_paths, path_is_fully_low_risk, path_segments);

  cout << "final validate trajectory has: " << backup_trajectories.size()
       << " trajectories" << endl;

  // TrajectoryIO<5>::loadTrajectory(final_trajectory);
  // backup_trajectories.push_back(final_trajectory);

  if (backup_trajectories.size() > 1) {
    final_trajectory = trajSelectHandler();
    // for real robot controller
    publishTrajectory(trajectory_pub_, final_trajectory);
    play_animation = true;
  } else if (backup_trajectories.size() == 0) {
    ROS_ERROR("No valid trajectory found");
    return;
  } else if (backup_trajectories.size() == 1) {
    final_trajectory = backup_trajectories[0];
    publishTrajectory(trajectory_pub_, final_trajectory);
    play_animation = true;
  }

  // The selected trajectory must actually land on the requested goal. A gap
  // here means a stretch was lost in the splice, not that the animation
  // stopped early.
  if (play_animation) {
    const Eigen::Vector3d traj_end =
        final_trajectory.getPos(final_trajectory.getTotalDuration());
    const double goal_gap = (traj_end.head(2) - end).norm();
    if (goal_gap > 2.0 * _resolution) {
      ROS_ERROR("Selected trajectory ends %.3f m short of the goal (ends at "
                "%.3f, %.3f, goal is %.3f, %.3f)",
                goal_gap, traj_end.x(), traj_end.y(), end.x(), end.y());
    } else {
      std::cout << "Selected trajectory reaches the goal (gap " << goal_gap
                << " m)" << std::endl;
    }
  }

  if (play_animation) {
    for (int i = 0; i < 2; i++) {
      planner_strategy->sv_manager->visualizeTrajectoryStates(final_trajectory);
    }
    double length = TrajectoryLength<5>::getLength(final_trajectory);
    cout << "Total trajectory length is: " << length << endl;

    if (final_trajectory.getTotalDuration() > 0.1) {
      // Start / goal meshes, then drive the robot between them in real time.

      planner_strategy->playTrajectoryAnimation(final_trajectory);
    }
  }
}

Trajectory<TRAJ_ORDER> PlannerManager::trajSelectHandler() {
  Trajectory<TRAJ_ORDER> selected_trajectory;
  if (backup_trajectories.empty()) {
    std::cerr << "No trajectories to select from" << std::endl;
    return selected_trajectory;
  }

  // Initialize with the first trajectory
  size_t best_index = 0;
  double min_duration = backup_trajectories[0].getTotalDuration();

  // Iterate through all trajectories to find the one with minimum duration
  for (size_t i = 1; i < backup_trajectories.size(); ++i) {
    double current_duration = backup_trajectories[i].getTotalDuration();
    if (current_duration < min_duration) {
      min_duration = current_duration;
      best_index = i;
    }
  }

  // Select the best trajectory
  selected_trajectory = backup_trajectories[best_index];

  std::cout << "Selected trajectory index: " << best_index
            << " with total duration: " << min_duration << std::endl;
  return selected_trajectory;
}

// Risk-aware candidate loop. Candidates are ranked by their post-fast_check
// length and optimized as whole R2 trajectories, regardless of risk labels.
// Colliding pieces are repaired in SE2; unless stop_at_first_success is
// cleared, the first safe candidate ends the search.
void PlannerManager::riskAwarePathHandler(
    const vector<vector<PathPoint>> &paths,
    const vector<bool> &path_is_fully_low_risk,
    const vector<vector<vector<PathPoint>>> &path_segments) {
  if (paths.empty()) {
    ROS_WARN("No candidate paths survived fast check");
    return;
  }
  if (paths.size() != path_is_fully_low_risk.size() ||
      paths.size() != path_segments.size()) {
    ROS_ERROR("Risk classification does not match the candidate paths");
    return;
  }

  // fast_check reshapes the raw paths (pushes, spliced waypoints, shortcuts),
  // so the topo front end's length ordering no longer describes the geometry
  // we are about to optimize -- rank by the repaired length instead.
  vector<size_t> order(paths.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&paths](size_t a, size_t b) {
    return PlannerStrategy::pathLength(paths[a]) <
           PlannerStrategy::pathLength(paths[b]);
  });

  for (size_t rank = 0; rank < order.size(); ++rank) {
    const size_t idx = order[rank];
    const bool low_risk = path_is_fully_low_risk[idx];

    std::cout << "\n=== Candidate " << rank + 1 << "/" << order.size()
              << " (path " << idx << ", length "
              << PlannerStrategy::pathLength(paths[idx]) << ", "
              << (low_risk ? "fully low risk" : "mixed risk")
              << ") ===" << std::endl;

    minco_traj_optimizer->clearvisAABBpoints();
    Trajectory<TRAJ_ORDER> trajectory;
    const bool ok = planner_strategy->optimizePathR2(paths[idx], trajectory);

    if (!ok || trajectory.getPieceNum() == 0) {
      ROS_WARN("Candidate %zu dropped, moving to the next one", idx);
      continue;
    }

    std::cout << "Candidate " << idx << " accepted, duration "
              << trajectory.getTotalDuration() << " s, length "
              << TrajectoryLength<5>::getLength(trajectory) << " m"
              << std::endl;
    planner_strategy->reportTrajectoryContinuity(trajectory);
    backup_trajectories.push_back(trajectory);

    if (stop_at_first_success) {
      break;
    }
  }

  if (backup_trajectories.empty()) {
    ROS_ERROR("Every candidate path was dropped");
  }
}

vector<vector<Eigen::Vector2d>> PlannerManager::generateTopoPath(Vector2d start,
                                                                 Vector2d end) {
  list<GraphNode::Ptr> graph;
  vector<vector<Eigen::Vector2d>> raw_paths_test, short_paths_test,
      filtered_paths_test, select_paths_test;
  vector<PathNode::Ptr> raw_paths, short_paths, filtered_paths, select_paths;

  _topo_path_finder->findTopoPaths(start, end, graph, raw_paths, short_paths,
                                   filtered_paths, select_paths);

  for (const auto &pathNode : raw_paths) {
    raw_paths_test.push_back(pathNode->path);
  }

  for (const auto &pathNode : short_paths) {
    short_paths_test.push_back(pathNode->path);
  }

  for (const auto &pathNode : filtered_paths) {
    filtered_paths_test.push_back(pathNode->path);
  }

  for (const auto &pathNode : select_paths) {
    select_paths_test.push_back(pathNode->path);
  }

  for (const auto &path : raw_paths) {
    std::cout << "The raw path ID in demo node is: " << path->pathID
              << std::endl;
    for (const auto &pt : path->path) {
      std::cout << "x: " << pt.x() << " y: " << pt.y() << std::endl;
    }
  }

  for (const auto &path : short_paths) {
    std::cout << "The short_paths path ID in demo node is: " << path->pathID
              << std::endl;
    for (const auto &pt : path->path) {
      std::cout << "x: " << pt.x() << " y: " << pt.y() << std::endl;
    }
  }

  _visPtr_front_end->drawTopoPathsPhase1(raw_paths_test, _resolution / 10);

  _visPtr_front_end->drawTopoPathsPhase2(short_paths_test, _resolution / 10);

  //   _visPtr_front_end->drawTopoPathsPhase1(select_paths_test, _resolution /
  //   10);
  return select_paths_test;
}

// Single-frame renderer, meant to be ticked from a timer: it draws whatever
// pose the elapsed time since the last setTrajStamp corresponds to. The
// blocking start-to-goal playback lives in
// PlannerStrategy::playTrajectoryAnimation.
void PlannerManager::animationPlay() {
  if (final_trajectory.getTotalDuration() <= 0.1) {
    return;
  }
  planner_strategy->sv_manager->process(final_trajectory);
}

void PlannerManager::publishTrajectory(const ros::Publisher &pub,
                                       const Trajectory<5> &trajectory,
                                       int drone_id, int traj_id) {
  traj_utils::PolyTraj msg;

  msg.drone_id = drone_id;
  msg.traj_id = traj_id;
  msg.start_time = ros::Time::now();
  msg.order = 5;

  int piece_nums = trajectory.getPieceNum();

  for (int i = 0; i < piece_nums; ++i) {
    const auto &piece = trajectory[i];
    const auto &coeffMat = piece.getCoeffMat();

    for (int j = 0; j < 6; ++j) // Order is 5, so we have 6 coefficients
    {
      msg.coef_x.push_back(coeffMat(0, j));
      msg.coef_y.push_back(coeffMat(1, j));
      msg.coef_z.push_back(coeffMat(2, j));
    }

    msg.duration.push_back(piece.getDuration());
  }

  pub.publish(msg);
}
// test purpose
void PlannerManager::getGoal(Vector2d end) {
  Vector2i end_idx = (pcsmap_manager->occupancy_map)->getGridIndex2d(end);
  Vector2d end_pt =
      (pcsmap_manager->occupancy_map)->getGridCubeCenter2d(end_idx);
  ROS_WARN("The end point is: %.1f, %.1f", end_pt.x(), end_pt.y());
  _visPtr_front_end->drawGoal(end_pt, 0.2, Vector4d(1.0, 0.0, 0.0, 1), 1);
}
