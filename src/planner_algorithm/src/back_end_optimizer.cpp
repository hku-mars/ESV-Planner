#include <planner_algorithm/back_end_optimizer.hpp>

int TrajOptimizer::optimize_traj_lmbm(const Eigen::Matrix3d &initS,
                                      const Eigen::Matrix3d &finalS,
                                      Eigen::VectorXd &opt_x, const int N,
                                      Trajectory<TRAJ_ORDER> &traj) {
  if (conf.se2_past <= 0 || !std::isfinite(conf.se2_delta_past) ||
      conf.se2_delta_past <= 0.0) {
    ROS_ERROR("[SE2] convergence settings must be finite and positive");
    traj.clear();
    return lmbm::LMBMERR_INVALIDPARAMS;
  }
  cout << "svsdf planner successfully call" << endl;
  total_opt_time = 0.0;
  total_sdf_time = 0.0;
  total_AABB_time = 0.0;
  ros::Time start_time = ros::Time::now();
  pieceN = N;
  temporalDim = N;
  spatialDim = 3 * (N - 1);
  initState = initS;
  finalState = finalS;

  std::cout << "N: " << N << ", temporalDim: " << temporalDim
            << ", spatialDim: " << spatialDim << std::endl;
  minco.setConditions(initState, finalState, pieceN);
  const int total_opt_variable_num = temporalDim + spatialDim;

  x_variable = (double *)malloc(sizeof(double) * total_opt_variable_num);
  if (x_variable == nullptr) {
    cout << "x_variable is nullptr" << endl;
    return -1;
  } else if (total_opt_variable_num > 400) {
    cout << "x_variable is too large" << endl;
    return -1;
  }
  Eigen::Map<Eigen::VectorXd>(x_variable, total_opt_variable_num) = opt_x;
  iter = 0;
  std::vector<double> clear_flag;
  clear_flag.push_back(-1);
  debug_publisher::DBSendOptiStep(clear_flag);
  debug_publisher::DBSendLogCost(clear_flag);

  double final_cost;
  lmbm::lmbm_parameter_t param;
  param.past = conf.se2_past;
  param.delta_past = conf.se2_delta_past;
  ROS_INFO("[SE2] convergence: past %d, delta %.3g",
           param.past, param.delta_past);
  cout << "max_iterations: " << param.max_iterations << endl;
  cout << "svsdf planner parameter successfully initial" << endl;

  resetSE2Termination();
  int ret = lmbm::lmbm_optimize(total_opt_variable_num, x_variable, &final_cost,
                                costFunctionLmbmParallel, this, earlyExitLMBM,
                                &param);
  ROS_INFO("[SE2] stopped with status %d (%s) after %.3f s (%d callbacks), "
           "safety-band intrusion %.4f m; candidate still requires collision "
           "validation",
           ret,
           se2_stop_reason.empty() ? "LMBM criteria" : se2_stop_reason.c_str(),
           std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         se2_start_time)
               .count(),
           iter,
           iterate_violations.empty() ? -1.0 : iterate_violations.back());
  {
    printf("-----------\n");
    printf("| Output: |\n");
    printf("-----------\n");
    printf("%-16s %f\n", "Final value:", final_cost);
    std::cout << "[total cost] = " << cost_total
              << " , [energy cost] = " << energy_cost_global
              << " , [time cost] = " << time_cost_global
              << " , [pos cost] = " << pos_cost_global << std::endl;
  }

  if (ret >= 0) {
    forwardT(x_variable, times, temporalDim); // tao--->T
    forwardP((x_variable + temporalDim), points, spatialDim / 3);
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);
    minco.getTrajectory(traj);
    std::cout << "[Optimization] Optimization Success. Final Cost = "
              << final_cost << std::endl;

    for (int i = 0; i < total_opt_variable_num; i++) {
      opt_x(i) = x_variable[i];
    }

    if (x_variable != NULL) {
      free(x_variable);
      x_variable = NULL;
    }
    if (ret == 0) {
      ret = 1;
    }
    return ret;
  } else {
    forwardT(x_variable, times, temporalDim); // tao--->T
    forwardP((x_variable + temporalDim), points, spatialDim / 3);
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);
    minco.getTrajectory(traj);
    std::cout << "\x33"
              << "[Optimization] Optimization Failed: "
              << " code = " << ret << std::endl;

    for (int i = 0; i < total_opt_variable_num; i++) {
      opt_x(i) = x_variable[i];
    }

    if (x_variable != NULL) {
      free(x_variable);
      x_variable = NULL;
    }
    return ret;
  }

  return ret;
}

void TrajOptimizer::drawStepTraj() {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.id = 125 + 8000;

  line_strip.color.r = 0;
  line_strip.color.g = 1;
  line_strip.color.b = 0;
  line_strip.color.a = 1;
  line_strip.scale.x = 0.05 / 2;
  line_strip.lifetime = ros::Duration(0.4);

  geometry_msgs::Point pt;
  double dur = step_traj.getDurations().sum();
  for (double i = 0; i < dur - 1e-4; i += 0.1) {
    Eigen::Vector3d dur_p = step_traj.getPos(i);
    pt.x = dur_p(0);
    pt.y = dur_p(1);
    pt.z = 0;
    line_strip.points.push_back(pt);
  }
  debug_step_pub.publish(line_strip);
}

void TrajOptimizer::drawFinalTraj() {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.id = 125 + 8000;

  line_strip.color.r = 0;
  line_strip.color.g = 1;
  line_strip.color.b = 0;
  line_strip.color.a = 1;
  line_strip.scale.x = 0.05 / 2;
  line_strip.lifetime = ros::Duration(0.01);

  geometry_msgs::Point pt;
  double dur = step_traj.getDurations().sum();
  for (double i = 0; i < dur - 1e-4; i += 0.1) {
    Eigen::Vector3d dur_p = step_traj.getPos(i);
    pt.x = dur_p(0);
    pt.y = dur_p(1);
    pt.z = 0;
    line_strip.points.push_back(pt);
  }
  debug_step_pub.publish(line_strip);
}

void TrajOptimizer::renderAABBpoints(const std::string &group_namespace) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = group_namespace;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = conf.occupancy_resolution * 0.8;
  marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0;
  for (const auto &entry : pcsmap_manager->aabb_points) {
    geometry_msgs::Point point;
    point.x = entry.second.x();
    point.y = entry.second.y();
    point.z = entry.second.z();
    marker.points.push_back(point);
  }

  // Keep a complete candidate snapshot so late subscribers see every group.
  bool replaced = false;
  for (auto &existing : aabb_group_markers.markers) {
    if (existing.action == visualization_msgs::Marker::ADD &&
        existing.ns == group_namespace) {
      existing = marker;
      replaced = true;
      break;
    }
  }
  if (!replaced) {
    aabb_group_markers.markers.push_back(marker);
  }
  aabb_points_pub.publish(aabb_group_markers);
  ROS_INFO("[AABB] %zu obstacle points in namespace %s on aabbpoints",
           marker.points.size(), group_namespace.c_str());
}

void TrajOptimizer::clearvisAABBpoints() {
  aabb_group_markers.markers.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::DELETEALL;
  // Retain deletion at the start of every snapshot to remove old namespaces.
  aabb_group_markers.markers.push_back(marker);
  aabb_points_pub.publish(aabb_group_markers);
}
