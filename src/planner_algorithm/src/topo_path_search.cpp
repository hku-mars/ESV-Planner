#include <planner_algorithm/topo_path_search.h>

// bool TopoPath::InitGlobalPath(){

// }

TopoPath::TopoPath(/* args */) {}

TopoPath::~TopoPath() {}
void TopoPath::init(ros::NodeHandle &nh, Config &config) {
  graph_.clear();
  eng_ = default_random_engine(rd_());
  rand_pos_ = uniform_real_distribution<double>(-1.0, 1.0);

  // _occ_grid_map.reset(new OccGridMap2D());
  // _occ_grid_map->initMap(nh);
  // getMapPara();
  _vis_ptr.reset(new PlanningVisualization(nh));

  // init parameter
  sample_margin_x = config.topo_prm_sample_margin_x;
  sample_margin_y = config.topo_prm_sample_margin_y;
  sample_width_x = config.topo_prm_sample_width_x;
  sample_width_y = config.topo_prm_sample_width_y;
  reserve_num_ = config.topo_prm_reserve_num;
  ratio_to_short_ = config.topo_prm_ratio_to_short;
  max_sample_num_ = config.topo_prm_max_sample_num;
  max_sample_time_ = config.topo_prm_max_sample_time;
  max_raw_path_ = config.topo_prm_max_raw_path;
  max_raw_path2_ = config.topo_prm_max_raw_path2;
  test_mode = config.topo_prm_test_mode;
  iterator_num = config.topo_prm_iterator_num;
  push_extra_dist_ = config.push_extra_dist;
  kernel_size = config.kernel_size;
  kernel_count = config.kernel_yaw_num;

  for (int i = 0; i < max_raw_path_; ++i) {
    raycaster.push_back(RayCaster());
  }

  boundary_xyzmin << -config.mapSize_x / 2.0, -config.mapSize_x / 2.0, 0.0;
  boundary_xyzmax << +config.mapSize_y / 2.0, +config.mapSize_y / 2.0, 0.2;
}

void TopoPath::setMapManager(PCSmapManager::Ptr map_manager,
                             SweptVolumeManager::Ptr sv) {
  environment = map_manager;
  resolution_ = environment->occupancy_map->grid_resolution;

  GLX_SIZE = ceil((boundary_xyzmax(0) - boundary_xyzmin(0)) / resolution_);
  GLY_SIZE = ceil((boundary_xyzmax(1) - boundary_xyzmin(1)) / resolution_);
  sv_manager = sv;
  robot_shape = sv_manager->getRobotShape();

  cout << "GLX_SIZE: " << GLX_SIZE << " GLY_SIZE: " << GLY_SIZE
       << " resolution: " << resolution_ << endl;
}

void TopoPath::findTopoPaths(Eigen::Vector2d start, Eigen::Vector2d end,
                             std::list<GraphNode::Ptr> &graph,
                             vector<PathNode::Ptr> &raw_paths,
                             vector<PathNode::Ptr> &short_paths_,
                             vector<PathNode::Ptr> &filtered_paths,
                             vector<PathNode::Ptr> &select_paths) {
  ros::Time t1, t2;

  double graph_time, search_time, short_time, prune_time, select_time;
  /* ---------- create the topo graph ---------- */
  t1 = ros::Time::now();

  Vector2i start_idx = (environment->occupancy_map)->getGridIndex2d(start);
  Vector2i end_idx = (environment->occupancy_map)->getGridIndex2d(end);

  start_pt = (environment->occupancy_map)->getGridCubeCenter2d(start_idx);
  end_pt = (environment->occupancy_map)->getGridCubeCenter2d(end_idx);

  start_pts_ = start_pt;
  end_pts_ = end_pt;
  ROS_INFO("start: %f, %f", start_pt.x(), start_pt.y());
  ROS_INFO("end: %f, %f", end_pt.x(), end_pt.y());

  graph = createGraph(start_pt, end_pt);

  graph_time = (ros::Time::now() - t1).toSec();

  /* ---------- search paths in the graph ---------- */
  t1 = ros::Time::now();

  raw_paths = searchPaths();

  search_time = (ros::Time::now() - t1).toSec();

  vector<PathNode::Ptr> raw_paths_copy;
  for (const auto &path : raw_paths) {
    PathNode::Ptr path_copy = std::make_shared<PathNode>(*path);
    raw_paths_copy.push_back(path_copy);
  }

  /* ---------- path shortening ---------- */
  // for parallel, save result in short_paths_
  t1 = ros::Time::now();

  short_paths_ = shortcutPath(raw_paths_copy);

  short_time = (ros::Time::now() - t1).toSec();

  /* ---------- prune equivalent paths ---------- */
  t1 = ros::Time::now();

  filtered_paths = pruneEquivalent(short_paths_);

  prune_time = (ros::Time::now() - t1).toSec();

  /* ---------- select N shortest paths ---------- */
  t1 = ros::Time::now();

  select_paths = selectShortPaths(filtered_paths, 1);

  select_time = (ros::Time::now() - t1).toSec();

  double total_time =
      graph_time + search_time + short_time + prune_time + select_time;

  std::cout << "\n[Topo]: total time: " << total_time
            << ", graph: " << graph_time << ", search: " << search_time
            << ", short: " << short_time << ", prune: " << prune_time
            << ", select: " << select_time
            << ", total sample numbers: " << total_samples << std::endl;
}

list<GraphNode::Ptr> TopoPath::createGraph(Eigen::Vector2d start,
                                           Eigen::Vector2d end) {
  // Output log message indicating the start of the search process.
  std::cout << "[Topo]: createGraph----------------------" << std::endl;
  vector<GraphNode::Ptr> connectors_log;
  // Initialize graph structure and clear any previous data.
  graph_.clear();
  // collis_.clear(); // Optionally clear collision data, if applicable.

  // Create and initialize the start and end nodes of the graph.
  GraphNode::Ptr start_node =
      GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
  GraphNode::Ptr end_node =
      GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));

  // Add start and end nodes to the graph.
  graph_.push_back(start_node);
  graph_.push_back(end_node);

  // Sampling rectangle for this query: world axis aligned, centred between
  // start and goal. Every guard the roadmap places lives inside this box, so
  // it has to contain both endpoints, which the half extents below enforce
  // whichever way they are configured.
  sample_center_ = 0.5 * (start + end);
  // Absolute width wins when it is set, otherwise the box is the start/goal
  // bounding box plus the configured margins. Either way the half extent is
  // never smaller than the start/goal span, so the rectangle always contains
  // both endpoints.
  const double half_span_x = 0.5 * std::abs(end.x() - start.x());
  const double half_span_y = 0.5 * std::abs(end.y() - start.y());
  if (sample_width_x > 0.0) {
    sample_half_x_ = std::max(0.5 * sample_width_x, half_span_x);
    if (0.5 * sample_width_x < half_span_x) {
      ROS_WARN("[Topo] sample width x %.2f m is narrower than the start/goal "
               "span %.2f m, widening it to contain both endpoints",
               sample_width_x, 2.0 * half_span_x);
    }
  } else {
    sample_half_x_ = half_span_x + sample_margin_x;
  }
  if (sample_width_y > 0.0) {
    sample_half_y_ = std::max(0.5 * sample_width_y, half_span_y);
    if (0.5 * sample_width_y < half_span_y) {
      ROS_WARN("[Topo] sample width y %.2f m is narrower than the start/goal "
               "span %.2f m, widening it to contain both endpoints",
               sample_width_y, 2.0 * half_span_y);
    }
  } else {
    sample_half_y_ = half_span_y + sample_margin_y;
  }

  ROS_INFO("sample box: x [%.2f, %.2f], y [%.2f, %.2f] (centre %.2f, %.2f)",
           sample_center_.x() - sample_half_x_,
           sample_center_.x() + sample_half_x_,
           sample_center_.y() - sample_half_y_,
           sample_center_.y() + sample_half_y_, sample_center_.x(),
           sample_center_.y());

  _vis_ptr->drawSampleRegion(sample_center_, sample_half_x_, sample_half_y_,
                             0.05, Eigen::Vector4d(1.0, 0.6, 0.0, 0.8));

  int node_id = 1; // Start node IDs from 1, since 0 is used for the start node.

  // Main sampling loop: run until either maximum sample time or number is
  // reached.
  int sample_num = 0;
  double sample_time = 0.0;
  Eigen::Vector2d pt;
  // Eigen::Vector2d pt1(1.7, 1.7);
  // Eigen::Vector2d pt2(0.1, 1.3);
  // Eigen::Vector2d pt3(1.5, 1.9);
  // Eigen::Vector2d pt4(1.1, 2.7);
  // Eigen::Vector2d pt5(1.9, 2.9);
  // Eigen::Vector2d pt6(2.5, 1.5);
  // Eigen::Vector2d pt7(1.9, 1.1);
  // Eigen::Vector2d pt8(1.3, 0.5);

  // Eigen::Vector2d pt1(-2.1,-0.1);
  // Eigen::Vector2d pt2(-1.1,0.3);
  // Eigen::Vector2d pt3(-0.3,3.7);
  // Eigen::Vector2d pt4(0.1,4.5);
  // Eigen::Vector2d pt5(0.1,2.3);
  // Eigen::Vector2d pt6(2.1,2.7);
  // Eigen::Vector2d pt7(1.9,2.9);
  // Eigen::Vector2d pt8(1.9,4.1);
  // Eigen::Vector2d pt9(3.1,3.5);

  // tunnel test
  //  Eigen::Vector2d pt1(-3.1,8.5);
  //  Eigen::Vector2d pt2(-3.1,1.3);
  //  Eigen::Vector2d pt3(0.5,8.5);
  //  Eigen::Vector2d pt4(3.3,8.7);

  // Eigen::Vector2d pt1(-1.1,0.3);
  // Eigen::Vector2d pt2(-2.1,-0.1);
  // Eigen::Vector2d pt3(0.1,4.5);

  Eigen::Vector2d pt1(-1.3, 4.1);

  // std::vector<Eigen::Vector2d> fixedPoints {pt1,pt2,pt3, pt4,pt5,pt6,pt7,pt8,
  // pt9};

  // std::vector<Eigen::Vector2d> fixedPoints {pt3,pt1,pt2,pt4};
  // std::vector<Eigen::Vector2d> fixedPoints {pt1,pt2,pt3};
  std::vector<Eigen::Vector2d> fixedPoints{pt1};

  int pointIndex = 0;

  vector<Eigen::Vector2d> sample_points;
  ros::Time t1;
  int counter1 = 0, counter2 = 0, counter3 = 0, counter4 = 0, counter5 = 0,
      counter6 = 0;
  while (true) {
    if (sample_time >= max_sample_time_ || sample_num >= max_sample_num_) {

      break;
    }

    t1 = ros::Time::now(); // Start timing for the sampling iteration.

    // Get a random sample point in the transformed coordinate system.
    pt = getSample();
    // pt = fixedPoints[pointIndex++];

    ++sample_num;
    if (test_mode) {
      ROS_ERROR("sample point: %f, %f", pt.x(), pt.y());
    }

    // if sample point is inside the obstacle
    if ((environment->occupancy_map)->isCoordOccupied2d(pt)) {
      sample_time += (ros::Time::now() - t1).toSec();
      ++counter1;
      continue;
    }
    sample_points.push_back(pt);
    // Attempt to find visible guards that can connect to the sampled point.
    vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt);
    if (visib_guards.size() ==
        0) { // If no guards are visible, add a new guard.
      if (test_mode) {
        ROS_WARN("Sample point %f, %f is not visible, added to Guard", pt.x(),
                 pt.y());
      }
      GraphNode::Ptr guard =
          GraphNode::Ptr(new GraphNode(pt, GraphNode::Guard, ++node_id));

      graph_.push_back(guard);
      ++counter5;
    } else if (visib_guards.size() ==
               2) { // If two guards are visible, possibly connect them.
      // Check if a new connection is necessary based on the visibility and
      // position of guards
      if (test_mode) {
        ROS_INFO("Two guards are visible");
      }
      bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
      if (!need_connect) {
        ++counter2;
        sample_time += (ros::Time::now() - t1).toSec();
        continue;
      }
      if (test_mode) {
        ROS_INFO("need connection");
      }
      // If a new connection is needed, create and add a new connector node.
      GraphNode::Ptr connector =
          GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));

      graph_.push_back(connector);

      // Connect the new connector with the visible guards.
      visib_guards[0]->neighbors_.push_back(connector);
      visib_guards[1]->neighbors_.push_back(connector);
      // After adding the connector to the neighbors of the guard nodes
      connector->neighbors_.push_back(visib_guards[0]);
      connector->neighbors_.push_back(visib_guards[1]);
      connectors_log.push_back(connector);
    }

    // Update the sample time with the duration of the current iteration.
    sample_time += (ros::Time::now() - t1).toSec();
  }

  // Output log showing the number of samples processed.
  // std::cout << "[Topo]: sample num: " << sample_num;
  total_samples = sample_num;

  // Optional graph pruning or cleanup function.
  pruneGraph();
  std::cout << "[Topo]: node num: " << graph_.size() << std::endl;

  return graph_;
}

/*Sample global map*/
Eigen::Vector2d TopoPath::getSample() {
  // Uniform over the world axis aligned rectangle centred between start and
  // goal: sample_center_ +- (sample_half_x_, sample_half_y_), which spans the
  // start/goal bounding box plus the configured margins. rand_pos_ is uniform
  // on [-1, 1], so each axis covers its full half extent both ways.
  Eigen::Vector2d pt;
  pt(0) = sample_center_(0) + rand_pos_(eng_) * sample_half_x_;
  pt(1) = sample_center_(1) + rand_pos_(eng_) * sample_half_y_;

  // Snap to the occupancy grid so guards and connectors land on cell centres.
  Eigen::Vector2i gridIndex = (environment->occupancy_map)->getGridIndex2d(pt);
  return (environment->occupancy_map)->getGridCubeCenter2d(gridIndex);
}

vector<GraphNode::Ptr> TopoPath::findVisibGuard(Eigen::Vector2d pt) {
  vector<GraphNode::Ptr> visib_guards;
  int visib_num = 0;
  if (test_mode) {
    cout << "-----------------------------findVisibGuard" << endl;
  }
  /* find visible GUARD from pt */
  for (list<GraphNode::Ptr>::iterator iter = graph_.begin();
       iter != graph_.end(); ++iter) {
    if ((*iter)->type_ == GraphNode::Connector) {
      continue;
    }
    // ROS_WARN("current viewing guard is x: %f, y: %f", (*iter)->pos_.x(),
    // (*iter)->pos_.y());
    if (lineOfSight(pt, (*iter)->pos_)) {
      if (test_mode) {
        ROS_WARN("guard x: %f, y: %f is visible with point x: %f, y: %f",
                 (*iter)->pos_.x(), (*iter)->pos_.y(), pt.x(), pt.y());
      }

      // cout<<"-----------------------------"<<endl;
      visib_guards.push_back((*iter));
      ++visib_num;
      // ROS_INFO("visible guard: %d", visib_num);
      if (visib_num > 2)
        break;
    }
  }
  return visib_guards;
}

bool TopoPath::lineOfSight(const Eigen::Vector2d &start,
                           const Eigen::Vector2d &target) {
  Eigen::Vector2i startIdx =
      (environment->occupancy_map)->getGridIndex2d(start);
  Eigen::Vector2i targetIdx =
      (environment->occupancy_map)->getGridIndex2d(target);
  int x0 = startIdx(0);
  int y0 = startIdx(1);
  int x1 = targetIdx(0);
  int y1 = targetIdx(1);

  int dx = x1 - x0;
  int dy = y1 - y0;
  int nx = abs(dx), ny = abs(dy);
  int sign_x = dx > 0 ? 1 : -1, sign_y = dy > 0 ? 1 : -1;

  int px = x0, py = y0;
  int ix = 0, iy = 0;

  while (ix < nx || iy < ny) {
    if ((0.5 + ix) / nx < (0.5 + iy) / ny) {
      px += sign_x;
      ix++;
    } else {
      py += sign_y;
      iy++;
    }
    Vector2i check_idx(px, py);
    if ((environment->occupancy_map)->isIndexOccupied2d(check_idx)) {
      return false;
    }
  }

  return true;
}

bool TopoPath::lineOfSight(const Eigen::Vector2d &start,
                           const Eigen::Vector2d &target,
                           Eigen::Vector2d &collid_pt) {
  Eigen::Vector2i startIdx =
      (environment->occupancy_map)->getGridIndex2d(start);
  Eigen::Vector2i targetIdx =
      (environment->occupancy_map)->getGridIndex2d(target);
  int x0 = startIdx(0);
  int y0 = startIdx(1);
  int x1 = targetIdx(0);
  int y1 = targetIdx(1);

  int dx = x1 - x0;
  int dy = y1 - y0;
  int nx = abs(dx), ny = abs(dy);
  int sign_x = dx > 0 ? 1 : -1, sign_y = dy > 0 ? 1 : -1;

  int px = x0, py = y0;
  int ix = 0, iy = 0;

  while (ix < nx || iy < ny) {
    if ((0.5 + ix) / nx < (0.5 + iy) / ny) {
      px += sign_x;
      ix++;
    } else {
      py += sign_y;
      iy++;
    }
    Vector2i check_idx(px, py);
    if ((environment->occupancy_map)->isIndexOccupied2d(check_idx)) {
      collid_pt = (environment->occupancy_map)->getGridCubeCenter2d(check_idx);
      return false;
    }
  }

  return true;
}

bool TopoPath::needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2,
                              Eigen::Vector2d pt) {
  vector<Eigen::Vector2d> path1(3), path2(3);
  path1[0] = g1->pos_;
  path1[1] = pt;
  path1[2] = g2->pos_;

  path2[0] = g1->pos_;
  path2[2] = g2->pos_;

  bool has_connect = false;

  for (int i = 0; i < g1->neighbors_.size(); ++i) {
    for (int j = 0; j < g2->neighbors_.size(); ++j) {
      if (g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) {
        path2[1] = g1->neighbors_[i]->pos_;
        bool same_topo = sameTopoPath(path1, path2);
        if (same_topo) {
          // get shorter connection ?
          // ROS_INFO("same topo");
          if (pathLength(path1) < pathLength(path2)) {
            g1->neighbors_[i]->pos_ = pt;
            // ROS_WARN("shorter!");
          }
          return false;
        }
      }
    }
  }
  return true;
}

bool TopoPath::sameTopoPath(const vector<Eigen::Vector2d> &path1,
                            const vector<Eigen::Vector2d> &path2) {
  // calc the length
  //   ROS_INFO("Path1 size: %d", path1.size());
  //   ROS_INFO("Path2 size: %d", path2.size());
  double len1 = pathLength(path1);
  double len2 = pathLength(path2);
  // ROS_WARN("len1: %f, len2: %f", len1, len2);
  // ROS_WARN("resolution: %f", resolution_);
  double max_len = max(len1, len2);
  // std::cout << "max len: " << max_len << std::endl;
  unsigned int pt_num = ceil(max_len / resolution_);

  // std::cout << "pt num: " << pt_num << std::endl;

  vector<Eigen::Vector2d> pts1 = discretizePath(path1, pt_num);
  vector<Eigen::Vector2d> pts2 = discretizePath(path2, pt_num);

  if (test_mode) {
    // _vis_ptr->drawsamplePoints(pts1, resolution_/4,Eigen::Vector4d(0, 0, 1,
    // 0.7),0); _vis_ptr->drawsamplePoints(pts2,
    // resolution_/4,Eigen::Vector4d(0, 1, 0, 0.7),1);
  }
  vector<Eigen::Vector2d> pts;

  for (int i = 0; i < pt_num; ++i) {
    if (!lineOfSight(pts1[i], pts2[i])) {
      // pts.push_back(pts1[i]);
      // pts.push_back(pts2[i]);
      // _vis_ptr->drawsamplePoints(pts, resolution_/2,Eigen::Vector4d(1, 0,
      // 0, 1.0),2); _vis_ptr->drawPath(pts1[i],pts2[i],resolution_,
      // Eigen::Vector4d(1, 0, 0, 1.0), 2);

      return false;
    }
  }

  return true;
}

double TopoPath::pathLength(const vector<Eigen::Vector2d> &path) {
  double length = 0.0;
  if (path.size() < 2)
    return length;

  for (int i = 0; i < path.size() - 1; ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}

vector<Eigen::Vector2d>
TopoPath::discretizePath(const std::vector<Eigen::Vector2d> &path, int pt_num) {
  std::vector<double> len_list;
  len_list.push_back(0.0);

  // Calculate the cumulative length of each segment
  for (int i = 0; i < path.size() - 1; ++i) {
    double inc_l = (path[i + 1] - path[i]).norm();
    len_list.push_back(inc_l + len_list.back());
  }

  // Calculate pt_num points along the path
  double len_total = len_list.back();
  double dl = len_total / (pt_num - 1);
  double cur_l;

  std::vector<Eigen::Vector2d> dis_path;
  for (int i = 0; i < pt_num; ++i) {
    cur_l = double(i) * dl;

    // Find the segment where cur_l falls
    int idx = -1;
    for (int j = 0; j < len_list.size() - 1; ++j) {
      if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
        idx = j;
        break;
      }
    }

    // Interpolate to find the point at cur_l
    double lambda =
        (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
    Eigen::Vector2d inter_pt =
        (1 - lambda) * path[idx] + lambda * path[idx + 1];
    dis_path.push_back(inter_pt);
  }

  return dis_path;
}

// search for useful path in the topo graph by DFS
// vector<vector<Eigen::Vector2d>> TopoPath::searchPaths() {
//     raw_paths_.clear();

//     vector<GraphNode::Ptr> visited;
//     visited.push_back(graph_.front());

//     depthFirstSearch(visited);

//     // sort the path by node number
//     int min_node_num = 100000, max_node_num = 1;
//     vector<vector<int>> path_list(100);
//     for (int i = 0; i < raw_paths_.size(); ++i) {
//     if (int(raw_paths_[i].size()) > max_node_num) max_node_num =
//     raw_paths_[i].size(); if (int(raw_paths_[i].size()) < min_node_num)
//     min_node_num = raw_paths_[i].size();
//     path_list[int(raw_paths_[i].size())].push_back(i);
//     }

//     // select paths with less nodes
//     vector<vector<Eigen::Vector2d>> filter_raw_paths;
//     for (int i = min_node_num; i <= max_node_num; ++i) {
//     bool reach_max = false;
//     for (int j = 0; j < path_list[i].size(); ++j) {
//         filter_raw_paths.push_back(raw_paths_[path_list[i][j]]);
//         if (filter_raw_paths.size() >= max_raw_path2_) {
//         reach_max = true;
//         break;
//         }
//     }
//     if (reach_max) break;
//     }
//     std::cout << ", raw path num: " << raw_paths_.size() << ", " <<
//     filter_raw_paths.size();

//     raw_paths_ = filter_raw_paths;

//     return raw_paths_;
// }

vector<PathNode::Ptr> TopoPath::searchPaths() {
  raw_paths_.clear();
  int pathID = 0;
  vector<GraphNode::Ptr> visited;
  visited.push_back(graph_.front());

  depthFirstSearch(visited);

  // Sort the paths by node number
  int min_node_num = 100000, max_node_num = 1;
  vector<vector<int>> path_list(100);
  for (int i = 0; i < raw_paths_.size(); ++i) {
    int path_size = int(raw_paths_[i].size());
    if (path_size > max_node_num)
      max_node_num = path_size;
    if (path_size < min_node_num)
      min_node_num = path_size;
    path_list[path_size].push_back(i);
  }

  // Select paths with fewer nodes
  vector<PathNode::Ptr> filter_raw_paths;
  for (int i = min_node_num; i <= max_node_num; ++i) {
    bool reach_max = false;
    for (int j = 0; j < path_list[i].size(); ++j) {
      PathNode::Ptr pathObj = PathNode::Ptr(new PathNode());
      pathObj->path = raw_paths_[path_list[i][j]];
      pathObj->pathID = pathID;
      filter_raw_paths.push_back(pathObj);
      pathID++;
      if (filter_raw_paths.size() >= max_raw_path2_) {
        reach_max = true;
        break;
      }
    }
    if (reach_max)
      break;
  }

  std::cout << ", raw path num: " << raw_paths_.size() << ", filter_raw_paths "
            << filter_raw_paths.size() << std::endl;

  // Convert filtered paths to the required return type
  // vector<vector<Eigen::Vector2d>> result_paths;
  // for (const auto& pathNode : filter_raw_paths) {
  //     result_paths.push_back(pathNode->path);
  //     std::cout << "path is: " << pathNode->pathID << std::endl;
  //     for(const auto& pt: pathNode->path){
  //         std::cout << "x: " << pt.x() << " y: " << pt.y() << std::endl;
  //     }
  // }

  return filter_raw_paths;
}

// void TopoPath::depthFirstSearch(vector<GraphNode::Ptr>& vis) {
//     GraphNode::Ptr cur = vis.back();

//     for (int i = 0; i < cur->neighbors_.size(); ++i) {
//         // check reach goal
//         if (cur->neighbors_[i]->id_ == 1) {
//             // add this path to paths set
//             vector<Eigen::Vector2d> path;
//             for (int j = 0; j < vis.size(); ++j) {
//                 path.push_back(vis[j]->pos_);
//             }
//             path.push_back(cur->neighbors_[i]->pos_);

//             raw_paths_.push_back(path);
//             if (raw_paths_.size() >= max_raw_path_)
//                 return;

//             break;
//         }
//     }
//     // Recursive Exploration Loop
//     for (int i = 0; i < cur->neighbors_.size(); ++i) {
//         // skip reach goal
//         if (cur->neighbors_[i]->id_ == 1)
//             continue;

//         // skip already visited node
//         bool revisit = false;
//         for (int j = 0; j < vis.size(); ++j) {
//             if (cur->neighbors_[i]->id_ == vis[j]->id_) {
//                 revisit = true;
//                 break;
//             }
//         }
//         if (revisit)
//             continue;

//         // recursive search
//         vis.push_back(cur->neighbors_[i]);
//         depthFirstSearch(vis);
//         if (raw_paths_.size() >= max_raw_path_)
//             return;

//         vis.pop_back();
//     }
// }

void TopoPath::depthFirstSearch(vector<GraphNode::Ptr> &vis) {
  // If we've exceeded the maximum number of paths, return immediately
  if (raw_paths_.size() >= max_raw_path_) {
    return;
  }

  GraphNode::Ptr cur = vis.back();

  // First check if we can reach the goal from current node
  for (const auto &neighbor : cur->neighbors_) {
    if (neighbor->id_ == 1) { // Found path to goal
      vector<Eigen::Vector2d> path;
      for (const auto &node : vis) {
        path.push_back(node->pos_);
      }
      path.push_back(neighbor->pos_);
      raw_paths_.push_back(path);

      if (raw_paths_.size() >= max_raw_path_) {
        return;
      }
      break; // No need to check other direct connections to goal
    }
  }

  // Then explore other paths
  for (const auto &neighbor : cur->neighbors_) {
    // Skip if this is the goal node (already handled above)
    if (neighbor->id_ == 1) {
      continue;
    }

    // Skip if we've already visited this node
    bool already_visited = false;
    for (const auto &visited_node : vis) {
      if (neighbor->id_ == visited_node->id_) {
        already_visited = true;
        break;
      }
    }
    if (already_visited) {
      continue;
    }

    // Add neighbor to visited list
    vis.push_back(neighbor);

    // Recursively search from this neighbor
    depthFirstSearch(vis);

    // Remove neighbor from visited list (backtrack)
    vis.pop_back();

    // If we've found enough paths, stop searching
    if (raw_paths_.size() >= max_raw_path_) {
      return;
    }
  }
}

void TopoPath::pruneGraph() {
  /* prune useless node */
  if (graph_.size() > 2) {
    for (list<GraphNode::Ptr>::iterator iter1 = graph_.begin();
         iter1 != graph_.end() && graph_.size() > 2; ++iter1) {
      if ((*iter1)->id_ <= 1)
        continue;

      /* core */
      // std::cout << "id: " << (*iter1)->id_ << std::endl;
      if ((*iter1)->neighbors_.size() <= 1) {
        // delete this node from others' neighbor
        for (list<GraphNode::Ptr>::iterator iter2 = graph_.begin();
             iter2 != graph_.end(); ++iter2) {
          for (vector<GraphNode::Ptr>::iterator it_nb =
                   (*iter2)->neighbors_.begin();
               it_nb != (*iter2)->neighbors_.end(); ++it_nb) {
            if ((*it_nb)->id_ == (*iter1)->id_) {
              (*iter2)->neighbors_.erase(it_nb);
              break;
            }
          }
        }

        // delete this node from graph, restart checking
        graph_.erase(iter1);
        iter1 = graph_.begin();
      }
    }
  }
}

vector<PathNode::Ptr> TopoPath::pruneEquivalent(vector<PathNode::Ptr> &paths) {
  vector<PathNode::Ptr> pruned_paths;
  if (paths.size() < 1)
    return pruned_paths;

  /* ---------- prune topo equivalent path ---------- */
  // output: pruned_paths
  vector<int> exist_paths_id;
  exist_paths_id.push_back(0);

  for (int i = 1; i < paths.size(); ++i) {
    // compare with exsit paths
    bool new_path = true;

    for (int j = 0; j < exist_paths_id.size(); ++j) {
      // compare with one path
      bool same_topo =
          sameTopoPath(paths[i]->path, paths[exist_paths_id[j]]->path);

      if (same_topo) {
        new_path = false;
        break;
      }
    }

    if (new_path) {
      exist_paths_id.push_back(i);
    }
  }

  // save pruned paths
  for (int i = 0; i < exist_paths_id.size(); ++i) {
    pruned_paths.push_back(paths[exist_paths_id[i]]);
  }

  std::cout << ", pruned path num: " << pruned_paths.size();

  return pruned_paths;
}

std::vector<PathNode::Ptr>
TopoPath::selectShortPaths(std::vector<PathNode::Ptr> &paths, int step) {
  std::vector<PathNode::Ptr> short_paths;
  double min_len = std::numeric_limits<double>::max();

  for (int i = 0; i < reserve_num_ && !paths.empty(); ++i) {
    int path_id = shortestPath(paths);
    if (path_id == -1)
      break; // no valid path found

    double current_length = pathLength(paths[path_id]->path);

    if (i == 0) {
      short_paths.push_back(paths[path_id]);
      min_len = current_length;
    } else {
      double ratio = current_length / min_len;
      if (ratio < ratio_to_short_) {
        short_paths.push_back(paths[path_id]);
      } else {
        break;
      }
    }
    paths.erase(paths.begin() + path_id);
  }

  std::cout << ", select path num: " << short_paths.size() << std::endl;
  return short_paths;
}

int TopoPath::shortestPath(const std::vector<PathNode::Ptr> &paths) {
  int short_id = -1;
  double min_len = 100000000;
  for (int i = 0; i < paths.size(); ++i) {
    double len = pathLength(paths[i]->path);
    if (len < min_len) {
      short_id = i;
      min_len = len;
    }
  }
  return short_id;
}

vector<PathNode::Ptr>
TopoPath::shortcutPath(const vector<PathNode::Ptr> &paths) {
  vector<PathNode::Ptr> new_paths;

  for (int i = 0; i < paths.size(); i++) {
    // Create a new PathNode (assuming you have a copy constructor or similar)
    PathNode::Ptr new_node = std::make_shared<PathNode>(*paths[i]);

    vector<Eigen::Vector2d> current_path = paths[i]->path;
    vector<Eigen::Vector2d> test_shortpath =
        shortcutPath_fastPlanner(current_path, i);

    if (!test_shortpath.empty()) {
      new_node->path = test_shortpath;
    } else {
      new_node->path = current_path; // Keep original path if shortcut fails
    }

    new_paths.push_back(new_node);
  }

  return new_paths;
}

std::vector<Eigen::Vector2d>
TopoPath::shortcutPath_fastPlanner(vector<Eigen::Vector2d> path, int path_id) {
  vector<Eigen::Vector2d> short_path = path;
  vector<Eigen::Vector2d> last_path;
  Eigen::Vector2d last_seenPt, start_pt;
  vector<Eigen::Vector2i> kernel_collision;

  for (int k = 0; k < iterator_num; ++k) {
    last_path = short_path;

    vector<Eigen::Vector2d> dis_path = discretizePath_fastplanner(short_path);
    cout << "[shortcut][path " << path_id << "][iter " << k
         << "] dis_path size: " << dis_path.size()
         << ", incoming short_path size: " << last_path.size() << endl;
    if (dis_path.size() < 2) {
      cout << "[shortcut][path " << path_id << "][iter " << k
           << "] dis_path.size() < 2, returning early" << endl;
      return dis_path;
    }
    /* visibility path shortening */
    Eigen::Vector2d colli_pt;
    short_path.clear();
    short_path.push_back(dis_path.front());
    for (int i = 1; i < dis_path.size(); ++i) {
      if (lineOfSight(short_path.back(), dis_path[i], colli_pt)) {
        continue;
      }
      start_pt = short_path.back();
      cout << "[shortcut][path " << path_id << "][iter " << k << "][i=" << i
           << "/" << dis_path.size() << "] blocked from ("
           << short_path.back().x() << "," << short_path.back().y() << ") to ("
           << dis_path[i].x() << "," << dis_path[i].y() << "), colli_pt: ("
           << colli_pt.x() << "," << colli_pt.y() << ")" << endl;

      // initial yaw angle
      double initial_yaw = atan2(colli_pt.y() - short_path.back().y(),
                                 colli_pt.x() - short_path.back().x());

      while (initial_yaw > M_PI)
        initial_yaw -= 2 * M_PI;
      while (initial_yaw < -M_PI)
        initial_yaw += 2 * M_PI;

      // desitination yaw angle; at the last point there is no next point, so
      // keep the direction of the final segment
      const Eigen::Vector2d dest_dir =
          (i + 1 < dis_path.size())
              ? Eigen::Vector2d(dis_path[i + 1] - dis_path[i])
              : Eigen::Vector2d(dis_path[i] - dis_path[i - 1]);
      double dest_yaw = atan2(dest_dir.y(), dest_dir.x());

      while (dest_yaw > M_PI)
        dest_yaw -= 2 * M_PI;
      while (dest_yaw < -M_PI)
        dest_yaw += 2 * M_PI;

      // collision yaw: halfway along the shorter arc, so headings either side
      // of +-pi (e.g. 170 and -170 deg) average to 180 deg, not 0
      double yaw_diff = dest_yaw - initial_yaw;
      while (yaw_diff > M_PI)
        yaw_diff -= 2 * M_PI;
      while (yaw_diff < -M_PI)
        yaw_diff += 2 * M_PI;
      double colli_yaw = initial_yaw + yaw_diff / 2;

      while (colli_yaw > M_PI)
        colli_yaw -= 2 * M_PI;
      while (colli_yaw < -M_PI)
        colli_yaw += 2 * M_PI;

      cout << "[shortcut][path " << path_id << "][iter " << k << "][i=" << i
           << "] initial_yaw: " << initial_yaw << ", dest_yaw: " << dest_yaw
           << ", colli_yaw: " << colli_yaw << endl;

      // yaw =1.5708;
      Eigen::Vector2d pushed_pt;
      int push_yaw_idx;
      bool push_ok = searchValidatedPushPoint(
          colli_pt, colli_yaw, short_path.back(), dis_path[i], pushed_pt,
          push_yaw_idx);

      cout << "[shortcut][path " << path_id << "][iter " << k << "][i=" << i
           << "] push " << (push_ok ? "accepted" : "failed") << ": ("
           << pushed_pt.x() << "," << pushed_pt.y() << ")" << endl;

      if (push_ok) {
        short_path.push_back(pushed_pt);
      } else {
        // Every candidate push failed validation: keep following the already
        // collision-checked discretized path instead of inserting an unsafe
        // point. dis_path[i-1] had verified line-of-sight from the current
        // anchor (its check passed last loop step), so it is a safe anchor.
        Eigen::Vector2d fallback_pt = dis_path[i - 1];
        if ((fallback_pt - short_path.back()).norm() < 1e-9) {
          // Anchor already sits on dis_path[i-1]; step to dis_path[i] to keep
          // making progress along the original polyline.
          fallback_pt = dis_path[i];
          cout << "[shortcut][path " << path_id << "][iter " << k << "][i=" << i
               << "] WARNING: all pushes failed and anchor == dis_path[i-1]; "
                  "stepping along original path despite blocked segment"
               << endl;
        } else {
          cout << "[shortcut][path " << path_id << "][iter " << k << "][i=" << i
               << "] all pushes failed validation, falling back to original "
                  "path point ("
               << fallback_pt.x() << "," << fallback_pt.y() << ")" << endl;
        }
        short_path.push_back(fallback_pt);
      }
    }
    short_path.push_back(dis_path.back());

    /* break if no shortcut */
    double len1 = pathLength(last_path);
    double len2 = pathLength(short_path);
    cout << "[shortcut][path " << path_id << "][iter " << k
         << "] short_path size: " << short_path.size() << ", len1: " << len1
         << ", len2: " << len2 << ", ratio: " << (len2 / len1) << endl;
    if (len2 > len1 * 1.3) {
      cout << "[shortcut][path " << path_id << "][iter " << k
           << "] len2 > len1*1.3, REVERTING to last_path and stopping" << endl;
      short_path = last_path;
      break;
    }
  }

  //   short_paths_[path_id] = short_path;
  // short_path.clear();

  return short_path;
}

vector<Eigen::Vector2d>
TopoPath::discretizeLine_fastplanner(Eigen::Vector2d p1, Eigen::Vector2d p2) {
  Eigen::Vector2d dir = p2 - p1;
  double len = dir.norm();
  int seg_num = ceil(len / resolution_);

  vector<Eigen::Vector2d> line_pts;
  if (seg_num <= 0) {
    return line_pts;
  }

  for (int i = 0; i <= seg_num; ++i)
    line_pts.push_back(p1 + dir * double(i) / double(seg_num));

  return line_pts;
}

vector<Eigen::Vector2d>
TopoPath::discretizePath_fastplanner(vector<Eigen::Vector2d> path) {
  vector<Eigen::Vector2d> dis_path, segment;

  if (path.size() < 2) {
    ROS_ERROR("what path? ");
    return dis_path;
  }

  for (int i = 0; i < path.size() - 1; ++i) {
    segment = discretizeLine_fastplanner(path[i], path[i + 1]);

    if (segment.size() < 1)
      continue;

    dis_path.insert(dis_path.end(), segment.begin(), segment.end());
    if (i != path.size() - 2)
      dis_path.pop_back();
  }
  return dis_path;
}

// Robot-centric ESDF at the pose (robo_pos, yaw).
// Takes every occupied cell inside an AABB around the robot, evaluates the
// robot's own SDF for it in the body frame, and keeps the worst (smallest) one.
// That worst value IS the robot-centric ESDF: negative means the robot body
// penetrates that cell. The returned gradient is the unit direction that moves
// the robot away from that cell, and margin_dis is how far it has to move so
// the body clears it.
void TopoPath::rcESDF(const Eigen::Vector2d &robo_pos, const double yaw,
                      double &margin_dis, Eigen::Vector2d &gradient) {
  margin_dis = 0.0;
  gradient.setZero();

  const Eigen::Matrix3d Rx =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d robo_pos_3d(robo_pos(0), robo_pos(1), 0.0);

  // AABB half extent: the robot bounding box (kernel_size cells wide) plus a
  // couple of cells so obstacles just outside the body are still seen.
  const double clearance = resolution_ * 2.0;
  const double half_extent = 0.5 * (kernel_size - 1) * resolution_ + clearance;
  const int half_cells = std::ceil(half_extent / resolution_);

  const Eigen::Vector2i center =
      (environment->occupancy_map)->getGridIndex2d(robo_pos);

  double min_sdf = std::numeric_limits<double>::max();
  Eigen::Vector2d worst_pt = Eigen::Vector2d::Zero();

  // Direction is accumulated over every cell the body touches, not read off the
  // single worst cell: that one sits near the body centroid, i.e. on the
  // shape's medial axis, where the finite-difference gradient degenerates.
  Eigen::Vector2d push_acc = Eigen::Vector2d::Zero();
  Eigen::Vector2d geo_acc = Eigen::Vector2d::Zero();
  int contributing = 0;

  for (int di = -half_cells; di <= half_cells; ++di) {
    for (int dj = -half_cells; dj <= half_cells; ++dj) {
      const Eigen::Vector2i idx(center(0) + di, center(1) + dj);
      if (!(environment->occupancy_map)->isIndexOccupied2d(idx)) {
        continue;
      }

      const Eigen::Vector2d pt =
          (environment->occupancy_map)->getGridCubeCenter2d(idx);

      // SDF and body-frame gradient of the cell in one query.
      const Eigen::Vector3d pt_rel = sv_manager->posEva2Rel(
          Eigen::Vector3d(pt(0), pt(1), 0.0), robo_pos_3d, Rx);
      Eigen::Vector3d grad_rel;
      const double sdf = robot_shape->getSDFwithGrad1(pt_rel, grad_rel);

      if (sdf < min_sdf) {
        min_sdf = sdf;
        worst_pt = pt;
      }

      // Cells deeper inside the body push harder; cells up to one clearance
      // band outside still steer the escape direction. Beyond that, no say.
      const double weight = clearance - sdf;
      if (weight <= 0.0) {
        continue;
      }

      // d/d(robot pos) of sdf(R^T (p_obs - x)) is -R * grad, so -R*grad is the
      // direction that increases the clearance to this cell. Kept unnormalized:
      // near-medial-axis cells have a small gradient and damp themselves out.
      const Eigen::Vector3d grad_w = Rx * grad_rel;
      // accumulate push direction
      push_acc += weight * Eigen::Vector2d(-grad_w(0), -grad_w(1));

      const Eigen::Vector2d away = robo_pos - pt;
      if (away.norm() > 1e-9) {
        // takes the vector from the robot's center, ignore the shape
        geo_acc += weight * away.normalized();
      }
      ++contributing;
    }
  }

  // No obstacle in the box, or the body already clears everything.
  if (min_sdf == std::numeric_limits<double>::max() || min_sdf >= 0.0) {
    return;
  }

  // If the shape gradients cancel (body buried symmetrically), fall back to
  // pushing away from the centre of mass of the offending cells.
  Eigen::Vector2d push_dir = push_acc;
  if (push_dir.norm() < 1e-6) {
    push_dir = geo_acc;
    if (push_dir.norm() < 1e-6) {
      return;
    }
  }

  gradient = push_dir.normalized();
  margin_dis = -min_sdf + clearance;

  cout << "[rcESDF] pos: " << robo_pos.transpose() << " yaw: " << yaw
       << " worst cell: " << worst_pt.transpose() << " rc-esdf: " << min_sdf
       << " cells: " << contributing << " margin_dis: " << margin_dis
       << " push_dir: " << gradient.transpose() << endl;
}

void TopoPath::forestDebug(
    std::vector<std::vector<Eigen::Vector2d>> raw_paths,
    std::vector<std::vector<Eigen::Vector2d>> short_paths) {
  for (int i = 0; i < raw_paths.size(); i++) {
    PathNode::Ptr pathObj = PathNode::Ptr(new PathNode());
    pathObj->path = raw_paths[i];
    pathObj->pathID = i;
    rawpathNodes_.push_back(pathObj);
    std::cout << "path is: " << pathObj->pathID << std::endl;
    for (const auto &pt : pathObj->path) {
      std::cout << "x: " << pt.x() << " y: " << pt.y() << std::endl;
    }
  }

  for (int i = 0; i < short_paths.size(); i++) {
    PathNode::Ptr pathObj = PathNode::Ptr(new PathNode());
    pathObj->path = short_paths[i];
    pathObj->pathID = i;
    shortpathNodes_.push_back(pathObj);
    std::cout << "path is: " << pathObj->pathID << std::endl;
    for (const auto &pt : pathObj->path) {
      std::cout << "x: " << pt.x() << " y: " << pt.y() << std::endl;
    }
  }
}

void TopoPath::findPathsWithWaypoints(
    const Eigen::Vector2d &start, const Eigen::Vector2d &end,
    const std::vector<std::vector<Eigen::Vector2d>>
        &path_waypoints, // Now takes a vector of waypoint sets
    std::vector<PathNode::Ptr> &raw_paths) {
  raw_paths_.clear();
  Eigen::Vector2d collid_pt;
  int pathID = 0;

  // Process each set of waypoints
  for (const auto &waypoints : path_waypoints) {
    // Create path starting from start point
    std::vector<Eigen::Vector2d> path;
    path.push_back(start);

    // Add all waypoints in order for this path
    for (const auto &waypoint : waypoints) {
      path.push_back(waypoint);
    }

    // Add end point
    path.push_back(end);

    // Create PathNode and add to results
    PathNode::Ptr pathObj = std::make_shared<PathNode>();
    pathObj->path = path;
    pathObj->pathID = pathID++;
    raw_paths.push_back(pathObj);

    // Also add to raw_paths_ vector for compatibility
    raw_paths_.push_back(path);

    std::cout << "[Manual Waypoint Path] Generated path " << pathID
              << " through " << waypoints.size() << " waypoints" << std::endl;
  }

  // Print debug information for all paths
  for (size_t path_idx = 0; path_idx < raw_paths.size(); ++path_idx) {
    std::cout << "\nPath " << path_idx + 1 << " points:" << std::endl;
    const auto &path = raw_paths[path_idx]->path;
    for (size_t pt_idx = 0; pt_idx < path.size(); ++pt_idx) {
      if (pt_idx == 0) {
        std::cout << "Start: (";
      } else if (pt_idx == path.size() - 1) {
        std::cout << "End: (";
      } else {
        std::cout << "Waypoint " << pt_idx << ": (";
      }
      std::cout << path[pt_idx].x() << ", " << path[pt_idx].y() << ")"
                << std::endl;
    }
  }
}

vector<Eigen::Vector2i>
TopoPath::extractCollisionPoints_fastcheck(const Eigen::Vector2d &grid_point,
                                           const int yaw_idx) {
  vector<Eigen::Vector2i> collision_points;

  Eigen::Vector2i grid_index =
      environment->occupancy_map->getGridIndex2d(grid_point);
  int half_size = (kernel_size - 1) / 2;

  bool first_collision =
      true; // Track if this is the first collision point found

  for (int i = 0; i < kernel_size; i++) {
    for (int j = 0; j < kernel_size; j++) {
      if (robot_shape->byte_shape_kernels[yaw_idx].getOccupied(i, j)) {
        Eigen::Vector2i rel_grid_pos(grid_index(0) + (i - half_size),
                                     grid_index(1) + (j - half_size));

        if (environment->occupancy_map->isIndexOccupied2d(rel_grid_pos)) {
          //   if (first_collision) {
          //     std::cout
          //         << "\n[COLLISION] First collision detected at world point "
          //         << formatPoint(grid_point) << std::endl;
          //     cout << "the index is:" << grid_index(0) << " " <<
          //     grid_index(1)
          //          << endl;
          //     first_collision = false;
          //   }

          Eigen::Vector2d world_pos =
              environment->occupancy_map->getGridCubeCenter2d(rel_grid_pos);
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

// Steers colli_pt away from an obstacle and returns the closest validated
// free position: PushAway() (rcESDF) only suggests a direction, since with a
// hollow obstacle its gradient sign can point into the obstacle interior. So
// candidates are searched radius-first (smallest displacement that validates
// wins) over a fan of directions around that gradient (needed at concave
// corners, where the averaged gradient points diagonally into the corner and
// no distance along it is ever free), and only accepted once the candidate
// itself is unoccupied, the robot shape there is collision-free, and it has
// line of sight to both prev_pt and next_pt.
bool TopoPath::searchValidatedPushPoint(
    const Eigen::Vector2d &colli_pt, double colli_yaw,
    const Eigen::Vector2d &prev_pt, const Eigen::Vector2d &next_pt,
    Eigen::Vector2d &pushed_pt, int &pushed_yaw_idx,
    const std::set<std::pair<int, int>> *exclude_cells) {
  double min_safety_dist;
  Eigen::Vector2d push_dir;
  rcESDF(colli_pt, colli_yaw, min_safety_dist, push_dir);

  const double yaw_res = 2 * M_PI / kernel_count;
  pushed_yaw_idx = std::round((colli_yaw + M_PI) / yaw_res);
  if (pushed_yaw_idx == kernel_count) {
    pushed_yaw_idx = 0;
  }

  Eigen::Vector2d seed_dir = push_dir;
  if (seed_dir.norm() < 1e-6) {
    seed_dir = Eigen::Vector2d(1.0, 0.0); // fan covers 360 deg anyway
  } else {
    seed_dir.normalize();
  }

  // Never need to move further than the body half-extent plus a margin.
  const int radius_steps = std::ceil(0.5 * (kernel_size - 1)) + 2;
  static const double kFanDeg[] = {0,   30,  -30,  60,  -60,  90,
                                   -90, 120, -120, 150, -150, 180};

  // Direction of travel through this point: candidates that land behind
  // prev_pt would fold the repaired path back on itself (zigzag), so they
  // are rejected even though they can satisfy both LOS checks.
  const Eigen::Vector2d travel_dir = next_pt - prev_pt;

  // Start the scan at the clearance rcESDF says the collision needs.
  // Nearer candidates along the gradient cannot be free, and nearer
  // tangential ones are the barely-free points that make SegAdjust
  // oscillate: accepted with ~zero real clearance, the neighboring samples
  // still collide, and the next (deterministic) search re-derives the same
  // candidate, burning the push budget without progress.
  const int r_start =
      std::max(1, (int)std::ceil(min_safety_dist / resolution_));

  // Validate the candidate the same way the final path will be validated:
  // fast_check accepts a sample when a free kernel exists within a small
  // search (depth 2) around the travel yaw, so demand exactly that at the
  // yaws the robot actually holds here -- arriving (prev -> cand) and
  // departing (cand -> next). Exact-kernel tests at those headings are too
  // strict (they reject good lateral candidates whose hop angle is steep)
  // while the old snapshot colli_yaw test was too loose (it certified
  // waypoints infeasible at both real travel headings, poisoning every
  // repair anchored on them).
  auto validate = [&](const Eigen::Vector2d &cand, int &free_idx) {
    if (travel_dir.squaredNorm() > 1e-12 &&
        (cand - prev_pt).dot(travel_dir) <= 0.0) {
      return false;
    }
    if (exclude_cells) {
      const Eigen::Vector2i cg =
          environment->occupancy_map->getGridIndex2d(cand);
      if (exclude_cells->count({cg.x(), cg.y()})) {
        return false;
      }
    }
    if (environment->occupancy_map->isCoordOccupied2d(cand)) {
      return false;
    }
    const double arrive_yaw =
        atan2(cand.y() - prev_pt.y(), cand.x() - prev_pt.x());
    int arrive_idx = std::round((arrive_yaw + M_PI) / yaw_res);
    if (arrive_idx == kernel_count) {
      arrive_idx = 0;
    }
    const double depart_yaw =
        atan2(next_pt.y() - cand.y(), next_pt.x() - cand.x());
    int depart_idx = std::round((depart_yaw + M_PI) / yaw_res);
    if (depart_idx == kernel_count) {
      depart_idx = 0;
    }
    const Eigen::Vector2i cand_grid =
        environment->occupancy_map->getGridIndex2d(cand);
    const Eigen::Vector3i cand_idx(cand_grid.x(), cand_grid.y(), 0);
    bool pose_ok =
        sv_manager->visit_kernels_by_distance(free_idx, arrive_idx, cand_idx, 2);
    if (!pose_ok && arrive_idx != depart_idx) {
      pose_ok = sv_manager->visit_kernels_by_distance(free_idx, depart_idx,
                                                      cand_idx, 2);
    }
    if (!pose_ok) {
      return false;
    }
    if (!lineOfSight(prev_pt, cand)) {
      return false;
    }
    if (!lineOfSight(cand, next_pt)) {
      return false;
    }
    return true;
  };

  // Optional extra clearance: the ring scan stops at the first feasible
  // candidate, which is the minimum move that clears the collision and hugs
  // the obstacle. Where the surroundings allow it, keep stepping along the
  // accepted direction so the waypoint ends up in open space instead. Each
  // extra step is validated like the first, so a corridor that cannot spare
  // the room simply keeps the near candidate. Zero reproduces the old
  // minimum-push behaviour.
  const int extra_steps =
      std::max(0, (int)std::floor(push_extra_dist_ / resolution_ + 1e-9));

  for (int r = r_start; r <= r_start + radius_steps; ++r) {
    const double dist = r * resolution_;
    for (double deg : kFanDeg) {
      const Eigen::Vector2d dir =
          Eigen::Rotation2Dd(deg * M_PI / 180.0) * seed_dir;
      const Eigen::Vector2d cand = colli_pt + dist * dir;
      int free_idx;
      if (!validate(cand, free_idx)) {
        continue;
      }

      Eigen::Vector2d best_pt = cand;
      int best_idx = free_idx;
      for (int extra = 1; extra <= extra_steps; ++extra) {
        const Eigen::Vector2d far_cand =
            colli_pt + (dist + extra * resolution_) * dir;
        int far_idx;
        if (!validate(far_cand, far_idx)) {
          break;
        }
        best_pt = far_cand;
        best_idx = far_idx;
      }
      if (extra_steps > 0) {
        cout << "[push] min move " << dist << " m, promoted to "
             << (best_pt - colli_pt).norm() << " m" << endl;
      }

      pushed_pt = best_pt;
      pushed_yaw_idx = best_idx;
      return true;
    }
  }

  return false;
}