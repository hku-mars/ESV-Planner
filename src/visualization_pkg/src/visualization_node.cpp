#include <visualization_pkg/visualization_node.h>

PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh) {
    node = nh;

    // latched (latch=true) so late-connecting subscribers (e.g. Foxglove opening a topic
    // after a one-shot planning cycle already published) still get the last markers.
    //
    // NOTE: latching replays only the LAST message on a topic, so any topic that has to show
    // several markers at once is advertised as a MarkerArray and gets the complete set published
    // in one message. Publishing markers one-by-one on a latched Marker topic loses everything
    // except the final marker for a subscriber that connects afterwards -- and foxglove_bridge
    // subscribes lazily (only when a panel enables the topic), so that is the normal case here.
    // pub ids 1, 7 and 8 are MarkerArray; the rest are plain Marker.
    traj_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 20, true);
    pubs_.push_back(traj_pub_);

    topo_pub_ = node.advertise<visualization_msgs::MarkerArray>("/planning_vis/topo_path", 20, true);
    pubs_.push_back(topo_pub_);

    predict_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/prediction", 20, true);
    pubs_.push_back(predict_pub_);

    visib_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/visib_constraint", 20, true);
    pubs_.push_back(visib_pub_);

    frontier_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 20, true);
    pubs_.push_back(frontier_pub_);

    yaw_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/yaw", 20, true);
    pubs_.push_back(yaw_pub_);

    sample_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/sample_points", 20, true);
    pubs_.push_back(sample_pub_);

    topo_raw_pub_ = node.advertise<visualization_msgs::MarkerArray>("/planning_vis/topo_path_raw", 20, true);
    pubs_.push_back(topo_raw_pub_);

    fast_check_pub_ = node.advertise<visualization_msgs::MarkerArray>("/planning_vis/fast_check_path", 20, true);
    pubs_.push_back(fast_check_pub_);

    fast_check_grid_pub_ = node.advertise<visualization_msgs::MarkerArray>("/planning_vis/fast_check_grid", 20, true);
    pubs_.push_back(fast_check_grid_pub_);

    sample_region_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/sample_region", 20, true);
    pubs_.push_back(sample_region_pub_);

    last_topo_path1_num_ = 0;
    last_topo_path2_num_ = 0;
    last_bspline_phase1_num_ = 0;
    last_bspline_phase2_num_ = 0;
    last_frontier_num_ = 0;
    last_fast_check_num_ = 0;
    last_fast_check_grid_num_ = 0;
}

namespace {
// pub ids whose topics are advertised as visualization_msgs::MarkerArray (see the constructor)
bool isMarkerArrayPub(int pub_id) {
    return pub_id == 1 || pub_id == 7 || pub_id == 8 || pub_id == 9;
}
}  // namespace

visualization_msgs::Marker PlanningVisualization::makeLineListMarker(
    const vector<Eigen::Vector2d>& list1, const vector<Eigen::Vector2d>& list2, double line_width, const Eigen::Vector4d& color, int id, const std::string& ns) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = ns;
    mk.id = id;
    mk.type = visualization_msgs::Marker::LINE_LIST;

    if (list1.empty() || list2.empty()) {
        mk.action = visualization_msgs::Marker::DELETE;
        return mk;
    }

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = line_width;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(std::min(list1.size(), list2.size())); ++i) {
        pt.x = list1[i](0);
        pt.y = list1[i](1);
        pt.z = 0;
        mk.points.push_back(pt);

        pt.x = list2[i](0);
        pt.y = list2[i](1);
        pt.z = 0;
        mk.points.push_back(pt);
    }
    return mk;
}

visualization_msgs::Marker PlanningVisualization::makeSphereListMarker(const vector<Eigen::Vector2d>& list, double resolution, const Eigen::Vector4d& color, int id, const std::string& ns) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = ns;
    mk.id = id;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;

    if (list.empty()) {
        mk.action = visualization_msgs::Marker::DELETE;
        return mk;
    }

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); ++i) {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = 0;
        mk.points.push_back(pt);
    }
    return mk;
}

visualization_msgs::Marker PlanningVisualization::makeGridCubeListMarker(
    const vector<Eigen::Vector3d>& points, const vector<bool>& is_red, double resolution, int id, const std::string& ns) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = ns;
    mk.id = id;
    mk.type = visualization_msgs::Marker::CUBE_LIST;

    if (points.empty()) {
        mk.action = visualization_msgs::Marker::DELETE;
        return mk;
    }

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = mk.scale.y = mk.scale.z = resolution;

    for (size_t i = 0; i < points.size(); ++i) {
        geometry_msgs::Point pt;
        pt.x = points[i](0);
        pt.y = points[i](1);
        pt.z = 0;
        mk.points.push_back(pt);

        std_msgs::ColorRGBA color;
        color.a = 0.8;
        if (is_red[i]) {
            color.r = 1.0;
        } else {
            color.g = 1.0;
        }
        mk.colors.push_back(color);
    }
    return mk;
}

// publish a single marker, wrapping it in a one-element array when the topic is a MarkerArray
void PlanningVisualization::publishMarker(const visualization_msgs::Marker& mk, int pub_id) {
    if (isMarkerArrayPub(pub_id)) {
        visualization_msgs::MarkerArray arr;
        arr.markers.push_back(mk);
        pubs_[pub_id].publish(arr);
    } else {
        pubs_[pub_id].publish(mk);
    }
}

void PlanningVisualization::displaySphereList(const vector<Eigen::Vector2d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id) {
    publishMarker(makeSphereListMarker(list, resolution, color, id, ""), pub_id);
}
void PlanningVisualization::displayCubeList(const vector<Eigen::Vector2d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::CUBE_LIST;
    mk.action = visualization_msgs::Marker::DELETE;
    mk.id = id;
    pubs_[pub_id].publish(mk);

    if (list.empty()) {
        return;
    }

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++) {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = 0;
        mk.points.push_back(pt);
    }
    pubs_[pub_id].publish(mk);

    ros::Duration(0.001).sleep();
}
// void PlanningVisualization::displayCoordinateList(const vector<Eigen::Vector2d>& coordinate_positions, int id, int pub_id) {
//   // Define text markers
//   visualization_msgs::Marker text_mk;
//   text_mk.header.frame_id = "map";
//   text_mk.header.stamp    = ros::Time::now();
//   text_mk.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
//   text_mk.action          = visualization_msgs::Marker::DELETE;
//   text_mk.id              = id;
//   pubs_[pub_id].publish(text_mk);

//   text_mk.action             = visualization_msgs::Marker::ADD;
//   text_mk.pose.orientation.w = 1.0; // Assuming no rotation

//   // Set text color
//   text_mk.color.r = 1.0; // Red
//   text_mk.color.g = 1.0; // Green
//   text_mk.color.b = 1.0; // Blue
//   text_mk.color.a = 1.0; // Alpha (transparency)

//   text_mk.scale.z = 0.2; // Text size

//   // Publish text markers
//   for (size_t i = 0; i < coordinate_positions.size(); ++i) {
//     const auto& position = coordinate_positions[i];

//     geometry_msgs::Point pt;
//     pt.x = position(0);
//     pt.y = position(1);
//     pt.z = 0;
//     text_mk.pose.position = pt;

//     // Create text in the format (x,y)
//     std::stringstream ss;
//     ss << "(" << position(0) << ", " << position(1) << ")"; // Example text: "(x, y)"
//     text_mk.text = ss.str();

//     pubs_[pub_id].publish(text_mk); // Publish text marker
//   }
//     ros::Duration(0.001).sleep();
// }
void PlanningVisualization::displayCoordinateList(const vector<Eigen::Vector2d>& coordinate_positions, int id, int pub_id) {
    // Define base marker properties
    visualization_msgs::Marker base_marker;
    base_marker.header.frame_id = "map";
    base_marker.header.stamp = ros::Time::now();
    base_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    base_marker.pose.orientation.w = 1.0;  // Assuming no rotation

    // Set text color
    base_marker.color.r = 1.0;  // Red
    base_marker.color.g = 1.0;  // Green
    base_marker.color.b = 1.0;  // Blue
    base_marker.color.a = 1.0;  // Alpha (transparency)

    base_marker.scale.z = 0.1;  // Text size

    // Publish text markers for each coordinate position
    for (size_t i = 0; i < coordinate_positions.size(); ++i) {
        visualization_msgs::Marker text_marker = base_marker;  // Copy base marker properties
        text_marker.id = id + i;                               // Unique ID for each marker

        const auto& position = coordinate_positions[i];
        geometry_msgs::Point pt;
        pt.x = 1;
        pt.y = 1;
        pt.z = 1;
        text_marker.pose.position = pt;

        // Create text in the format (x, y)
        std::stringstream ss;
        ss << "(" << position(0) << ", " << position(1) << ")";  // Example text: "(x, y)"
        text_marker.text = ss.str();

        pubs_[pub_id].publish(text_marker);  // Publish text marker
    }
}

void PlanningVisualization::displayLineList(
    const vector<Eigen::Vector2d>& list1, const vector<Eigen::Vector2d>& list2, double line_width, const Eigen::Vector4d& color, int id, int pub_id, const std::string& ns, double lifetime) {
    publishMarker(makeLineListMarker(list1, list2, line_width, color, id, ns), pub_id);
}

void PlanningVisualization::displayCylinder(const Eigen::Vector2d& pos, double resolution, const Eigen::Vector4d& color, int id, int pub_id) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::CYLINDER;
    mk.action = visualization_msgs::Marker::DELETE;
    mk.id = id;

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1.0;

    mk.pose.position.x = pos(0);
    mk.pose.position.y = pos(1);
    mk.pose.position.z = resolution / 2.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    pubs_[pub_id].publish(mk);
}

void PlanningVisualization::displayLineStrip(const std::vector<Eigen::Vector2d>& points, double resolution, const Eigen::Vector4d& color, int id, int pub_id) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "line_strip";
    mk.id = id;
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.action = visualization_msgs::Marker::ADD;

    // Set the orientation (not necessary for LINE_STRIP but good practice)
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;

    // Set scale
    mk.scale.x = resolution / 10;  // Width of the line

    // Set color
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);  // Make sure alpha is not zero to make the line visible

    // Add points to the line strip
    for (const auto& point : points) {
        geometry_msgs::Point p;
        p.x = point(0);
        p.y = point(1);
        p.z = resolution / 2.0;  // Elevate the line slightly above the ground plane
        mk.points.push_back(p);
    }

    // Publish the marker
    pubs_[pub_id].publish(mk);
}

// Pack one path set into a single MarkerArray: every path gets its own namespace ("path_<i>") and
// its own marker id, plus DELETE markers for the namespaces left over from a previous, longer set.
// Publishing the whole set as one message is what makes the latch useful -- a subscriber that
// connects later receives all the paths, not just the last marker that happened to go out.
void PlanningVisualization::fillPathMarkers(
    const vector<vector<Eigen::Vector2d>>& paths, double line_width, int base_id, int last_num, double alpha, visualization_msgs::MarkerArray& arr) {
    for (int i = 0; i < int(paths.size()); ++i) {
        vector<Eigen::Vector2d> edge_pt1, edge_pt2;

        for (int j = 0; j + 1 < int(paths[i].size()); ++j) {
            edge_pt1.push_back(paths[i][j]);
            edge_pt2.push_back(paths[i][j + 1]);
        }

        arr.markers.push_back(makeLineListMarker(edge_pt1, edge_pt2, line_width, getColor(double(i) / paths.size(), alpha), base_id + i % 100, "path_" + std::to_string(i)));
    }

    const Eigen::Vector4d white(1, 1, 1, 1);
    const vector<Eigen::Vector2d> empty;
    for (int i = int(paths.size()); i < last_num; ++i) {
        arr.markers.push_back(makeLineListMarker(empty, empty, line_width, white, base_id + i % 100, "path_" + std::to_string(i)));
    }
}

void PlanningVisualization::drawTopoGraph(
    list<GraphNode::Ptr>& graph, double point_size, double line_width, const Eigen::Vector4d& color1, const Eigen::Vector4d& color2, const Eigen::Vector4d& color3, int id) {
    /* draw graph node */
    vector<Eigen::Vector2d> guards, connectors;
    for (list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter) {
        if ((*iter)->type_ == GraphNode::Guard) {
            guards.push_back((*iter)->pos_);
        } else if ((*iter)->type_ == GraphNode::Connector) {
            connectors.push_back((*iter)->pos_);
        }
    }

    /* draw graph edge */
    vector<Eigen::Vector2d> edge_pt1, edge_pt2;
    for (list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter) {
        for (int k = 0; k < (*iter)->neighbors_.size(); ++k) {
            edge_pt1.push_back((*iter)->pos_);
            edge_pt2.push_back((*iter)->neighbors_[k]->pos_);
        }
    }

    // an empty list yields a DELETE marker, so this also clears what was drawn last time
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(makeSphereListMarker(guards, point_size, color1, GRAPH_NODE, "graph_guard"));
    arr.markers.push_back(makeSphereListMarker(connectors, point_size, color2, GRAPH_NODE + 50, "graph_connector"));
    arr.markers.push_back(makeLineListMarker(edge_pt1, edge_pt2, line_width, color3, GRAPH_EDGE, "graph_edge"));
    pubs_[1].publish(arr);
}

void PlanningVisualization::drawTopoPathsPhase2(vector<vector<Eigen::Vector2d>>& paths, double line_width) {
    // topic: /planning_vis/topo_path (pub_id 1, MarkerArray) -- post-shortcut paths, one namespace per path
    visualization_msgs::MarkerArray arr;
    fillPathMarkers(paths, line_width, SELECT_PATH, last_topo_path1_num_, 1.0, arr);
    last_topo_path1_num_ = paths.size();
    pubs_[1].publish(arr);
}

void PlanningVisualization::drawTopoPathsPhase1(vector<vector<Eigen::Vector2d>>& paths, double size) {
    // topic: /planning_vis/topo_path_raw (pub_id 7, MarkerArray) -- pre-shortcut paths, one namespace per path
    visualization_msgs::MarkerArray arr;
    fillPathMarkers(paths, size, FILTERED_PATH, last_topo_path2_num_, 0.5, arr);
    last_topo_path2_num_ = paths.size();
    pubs_[7].publish(arr);
}

void PlanningVisualization::drawFastCheckPath(vector<vector<Eigen::Vector2d>>& paths, double line_width) {
    // topic: /planning_vis/fast_check_path (pub_id 8, MarkerArray) -- path(s) currently being fast-checked,
    // kept separate from drawTopoPathsPhase2 so it doesn't clobber the topo-search candidate display.
    visualization_msgs::MarkerArray arr;
    fillPathMarkers(paths, line_width, SELECT_PATH, last_fast_check_num_, 1.0, arr);
    last_fast_check_num_ = paths.size();
    pubs_[8].publish(arr);
}

void PlanningVisualization::drawFastCheckGrid(const vector<vector<Eigen::Vector3d>>& paths, const vector<vector<bool>>& is_red, double resolution) {
    // topic: /planning_vis/fast_check_grid (pub_id 9, MarkerArray) -- discretized fast_check points as
    // small grid cubes, one CUBE_LIST namespace per path (mirrors fillPathMarkers), green = free/low-risk,
    // red = occupied/high-risk.
    visualization_msgs::MarkerArray arr;
    for (int i = 0; i < int(paths.size()); ++i) {
        arr.markers.push_back(makeGridCubeListMarker(paths[i], is_red[i], resolution, 0, "grid_path_" + std::to_string(i)));
    }

    const vector<Eigen::Vector3d> empty_pts;
    const vector<bool> empty_risk;
    for (int i = int(paths.size()); i < last_fast_check_grid_num_; ++i) {
        arr.markers.push_back(makeGridCubeListMarker(empty_pts, empty_risk, resolution, 0, "grid_path_" + std::to_string(i)));
    }
    last_fast_check_grid_num_ = paths.size();

    pubs_[9].publish(arr);
}

void PlanningVisualization::drawGoal(Eigen::Vector2d goal, double resolution, const Eigen::Vector4d& color, int id) {
    displayCylinder(goal, resolution, color, GOAL + id % 100);
}

// Closed outline of the topo sampling rectangle: center +- (half_x, half_y),
// world axis aligned. One LINE_STRIP of five points (last repeats the first),
// so the region the roadmap may place guards in is visible next to the paths.
void PlanningVisualization::drawSampleRegion(const Eigen::Vector2d& center, double half_x, double half_y, double line_width, const Eigen::Vector4d& color, int id) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.action = visualization_msgs::Marker::ADD;
    mk.id = id;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = line_width;
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    const double xs[5] = {center(0) - half_x, center(0) + half_x, center(0) + half_x, center(0) - half_x, center(0) - half_x};
    const double ys[5] = {center(1) - half_y, center(1) - half_y, center(1) + half_y, center(1) + half_y, center(1) - half_y};
    for (int i = 0; i < 5; i++) {
        geometry_msgs::Point pt;
        pt.x = xs[i];
        pt.y = ys[i];
        pt.z = 0.0;
        mk.points.push_back(pt);
    }

    pubs_[10].publish(mk);
}

void PlanningVisualization::drawPath(const Eigen::Vector2d start_pt, const Eigen::Vector2d target_pt, double resolution, const Eigen::Vector4d& color, int id) {
    vector<Eigen::Vector2d> path;
    path.push_back(start_pt);
    path.push_back(target_pt);
    displayLineStrip(path, resolution, color, PATH + id % 100, 0);
}

void PlanningVisualization::drawsamplePoints0(vector<Eigen::Vector2d> points, double resolution, const Eigen::Vector4d& color, int id) {
    vector<Eigen::Vector2d> empty;
    // displayCoordinateList(empty, 600 + id % 100, 6);
    displayCubeList(empty, resolution, color, BSPLINE + id % 100, 6);

    displayCubeList(points, resolution, color, BSPLINE + id % 100, 6);
    // displayCoordinateList(points, 600 + id % 100, 6);
}

void PlanningVisualization::drawsamplePoints1(vector<Eigen::Vector2d> points, double resolution, const Eigen::Vector4d& color, int id) {
    vector<Eigen::Vector2d> empty;
    // displayCoordinateList(empty, 600 + id % 100, 6);
    displayCubeList(empty, resolution, color, BSPLINE + id % 100, 5);

    displayCubeList(points, resolution, color, BSPLINE + id % 100, 5);
    // displayCoordinateList(points, 600 + id % 100, 6);
}

Eigen::Vector4d PlanningVisualization::getColor(double h, double alpha) {
    if (h < 0.0 || h > 1.0) {
        std::cout << "h out of range" << std::endl;
        h = 0.0;
    }

    double lambda;
    Eigen::Vector4d color1, color2;
    if (h >= -1e-4 && h < 1.0 / 6) {
        lambda = (h - 0.0) * 6;
        color1 = Eigen::Vector4d(1, 0, 0, 1);
        color2 = Eigen::Vector4d(1, 0, 1, 1);

    } else if (h >= 1.0 / 6 && h < 2.0 / 6) {
        lambda = (h - 1.0 / 6) * 6;
        color1 = Eigen::Vector4d(1, 0, 1, 1);
        color2 = Eigen::Vector4d(0, 0, 1, 1);

    } else if (h >= 2.0 / 6 && h < 3.0 / 6) {
        lambda = (h - 2.0 / 6) * 6;
        color1 = Eigen::Vector4d(0, 0, 1, 1);
        color2 = Eigen::Vector4d(0, 1, 1, 1);

    } else if (h >= 3.0 / 6 && h < 4.0 / 6) {
        lambda = (h - 3.0 / 6) * 6;
        color1 = Eigen::Vector4d(0, 1, 1, 1);
        color2 = Eigen::Vector4d(0, 1, 0, 1);

    } else if (h >= 4.0 / 6 && h < 5.0 / 6) {
        lambda = (h - 4.0 / 6) * 6;
        color1 = Eigen::Vector4d(0, 1, 0, 1);
        color2 = Eigen::Vector4d(1, 1, 0, 1);

    } else if (h >= 5.0 / 6 && h <= 1.0 + 1e-4) {
        lambda = (h - 5.0 / 6) * 6;
        color1 = Eigen::Vector4d(1, 1, 0, 1);
        color2 = Eigen::Vector4d(1, 0, 0, 1);
    }

    Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
    fcolor(3) = alpha;

    return fcolor;
}