#include "map_manager/PCSmap_manager.h"
#include <pcl/io/pcd_io.h>
#define RENDER_OCCUPANCY

#define INF         0x7fffffff
#define HUGE_NUMBER 999999999
constexpr uint8_t PCSmapManager::or_mask[8];
PCSmapManager::PCSmapManager(const Config &conf) {
    recieved_globalmap = false;
    boundary_xyzmin = Vector3d(HUGE_NUMBER, HUGE_NUMBER, HUGE_NUMBER);
    boundary_xyzmax = Vector3d(-HUGE_NUMBER, -HUGE_NUMBER, -HUGE_NUMBER);
    occupancy_resolution = conf.occupancy_resolution;
    sta_threshold = conf.sta_threshold;
    debug_output = conf.debug_output;
    grid_map_sizeX = conf.mapSize_x;
    grid_map_sizeY = conf.mapSize_y;
    grid_map_sizeZ = conf.mapSize_z;

    inflation_size = conf.inflation_size;
}

PCSmapManager::~PCSmapManager() {
    // release memory
    occupancy_map->releaseMemory();
}

void PCSmapManager::init(ros::NodeHandle &nh) {
    cout << "PCSmapManager initial done" << endl;
    occupancy_map.reset(new GridMap3D);
    occupancy_map->grid_resolution = occupancy_resolution;
    occupancy_map->debug_output = debug_output;

    globalmap_sub = nh.subscribe("globalmap", 1, &PCSmapManager::rcvGlobalMapHandler, this);
    debug_grad_sub = nh.subscribe("/renderGrad", 1, &PCSmapManager::rcvRenderGrad, this);

    globalmap_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("globalmap_vis", 10);
    gridmap_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("gridmap_vis", 10);
    inflation_cells_pub = nh.advertise<sensor_msgs::PointCloud2>("inflation_gridmap_vis", 10);
    rcvmap_signal_pub = nh.advertise<std_msgs::Empty>("rcvmap_signal", 10);
    mapkernel_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("mapkernel_vis", 10);
    mapkernel_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("mapkernel_marker", 10);
    gridmap_index_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("gridmap_index_marker", 10);

    debug_grad_pub = nh.advertise<sensor_msgs::PointCloud2>("grad_vis", 10);

    republish_timer = nh.createTimer(ros::Duration(1.0), &PCSmapManager::republishTimerCallback, this);
}

void PCSmapManager::republishTimerCallback(const ros::TimerEvent&) {
    if (!recieved_globalmap) {
        return;
    }
    ros::Time now = ros::Time::now();

    globalmap_vis_msg.header.stamp = now;
    globalmap_vis_pub.publish(globalmap_vis_msg);

    gridmap_vis_msg.header.stamp = now;
    gridmap_vis_pub.publish(gridmap_vis_msg);

    inflation_cells_vis_msg.header.stamp = now;
    inflation_cells_pub.publish(inflation_cells_vis_msg);
}

void PCSmapManager::rcvRenderGrad(const std_msgs::Int16 msg) {
    // $rosrun debug grad_viewer.py
    // press 'W' and 'S' to adjust the layer, press 'B' to start rendering.
    int layer_z = msg.data;
    if (layer_z < 0) {
        layer_z = 0;
    }
    if (layer_z >= occupancy_map->Z_size) {
        layer_z = occupancy_map->Z_size - 1;
    }

    sensor_msgs::PointCloud2 grad_vis;
    pcl::PointCloud<pcl::PointXYZI> grad_vis_pcl;
    pcl::PointXYZI pt;
    Vector3d pos;
    double z = layer_z * occupancy_resolution;
    double i;

    double resolution = 0.04;
    for (double x = boundary_xyzmin(0) + 0.5; x < boundary_xyzmax(0) - 0.5; x += resolution) {
        for (double y = boundary_xyzmin(1) + 0.5; y < boundary_xyzmax(1) - 0.5; y += resolution) {
            pos = Vector3d(x, y, z);
            pt.x = x;
            pt.y = y;
            pt.z = z;  // + 0.01 * travelcost_map -> getSDFValue(pos);
            i = occupancy_map->getSDFValue(pos);
            pt.intensity = i;
            grad_vis_pcl.points.push_back(pt);
        }
    }

    pcl::toROSMsg(grad_vis_pcl, grad_vis);
    grad_vis.header.frame_id = "map";
    debug_grad_pub.publish(grad_vis);
}

void PCSmapManager::rcvGlobalMapHandler(const sensor_msgs::PointCloud2 &globalmap) {
    obs_forblender3d.clear();
    if (recieved_globalmap == true) {
        return;
    }

    sensor_msgs::PointCloud2 globalmap_vis;
    pcl::PointCloud<pcl::PointXYZ> original_obstacles_vis;
    pcl::PointCloud<pcl::PointXYZ> inflation_cells_vis;

    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    pcl::fromROSMsg(globalmap, global_cloud);

    // Set all z-values to 0
    for (size_t i = 0; i < global_cloud.points.size(); i++) {
        global_cloud.points[i].z = 0.0;
    }

    globalmap_vis = globalmap;
    globalmap_vis.header.frame_id = "map";
    globalmap_vis_pub.publish(globalmap_vis);
    globalmap_vis_msg = globalmap_vis;

    // STEP1 generate occupancy map using point cloud
    global_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(global_cloud, *global_pointcloud);
    global_kdtree.setInputCloud(global_pointcloud);

    boundary_xyzmin << -grid_map_sizeX / 2.0, -grid_map_sizeY / 2.0, 0.0;
    boundary_xyzmax << +grid_map_sizeX / 2.0, +grid_map_sizeY / 2.0, 0.2;

    occupancy_map->createGridMap(boundary_xyzmin, boundary_xyzmax);
    cout << "boundary_xyzmin: " << boundary_xyzmin << endl;
    cout << "boundary_xyzmax: " << boundary_xyzmax << endl;

    // generate occupancy grid map
    Vector3i index;
    Vector3d center_coord;
    pcl::PointXYZ s_point;
    visualization_msgs::MarkerArray marker_array;
    std::map<int, bool> marker_exists;  // Track which markers have been created

    for (size_t i = 0; i < global_cloud.points.size(); i++) {
        index = occupancy_map->getGridIndex3d(Vector3d(global_cloud.points[i].x, global_cloud.points[i].y, global_cloud.points[i].z));
        occupancy_map->grid_map[occupancy_map->toAddr(index(0), index(1), index(2))]++;
    }

    // Create a temporary map for storing original occupancy
    int total_size = occupancy_map->X_size * occupancy_map->Y_size * occupancy_map->Z_size;
    double *original_map = new double[total_size];

    // First pass: Mark occupied cells based on threshold
    for (int i = 0; i < occupancy_map->X_size; i++) {
        for (int j = 0; j < occupancy_map->Y_size; j++) {
            for (int k = 0; k < occupancy_map->Z_size; k++) {
                int addr = occupancy_map->toAddr(i, j, k);
                if (occupancy_map->grid_map[addr] >= sta_threshold) {
                    original_map[addr] = 1.0;
                    occupancy_map->grid_map[addr] = 1.0;

#ifdef RENDER_OCCUPANCY
                    // Visualize original obstacles
                    center_coord = occupancy_map->getGridCubeCenter3d(i, j, 0);
                    obs_forblender3d.emplace_back(center_coord);
                    s_point.x = center_coord(0);
                    s_point.y = center_coord(1);
                    s_point.z = center_coord(2);
                    original_obstacles_vis.push_back(s_point);

                    // Add marker for original obstacle if it doesn't exist
                    int marker_id = i * occupancy_map->Y_size + j;
                    if (!marker_exists[marker_id]) {
                        visualization_msgs::Marker marker;
                        marker.header.frame_id = "map";
                        marker.header.stamp = ros::Time::now();
                        marker.ns = "grid_map_indices";
                        marker.id = marker_id;
                        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                        marker.action = visualization_msgs::Marker::ADD;

                        marker.pose.position.x = center_coord(0);
                        marker.pose.position.y = center_coord(1);
                        marker.pose.position.z = 0.2;

                        marker.scale.x = occupancy_resolution;
                        marker.scale.y = occupancy_resolution;
                        marker.scale.z = 0.1;

                        std::stringstream ss;
                        ss << std::fixed << std::setprecision(1) << "(" << center_coord(0) << "," << center_coord(1) << ")";
                        marker.text = ss.str();

                        marker.color.r = 1.0;
                        marker.color.g = 1.0;
                        marker.color.b = 1.0;
                        marker.color.a = 1.0;

                        marker_array.markers.push_back(marker);
                        marker_exists[marker_id] = true;
                    }
#endif
                } else {
                    original_map[addr] = 0.0;
                    occupancy_map->grid_map[addr] = 0.0;
                }
            }
        }
    }

    // Second pass: Find boundary cells of obstacles and inflate only into free space
    for (int i = 0; i < occupancy_map->X_size; i++) {
        for (int j = 0; j < occupancy_map->Y_size; j++) {
            for (int k = 0; k < occupancy_map->Z_size; k++) {
                int curr_addr = occupancy_map->toAddr(i, j, k);
                if (original_map[curr_addr] > 0.5) {
                    bool is_boundary = false;
                    for (int dx = -1; dx <= 1 && !is_boundary; dx++) {
                        for (int dy = -1; dy <= 1 && !is_boundary; dy++) {
                            int ni = i + dx;
                            int nj = j + dy;

                            if (ni >= 0 && ni < occupancy_map->X_size && nj >= 0 && nj < occupancy_map->Y_size) {
                                int neighbor_addr = occupancy_map->toAddr(ni, nj, k);
                                if (original_map[neighbor_addr] < 0.5) {
                                    is_boundary = true;
                                }
                            }
                        }
                    }

                    if (is_boundary) {
                        for (int dx = -inflation_size; dx <= inflation_size; dx++) {
                            for (int dy = -inflation_size; dy <= inflation_size; dy++) {
                                int new_i = i + dx;
                                int new_j = j + dy;

                                if (new_i >= 0 && new_i < occupancy_map->X_size && new_j >= 0 && new_j < occupancy_map->Y_size) {
                                    int new_addr = occupancy_map->toAddr(new_i, new_j, k);
                                    if (original_map[new_addr] < 0.5) {
                                        occupancy_map->grid_map[new_addr] = 1.0;

#ifdef RENDER_OCCUPANCY
                                        center_coord = occupancy_map->getGridCubeCenter3d(new_i, new_j, 0);
                                        obs_forblender3d.emplace_back(center_coord);
                                        s_point.x = center_coord(0);
                                        s_point.y = center_coord(1);
                                        s_point.z = center_coord(2);
                                        inflation_cells_vis.push_back(s_point);

                                        // Add marker for inflated cell if it doesn't exist
                                        int marker_id = new_i * occupancy_map->Y_size + new_j;
                                        if (!marker_exists[marker_id]) {
                                            visualization_msgs::Marker marker;
                                            marker.header.frame_id = "map";
                                            marker.header.stamp = ros::Time::now();
                                            marker.ns = "grid_map_indices";
                                            marker.id = marker_id;
                                            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                                            marker.action = visualization_msgs::Marker::ADD;

                                            marker.pose.position.x = center_coord(0);
                                            marker.pose.position.y = center_coord(1);
                                            marker.pose.position.z = 0.2;

                                            marker.scale.x = occupancy_resolution;
                                            marker.scale.y = occupancy_resolution;
                                            marker.scale.z = 0.1;

                                            std::stringstream ss;
                                            ss << std::fixed << std::setprecision(1) << "(" << center_coord(0) << "," << center_coord(1) << ")";
                                            marker.text = ss.str();

                                            marker.color.r = 1.0;
                                            marker.color.g = 1.0;
                                            marker.color.b = 1.0;
                                            marker.color.a = 1.0;

                                            marker_array.markers.push_back(marker);
                                            marker_exists[marker_id] = true;
                                        }
#endif
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    delete[] original_map;

    ROS_INFO("[map_manager]generate occupancy map with optimized inflation done!");

#ifdef RENDER_OCCUPANCY
    // Publish original obstacles
    sensor_msgs::PointCloud2 original_obstacles_msg;
    pcl::toROSMsg(original_obstacles_vis, original_obstacles_msg);
    original_obstacles_msg.header.frame_id = "map";
    gridmap_vis_pub.publish(original_obstacles_msg);
    gridmap_vis_msg = original_obstacles_msg;

    // Publish inflation cells
    sensor_msgs::PointCloud2 inflation_cells_msg;
    pcl::toROSMsg(inflation_cells_vis, inflation_cells_msg);
    inflation_cells_msg.header.frame_id = "map";
    inflation_cells_pub.publish(inflation_cells_msg);
    inflation_cells_vis_msg = inflation_cells_msg;

    // Publish all text markers in a single array
    gridmap_index_marker_pub.publish(marker_array);
#endif

    recieved_globalmap = true;
    std_msgs::Empty s;
    rcvmap_signal_pub.publish(s);
}

// void PCSmapManager::rcvGlobalMapHandler(const sensor_msgs::PointCloud2 &globalmap) {
//     obs_forblender3d.clear();
//     if (recieved_globalmap == true) {
//         return;
//     }

//     sensor_msgs::PointCloud2 globalmap_vis;
//     pcl::PointCloud<pcl::PointXYZ> gridmap_vis;

//     pcl::PointCloud<pcl::PointXYZ> global_cloud;
//     pcl::fromROSMsg(globalmap, global_cloud);

//     // Set all z-values to 0
//     for (size_t i = 0; i < global_cloud.points.size(); i++) {
//         global_cloud.points[i].z = 0.0;
//     }

//     // for (size_t i = 0; i < global_cloud.points.size(); i++) {
//     //     cout << "received cloud point x:" << global_cloud.points[i].x << endl;
//     //     cout << "received cloud point y:" << global_cloud.points[i].y << endl;
//     //     cout << "received cloud point z:" << global_cloud.points[i].z << endl;
//     // }
//     // received cloud point x:1.301
//     // received cloud point y:2.501
//     // received cloud point z:0

//     globalmap_vis = globalmap;
//     globalmap_vis.header.frame_id = "map";
//     globalmap_vis_pub.publish(globalmap_vis);

//     // STEP1 generate occupancy map using point cloud
//     global_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::copyPointCloud(global_cloud, *global_pointcloud);
//     global_kdtree.setInputCloud(global_pointcloud);

//     // measure boundary
//     // for (size_t i = 0; i < global_cloud.points.size(); i++) {
//     //     if (global_cloud.points[i].x > boundary_xyzmax(0)) {
//     //         boundary_xyzmax(0) = global_cloud.points[i].x;
//     //     }

//     //     if (global_cloud.points[i].x < boundary_xyzmin(0)) {
//     //         boundary_xyzmin(0) = global_cloud.points[i].x;
//     //     }

//     //     if (global_cloud.points[i].y > boundary_xyzmax(1)) {
//     //         boundary_xyzmax(1) = global_cloud.points[i].y;
//     //     }

//     //     if (global_cloud.points[i].y < boundary_xyzmin(1)) {
//     //         boundary_xyzmin(1) = global_cloud.points[i].y;
//     //     }

//     //     if (global_cloud.points[i].z > boundary_xyzmax(2)) {
//     //         boundary_xyzmax(2) = global_cloud.points[i].z + 0.1;
//     //     }

//     //     if (global_cloud.points[i].z < boundary_xyzmin(2)) {
//     //         boundary_xyzmin(2) = global_cloud.points[i].z;
//     //     }
//     // }

//     // malloc 3D grid map space
//     // if the z axis of boundary_xyzmin is less than zero, then the index conversion will be wrong.
//     // for example if z = -0.2, resolution is 0.2, then the index of the map should be 0 -> -0.2, 1 -> 0, 2 -> 0.2
//     // boundary measurement 不是整数，导致index转换错误。生成的grid map与实际接受到的地图错位。

//     // for the potato_ma created map, the boundary_xyzmin should be zero. becuase the point cloud is not centerd at map center.
//     boundary_xyzmin << -grid_map_sizeX / 2.0, -grid_map_sizeY / 2.0, 0.0;
//     boundary_xyzmax << +grid_map_sizeX / 2.0, +grid_map_sizeY / 2.0, 0.2;

//     occupancy_map->createGridMap(boundary_xyzmin, boundary_xyzmax);
//     cout << "boundary_xyzmin: " << boundary_xyzmin << endl;
//     cout << "boundary_xyzmax: " << boundary_xyzmax << endl;
//     // occupancy_map->test();

//     // generate occupancy grid map
//     Vector3i index;
//     Vector3d center_coord;
//     pcl::PointXYZ s_point;
//     for (size_t i = 0; i < global_cloud.points.size(); i++) {
//         index = occupancy_map->getGridIndex3d(Vector3d(global_cloud.points[i].x, global_cloud.points[i].y, global_cloud.points[i].z));
//         occupancy_map->grid_map[occupancy_map->toAddr(index(0), index(1), index(2))]++;
//     }
//     visualization_msgs::MarkerArray marker_array;
//     // Iterates over all cells in the grid, applying a threshold (sta_threshold=1). If a cell has more points than the threshold, it’s marked as occupied (1). Otherwise, it’s marked as empty (0).
//     // cout << "Occupied index number in grid map!!" << endl;
//     for (int i = 0; i < occupancy_map->X_size; i++) {
//         for (int j = 0; j < occupancy_map->Y_size; j++) {
//             for (int k = 0; k < occupancy_map->Z_size; k++) {
//                 if (occupancy_map->grid_map[occupancy_map->toAddr(i, j, k)] >= sta_threshold) {
//                     // cout << "i: " << i << " j: " << j << " k: " << k << endl;
//                     center_coord = occupancy_map->getGridCubeCenter3d(i, j, 0);
//                     // if(std::abs(center_coord.head(2).norm())<2.0)
//                     // {
//                     //     continue;
//                     // }
// #ifdef RENDER_OCCUPANCY
//                     obs_forblender3d.emplace_back(center_coord);
//                     s_point.x = center_coord(0);
//                     s_point.y = center_coord(1);
//                     s_point.z = center_coord(2);
//                     gridmap_vis.push_back(s_point);
//                     // Create a text marker for the original index
//                     visualization_msgs::Marker marker;
//                     marker.header.frame_id = "map";
//                     marker.header.stamp = ros::Time::now();
//                     marker.ns = "grid_map_indices";
//                     marker.id = i * occupancy_map->Y_size + j;  // Unique ID for each marker
//                     marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//                     marker.action = visualization_msgs::Marker::ADD;

//                     // Set the position of the text marker slightly above the grid point
//                     marker.pose.position.x = center_coord(0);
//                     marker.pose.position.y = center_coord(1);
//                     marker.pose.position.z = 0.2;

//                     // Set the scale (size) of the text
//                     marker.scale.x = occupancy_resolution;
//                     marker.scale.y = occupancy_resolution;
//                     marker.scale.z = 0.1;  // Size of the text

//                     // Format the coordinates directly from center_coord
//                     std::stringstream ss;
//                     ss << std::fixed << std::setprecision(1) << "(" << center_coord(0) << "," << center_coord(1) << ")";
//                     marker.text = ss.str();

//                     // Set the color
//                     marker.color.r = 1.0;
//                     marker.color.g = 1.0;
//                     marker.color.b = 1.0;
//                     marker.color.a = 1.0;  // Fully opaque

//                     // Add the marker to the array
//                     marker_array.markers.push_back(marker);
// #endif
//                     // occupied
//                     occupancy_map->grid_map[occupancy_map->toAddr(i, j, k)] = 1;
//                 } else {
//                     // free
//                     occupancy_map->grid_map[occupancy_map->toAddr(i, j, k)] = 0;
//                 }
//             }
//         }
//     }

//     ROS_INFO("[map_manager]generate occupancy map done!");

// // EX STEP visualize occupancy map
// #ifdef RENDER_OCCUPANCY
//     sensor_msgs::PointCloud2 occupancy_map_vis;
//     pcl::toROSMsg(gridmap_vis, occupancy_map_vis);
//     occupancy_map_vis.header.frame_id = "map";
//     gridmap_vis_pub.publish(occupancy_map_vis);
//     // Publish the marker array for displaying indices in RViz
//     gridmap_index_marker_pub.publish(marker_array);
// #endif

//     recieved_globalmap = true;
//     std_msgs::Empty s;
//     rcvmap_signal_pub.publish(s);
// }

void PCSmapManager::occupied_test(Vector2d pt) {
    cout << "test point: " << pt.transpose() << endl;
    Vector2i index = occupancy_map->getGridIndex2d(pt);
    if (occupancy_map->isIndexOccupied2d(index)) {
        cout << "occupied" << endl;
    } else {
        cout << "free" << endl;
    }
}