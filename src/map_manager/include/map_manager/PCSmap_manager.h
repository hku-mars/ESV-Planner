#ifndef PCSMAP_MANAGER_H
#define PCSMAP_MANAGER_H

#include <string.h>
#include <ros/ros.h>
#include <unordered_map>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils/config.hpp>
#include <utils/debug_publisher.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "map_manager/GridMap3D.h"

class PCSmapManager {
public:
    PCSmapManager(const Config& conf);
    ~PCSmapManager();
    static constexpr uint8_t or_mask[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
    // or_mask[0] = 10000000
    // or_mask[1] = 01000000
    // or_mask[2] = 00100000
    // or_mask[3] = 00010000
    // or_mask[4] = 00001000
    // or_mask[5] = 00000100
    // or_mask[6] = 00000010
    // or_mask[7] = 00000001

    // param load
    void init(ros::NodeHandle& nh);

    std::vector<Eigen::Vector3d> obs_forblender3d;
    // callback functions
    void rcvGlobalMapHandler(const sensor_msgs::PointCloud2& globalmap);
    void rcvRenderGrad(const std_msgs::Int16 msg);
    void republishTimerCallback(const ros::TimerEvent& event);

    inline uint8_t* generateMapKernel(const int& kernel_size) {
        int side_size = (kernel_size - 1) / 2;
        int x_size = occupancy_map->X_size + 2 * side_size;
        int y_size = occupancy_map->Y_size + 2 * side_size;
        int z_size = occupancy_map->Z_size + 2 * side_size;
        int bytes_len_of_last_dim = (z_size + 7) / 8;
        int size_yz = y_size * bytes_len_of_last_dim;
        int bytes_len = x_size * y_size * bytes_len_of_last_dim;
        uint8_t* map_kernel = new uint8_t[bytes_len]();

        for (int x = 0; x < occupancy_map->X_size; x++) {
            for (int y = 0; y < occupancy_map->Y_size; y++) {
                for (int z = 0; z < occupancy_map->Z_size; z++) {
                    int flate_x = x + side_size;
                    int flate_y = y + side_size;
                    int flate_z = z + side_size;

                    int byte_idx = flate_x * size_yz + flate_y * bytes_len_of_last_dim + (flate_z + 0) / 8;
                    int byte_offset = flate_z % 8;
                    if (occupancy_map->isIndexOccupied3d(x, y, z) == true) {
                        map_kernel[byte_idx] |= or_mask[byte_offset];
                    }
                }
            }
        }
        mk = map_kernel;
        return map_kernel;
    }

    inline uint8_t* generateMapKernel2D(const int& kernel_size) {
        // the kernel size should be (kernel_size - 1)/2 + original occupancy map size
        int side_size = (kernel_size - 1) / 2;
        // X_size = measured boundary size / resolution
        int x_size = occupancy_map->X_size + 2 * side_size;
        int y_size = occupancy_map->Y_size + 2 * side_size;
        // number of bytes need to store the y_size(the row of this map)
        // integer division rounds down. int a = 1.5, -> a = 1
        int bytes_len_of_last_dim = (y_size + 7) / 8;
        // total bytes length of the map
        int bytes_len = x_size * bytes_len_of_last_dim;
        // map_kernel is initialized as an array of bytes_len (bytes_len x 8 as bits), all set to 0.
        // map_kernel[0] = 0b00000000
        // map_kernel[1] = 0b00000000
        // map_kernel[2] = 0b00000000
        // map_kernel[3] = 0b00000000
        // map_kernel[4] = 0b00000000
        uint8_t* map_kernel = new uint8_t[bytes_len]();

        for (int x = 0; x < occupancy_map->X_size; x++) {
            for (int y = 0; y < occupancy_map->Y_size; y++) {
                int flate_x = x + side_size;
                int flate_y = y + side_size;
                // if the grid number of y axis more than 8, the byte_idx will spend more row to store the map
                int byte_idx = flate_x * bytes_len_of_last_dim + (flate_y + 0) / 8;
                int byte_offset = flate_y % 8;
                if (occupancy_map->isIndexOccupied3d(x, y, 0) == true) {
                    // std::cout << "Setting occupied grid at Original Index (x, y): (" << x << ", " << y << ") -> "
                    //           << "Kernel Index (flate_x, flate_y): (" << flate_x << ", " << flate_y << ")\n"
                    //           << "byte_idx: " << byte_idx << ", byte_offset: " << byte_offset << std::endl;
                    // if byte_offset = 3, or_mask[3] will be 00010000
                    // map_kernel[byte_idx], the byte_idx means the row number(xth row) of the map
                    // the bitwise OR (|) and assignment (=).
                    // 	•	Bitwise OR (|): This operation compares each bit of two operands. If either of the corresponding bits of the operands is 1, the resulting bit is set to 1. Otherwise, it
                    // is set to 0.
                    map_kernel[byte_idx] |= or_mask[byte_offset];
                }
            }
        }
        mk = map_kernel;
        publishMapKernel(map_kernel, x_size, y_size, bytes_len_of_last_dim, mapkernel_vis_pub, mapkernel_marker_pub, side_size);
        // printKernelBits(map_kernel, x_size, y_size, bytes_len_of_last_dim);
        // printOccupiedGrids(map_kernel, x_size, y_size, bytes_len_of_last_dim, side_size);

        return map_kernel;
    }

    void printKernelBits(const uint8_t* kernel, int x_size, int y_size, int bytes_len_of_last_dim) {
        std::cout << "Kernel Bit Representation:" << std::endl;
        int total_bytes = x_size * bytes_len_of_last_dim;

        for (int i = 0; i < total_bytes; ++i) {
            std::cout << "map_kernel[" << i << "] = 0b";

            uint8_t byte = kernel[i];
            for (int bit = 7; bit >= 0; --bit) {
                std::cout << ((byte & (1 << bit)) ? '1' : '0');
            }

            std::cout << std::endl;

            // Add a blank line after each row (x-dimension) of the kernel
            if ((i + 1) % bytes_len_of_last_dim == 0) {
                std::cout << std::endl;
            }
        }
    }

    inline void publishMapKernel(const uint8_t* map_kernel, int x_size, int y_size, int bytes_len_of_last_dim, ros::Publisher& pub, ros::Publisher& marker_pub, int side_size) {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        visualization_msgs::MarkerArray marker_array;

        // Iterate over the entire map
        for (int flate_x = 0; flate_x < x_size; flate_x++) {
            for (int flate_y = 0; flate_y < y_size; flate_y++) {
                int byte_idx = flate_x * bytes_len_of_last_dim + flate_y / 8;
                int byte_offset = flate_y % 8;
                // wrong occupied checking
                // bool occupied = map_kernel[byte_idx] & (1 << byte_offset);
                // correct occupied checking
                bool occupied = map_kernel[byte_idx] & (1 << (7 - byte_offset));

                // Adjust the indices to map back to the original grid map
                int original_x = flate_x - side_size;
                int original_y = flate_y - side_size;

                // Define the point
                pcl::PointXYZRGB point;
                point.x = original_x;  // Use the original index for the point position
                point.y = original_y;
                point.z = 0;

                if (occupied) {
                    // Occupied cells: Red color
                    point.r = 255;
                    point.g = 0;
                    point.b = 0;

                    // Create a text marker for this occupied cell
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "map";
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "map_kernel_numbers";
                    marker.id = flate_x * y_size + flate_y;  // Unique ID for each marker
                    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    marker.action = visualization_msgs::Marker::ADD;

                    // Set the position of the text
                    marker.pose.position.x = original_x;
                    marker.pose.position.y = original_y;
                    marker.pose.position.z = 0.2;  // Slightly above the grid

                    // Set the scale (size) of the text
                    marker.scale.z = 1.0;  // Size of the text

                    // Set the text content to display the adjusted original indices
                    marker.text = std::to_string(original_x) + "," + std::to_string(original_y);

                    // Set the color
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;
                    marker.color.a = 1.0;  // Fully opaque

                    // Add the marker to the array
                    marker_array.markers.push_back(marker);
                } else {
                    // Free cells: Green color
                    point.r = 0;
                    point.g = 255;
                    point.b = 0;
                }

                cloud.points.push_back(point);
            }
        }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "map";
        output.header.stamp = ros::Time::now();

        pub.publish(output);
        marker_pub.publish(marker_array);
    }

    void printOccupiedGrids(uint8_t* map_kernel, int x_size, int y_size, int bytes_len_of_last_dim, int side_size) {
        // Iterate over the entire x dimension of the kernel
        for (int flate_x = 0; flate_x < x_size; flate_x++) {
            // Iterate over the entire y dimension of the kernel
            for (int flate_y = 0; flate_y < y_size; flate_y++) {
                // Calculate the index of the byte in the map_kernel array
                int byte_idx = flate_x * bytes_len_of_last_dim + (flate_y + 0) / 8;
                // Calculate the offset within the byte (which bit we're interested in)
                int byte_offset = flate_y % 8;

                // Extract the specific bit and check if it's set (i.e., if the grid is occupied)
                // The expression map_kernel[byte_idx] & (1 << (7 - byte_offset)) is used to check whether a specific bit in map_kernel[byte_idx] is set to 1. This is done by creating a mask using 1
                // << (7 - byte_offset) that has a single bit set in the position you’re interested in. The bitwise AND operation then checks if that particular bit in map_kernel[byte_idx] is set
                // to 1.
                if (map_kernel[byte_idx] & (1 << (7 - byte_offset))) {  // Adjust bit offset to match how bits are set
                    // Calculate the original x, y indices based on the flattened indices
                    int original_x = flate_x - side_size;
                    int original_y = flate_y - side_size;

                    // Ensure that we're within the bounds of the original map
                    if (original_x >= 0 && original_x < occupancy_map->X_size && original_y >= 0 && original_y < occupancy_map->Y_size) {
                        std::cout << "Occupied Grid - Kernel Index (flate_x, flate_y): (" << flate_x << ", " << flate_y << ")"
                                  << " -> Original Index (x, y): (" << original_x << ", " << original_y << ")" << std::endl;
                    }
                }
            }
        }
    }
    inline void clearMap() {
        if (recieved_globalmap == true) {
            occupancy_map->releaseMemory();
            recieved_globalmap = false;
        }
    }

    inline int unifiedID(const int& i, const int& j, const int& k) const {
        int unified_id = 0;
        unified_id += k * (occupancy_map->X_size) * (occupancy_map->Y_size);
        unified_id += j * (occupancy_map->X_size);
        unified_id += i;
        return unified_id;
    }

    inline void projInMap(Eigen::Vector3d& pt) const {
        if (pt(0) < boundary_xyzmin(0)) {
            pt(0) = boundary_xyzmin(0);
        }
        if (pt(1) < boundary_xyzmin(1)) {
            pt(1) = boundary_xyzmin(1);
        }
        if (pt(2) < boundary_xyzmin(2)) {
            pt(2) = boundary_xyzmin(2);
        }
        if (pt(0) > boundary_xyzmax(0)) {
            pt(0) = boundary_xyzmax(0);
        }
        if (pt(1) > boundary_xyzmax(1)) {
            pt(1) = boundary_xyzmax(1);
        }
        if (pt(2) > boundary_xyzmax(2)) {
            pt(2) = boundary_xyzmax(2);
        }
    }

    inline void getPointsInAABB2D(const Eigen::Vector3d& center, double halfbdx, double halfbdy, std::vector<Eigen::Vector2d>& ob_pts) const {
        Eigen::Vector3d corner1 = center - Eigen::Vector3d(halfbdx, halfbdy, 0);
        Eigen::Vector3d corner2 = center + Eigen::Vector3d(halfbdx, halfbdy, 0);
        corner1(2) = 0.0;
        corner2(2) = 0.0;
        projInMap(corner1);
        projInMap(corner2);
        const Eigen::Vector3i idcorner1 = occupancy_map->getGridIndex3d(corner1);
        const Eigen::Vector3i idcorner2 = occupancy_map->getGridIndex3d(corner2);

        for (int i = idcorner1(0); i <= idcorner2(0); i++) {
            for (int j = idcorner1(1); j <= idcorner2(1); j++) {
                if (occupancy_map->isIndexOccupied3d(i, j, 0)) {
                    ob_pts.emplace_back(occupancy_map->getGridCubeCenter3d(i, j, 0).head(2));  // posI2D
                }
            }
        }
    }

    inline void getPointsInAABB(const Eigen::Vector3d& center, double halfbdx, double halfbdy, double halfbdz, std::vector<Eigen::Vector3d>& ob_pts) const {
        Eigen::Vector3d corner1 = center - Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
        Eigen::Vector3d corner2 = center + Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
        projInMap(corner1);
        projInMap(corner2);
        const Eigen::Vector3i idcorner1 = occupancy_map->getGridIndex3d(corner1);
        const Eigen::Vector3i idcorner2 = occupancy_map->getGridIndex3d(corner2);

        for (int i = idcorner1(0); i <= idcorner2(0); i++) {
            for (int j = idcorner1(1); j <= idcorner2(1); j++) {
                for (int k = idcorner1(2); k <= idcorner2(2); k++) {
                    if (occupancy_map->isIndexOccupied3d(i, j, k)) {
                        ob_pts.emplace_back(occupancy_map->getGridCubeCenter3d(i, j, k));  // posI2D
                    }
                }
            }
        }
    }

    // Same box as getPointsInAABBOutOfLastOne but with no previous box to
    // subtract, for the first sample of a chain. Seeding that chain with a
    // sentinel "last" centre does not work: projInMap clamps an out-of-map
    // centre onto the map corner, so the sentinel names a real region of the
    // map and silently drops the occupied cells the first box shares with it.
    inline void getPointsInAABBIntoSet(const Eigen::Vector3d& center, double halfbdx, double halfbdy, double halfbdz) {
        Eigen::Vector3d corner1 = center - Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
        Eigen::Vector3d corner2 = center + Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
        projInMap(corner1);
        projInMap(corner2);
        const Eigen::Vector3i idcorner1 = occupancy_map->getGridIndex3d(corner1);
        const Eigen::Vector3i idcorner2 = occupancy_map->getGridIndex3d(corner2);

        for (int i = idcorner1(0); i <= idcorner2(0); i++) {
            for (int j = idcorner1(1); j <= idcorner2(1); j++) {
                for (int k = idcorner1(2); k <= idcorner2(2); k++) {
                    if (occupancy_map->isIndexOccupied3d(i, j, k)) {
                        aabb_points.emplace(unifiedID(i, j, k), occupancy_map->getGridCubeCenter3d(i, j, k));
                    }
                }
            }
        }
    }

    inline void getPointsInAABBOutOfLastOne(const Eigen::Vector3d& center, const Eigen::Vector3d& center_last, double halfbdx, double halfbdy, double halfbdz) {
        Eigen::Vector3d corner1 = center - Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
        Eigen::Vector3d corner2 = center + Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
        projInMap(corner1);
        projInMap(corner2);
        const Eigen::Vector3i idcorner1 = occupancy_map->getGridIndex3d(corner1);
        const Eigen::Vector3i idcorner2 = occupancy_map->getGridIndex3d(corner2);

        Eigen::Vector3d corner1_l = center_last - Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
        Eigen::Vector3d corner2_l = center_last + Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
        projInMap(corner1_l);
        projInMap(corner2_l);
        const Eigen::Vector3i idcorner1_l = occupancy_map->getGridIndex3d(corner1_l);
        const Eigen::Vector3i idcorner2_l = occupancy_map->getGridIndex3d(corner2_l);
        // from mid to top
        Vector3d test = occupancy_map->getGridCubeCenter3d(186, 190, 0);

        for (int i = idcorner1(0); i <= idcorner2(0); i++) {
            for (int j = idcorner1(1); j <= idcorner2(1); j++) {
                for (int k = idcorner1(2); k <= idcorner2(2); k++) {
                    if (i > idcorner2_l(0) || i < idcorner1_l(0) || j > idcorner2_l(1) || j < idcorner1_l(1) || k > idcorner2_l(2) || k < idcorner1_l(2)) {
                        if (occupancy_map->isIndexOccupied3d(i, j, k)) {
                            aabb_points.emplace(unifiedID(i, j, k), occupancy_map->getGridCubeCenter3d(i, j, k));
                        }
                    }
                }
            }
        }
    }

    // map size box
    Vector3d boundary_xyzmin;
    Vector3d boundary_xyzmax;

    // inflation size
    int inflation_size;

    // map resolutions
    double occupancy_resolution;

    // some params
    // point count threshold while generating occupancy grid map using point cloud
    int sta_threshold;
    // map received flag
    bool recieved_globalmap;
    // debug output switch
    bool debug_output;

    // global 3D occupancy grid map
    GridMap3D::Ptr occupancy_map;

    uint8_t* mk;

    std::unordered_map<int, Eigen::Vector3d> aabb_points;

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_pointcloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> global_kdtree;

    // ros
    ros::Subscriber globalmap_sub;
    ros::Subscriber debug_grad_sub;
    ros::Timer republish_timer;
    sensor_msgs::PointCloud2 globalmap_vis_msg;
    sensor_msgs::PointCloud2 gridmap_vis_msg;
    sensor_msgs::PointCloud2 inflation_cells_vis_msg;

    ros::Publisher globalmap_vis_pub;
    ros::Publisher gridmap_vis_pub;
    ros::Publisher inflation_cells_pub;
    ros::Publisher rcvmap_signal_pub;
    ros::Publisher debug_grad_pub, mapkernel_vis_pub, mapkernel_marker_pub, gridmap_index_marker_pub;


    double grid_map_sizeX, grid_map_sizeY, grid_map_sizeZ;

public:
    void occupied_test(Vector2d pt);
    typedef shared_ptr<PCSmapManager> Ptr;
};

#endif