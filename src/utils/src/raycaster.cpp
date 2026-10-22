
#include "utils/raycaster.h"
#include <iostream>

RayCaster::RayCaster(const double &resolution) {
    if (resolution < 0) {
        throw std::runtime_error(" -- [Raycaster]: resolution must be positive!");
    } else {
        resolution_ = resolution;
    }
}

void RayCaster::setResolution(const double &resolution) {
    resolution_ = resolution;
}

// void RayCaster::posToIndex(const double d, int &id) const {
// #if RAYCAST_MODE == ORIGIN_AT_CORNER
//     id = std::floor((d / resolution_));
// #else RAYCAST_MODE == ORIGIN_AT_CENTER
//     id = static_cast<int>((d / resolution_ + signum(d) * 0.5));
// #endif
// }

// void RayCaster::indexToPos(const int &id, double &d) const {
// #if RAYCAST_MODE == ORIGIN_AT_CORNER
//     d = (id + 0.5) * resolution_;
// #else RAYCAST_MODE == ORIGIN_AT_CENTER
//     d = id * resolution_ - 0.1;
// #endif
// }

void RayCaster::indexToPos(const int &id, double &d) const {
#if RAYCAST_MODE == ORIGIN_AT_CORNER
    d = (id + 0.5) * resolution_;
#else RAYCAST_MODE == ORIGIN_AT_CENTER
    d = id * resolution_ + map_origin_;  // Add map origin instead of subtracting 0.1
#endif
}

void RayCaster::posToIndex(const double d, int &id) const {
#if RAYCAST_MODE == ORIGIN_AT_CORNER
    id = std::floor((d / resolution_));
#else RAYCAST_MODE == ORIGIN_AT_CENTER
    id = static_cast<int>(((d - map_origin_) / resolution_ + signum(d) * 0.5));  // Subtract origin before converting
#endif
}

bool RayCaster::setInput(const Eigen::Vector3d &start, const Eigen::Vector3d &end) {
    // std::cout << "\n=== RayCaster SetInput ===\n";
    // std::cout << "Start point: (" << start.x() << ", " << start.y() << ", " << start.z() << ")\n";
    // std::cout << "End point: (" << end.x() << ", " << end.y() << ", " << end.z() << ")\n";
    // std::cout << "Resolution: " << resolution_ << "\n";

    if (resolution_ < 0) {
        throw std::runtime_error(" -- [RayCaster] Resolution is not set!");
    }
    start_x_d_ = start.x();
    start_y_d_ = start.y();
    start_z_d_ = start.z();

    end_x_d_ = end.x();
    end_y_d_ = end.y();
    end_z_d_ = end.z();

    // Calculate expand dir and index
    posToIndex(start_x_d_, start_x_i_);
    posToIndex(start_y_d_, start_y_i_);
    posToIndex(start_z_d_, start_z_i_);

    posToIndex(end_x_d_, end_x_i_);
    posToIndex(end_y_d_, end_y_i_);
    posToIndex(end_z_d_, end_z_i_);

    // std::cout << "Start indices: (" << start_x_i_ << ", " << start_y_i_ << ", " << start_z_i_ << ")\n";
    // std::cout << "End indices: (" << end_x_i_ << ", " << end_y_i_ << ", " << end_z_i_ << ")\n";

    int delta_X = end_x_i_ - start_x_i_;
    int delta_Y = end_y_i_ - start_y_i_;
    int delta_Z = end_z_i_ - start_z_i_;

    // std::cout << "Delta indices: (" << delta_X << ", " << delta_Y << ", " << delta_Z << ")\n";

    cur_ray_pt_id_x_ = start_x_i_;
    cur_ray_pt_id_y_ = start_y_i_;
    cur_ray_pt_id_z_ = start_z_i_;

    expand_dir_x_ = static_cast<int>(signum(static_cast<int>(delta_X)));
    expand_dir_y_ = static_cast<int>(signum(static_cast<int>(delta_Y)));
    expand_dir_z_ = static_cast<int>(signum(static_cast<int>(delta_Z)));

    // std::cout << "Expand directions: (" << expand_dir_x_ << ", " << expand_dir_y_ << ", " << expand_dir_z_ << ")\n";

    if (expand_dir_x_ == 0 && expand_dir_y_ == 0 && expand_dir_z_ == 0) {
        std::cout << "No direction to expand!\n";
        return false;
    }

    double dis_x_over_t = std::abs(end_x_d_ - start_x_d_);
    double dis_y_over_t = std::abs(end_y_d_ - start_y_d_);
    double dis_z_over_t = std::abs(end_z_d_ - start_z_d_);

    // std::cout << "Raw distances: (" << dis_x_over_t << ", " << dis_y_over_t << ", " << dis_z_over_t << ")\n";

    double t_max = sqrt(dis_x_over_t * dis_x_over_t + dis_y_over_t * dis_y_over_t + dis_z_over_t * dis_z_over_t);
    // std::cout << "Total distance (t_max): " << t_max << "\n";

    dis_x_over_t /= t_max;
    dis_y_over_t /= t_max;
    dis_z_over_t /= t_max;

    // std::cout << "Normalized distances: (" << dis_x_over_t << ", " << dis_y_over_t << ", " << dis_z_over_t << ")\n";

    t_when_step_x_ = expand_dir_x_ == 0 ? std::numeric_limits<double>::max() : std::abs(resolution_ / dis_x_over_t);
    t_when_step_y_ = expand_dir_y_ == 0 ? std::numeric_limits<double>::max() : std::abs(resolution_ / dis_y_over_t);
    t_when_step_z_ = expand_dir_z_ == 0 ? std::numeric_limits<double>::max() : std::abs(resolution_ / dis_z_over_t);

    // std::cout << "Step times: (" << t_when_step_x_ << ", " << t_when_step_y_ << ", " << t_when_step_z_ << ")\n";

    double start_grid_center_x, start_grid_center_y, start_grid_center_z;
    indexToPos(start_x_i_, start_grid_center_x);
    indexToPos(start_y_i_, start_grid_center_y);
    indexToPos(start_z_i_, start_grid_center_z);

    // std::cout << "Start grid centers: (" << start_grid_center_x << ", " << start_grid_center_y << ", " << start_grid_center_z << ")\n";

    double next_bound_x = start_grid_center_x + expand_dir_x_ * resolution_ * 0.5;
    double next_bound_y = start_grid_center_y + expand_dir_y_ * resolution_ * 0.5;
    double next_bound_z = start_grid_center_z + expand_dir_z_ * resolution_ * 0.5;

    // std::cout << "Next bounds: (" << next_bound_x << ", " << next_bound_y << ", " << next_bound_z << ")\n";

    t_to_bound_x_ = expand_dir_x_ == 0 ? std::numeric_limits<double>::max() : std::fabs(next_bound_x - start_x_d_) / dis_x_over_t;
    t_to_bound_y_ = expand_dir_y_ == 0 ? std::numeric_limits<double>::max() : std::fabs(next_bound_y - start_y_d_) / dis_y_over_t;
    t_to_bound_z_ = expand_dir_z_ == 0 ? std::numeric_limits<double>::max() : std::fabs(next_bound_z - start_z_d_) / dis_z_over_t;

    // std::cout << "Initial times to bounds: (" << t_to_bound_x_ << ", " << t_to_bound_y_ << ", " << t_to_bound_z_ << ")\n";

    step_num_ = 0;
    first_point = true;
    return true;
}

// bool RayCaster::setInput(const Eigen::Vector3d &start, const Eigen::Vector3d &end) {
//     if (resolution_ < 0) {
//         throw std::runtime_error(" -- [RayCaster] Resolution is not set!");
//     }
//     start_x_d_ = start.x();
//     start_y_d_ = start.y();
//     start_z_d_ = start.z();

//     end_x_d_ = end.x();
//     end_y_d_ = end.y();
//     end_z_d_ = end.z();

//     // Calculate expand dir and index
//     posToIndex(start_x_d_, start_x_i_);
//     posToIndex(start_y_d_, start_y_i_);
//     posToIndex(start_z_d_, start_z_i_);

//     posToIndex(end_x_d_, end_x_i_);
//     posToIndex(end_y_d_, end_y_i_);
//     posToIndex(end_z_d_, end_z_i_);

//     int delta_X = end_x_i_ - start_x_i_;
//     int delta_Y = end_y_i_ - start_y_i_;
//     int delta_Z = end_z_i_ - start_z_i_;

//     cur_ray_pt_id_x_ = start_x_i_;
//     cur_ray_pt_id_y_ = start_y_i_;
//     cur_ray_pt_id_z_ = start_z_i_;

//     expand_dir_x_ = static_cast<int>(signum(static_cast<int>(delta_X)));
//     expand_dir_y_ = static_cast<int>(signum(static_cast<int>(delta_Y)));
//     expand_dir_z_ = static_cast<int>(signum(static_cast<int>(delta_Z)));

//     if (expand_dir_x_ == 0 && expand_dir_y_ == 0 && expand_dir_z_ == 0) {
//         return false;
//     }

//     double dis_x_over_t, dis_y_over_t, dis_z_over_t, t_max;
//     dis_x_over_t = std::abs(end_x_d_ - start_x_d_);
//     dis_y_over_t = std::abs(end_y_d_ - start_y_d_);
//     dis_z_over_t = std::abs(end_z_d_ - start_z_d_);

//     t_max = sqrt(dis_x_over_t * dis_x_over_t + dis_y_over_t * dis_y_over_t + dis_z_over_t * dis_z_over_t);

//     dis_x_over_t /= t_max;
//     dis_y_over_t /= t_max;
//     dis_z_over_t /= t_max;

//     // construct the look-up table
//     // Which means, how long is t change when extend one resolution along A axis
//     t_when_step_x_ = expand_dir_x_ == 0 ? std::numeric_limits<double>::max() : std::abs(resolution_ / dis_x_over_t);
//     t_when_step_y_ = expand_dir_y_ == 0 ? std::numeric_limits<double>::max() : std::abs(resolution_ / dis_y_over_t);
//     t_when_step_z_ = expand_dir_z_ == 0 ? std::numeric_limits<double>::max() : std::abs(resolution_ / dis_z_over_t);

//     // Calculate nex bound
//     double start_grid_center_x, start_grid_center_y, start_grid_center_z;
//     indexToPos(start_x_i_, start_grid_center_x);
//     indexToPos(start_y_i_, start_grid_center_y);
//     indexToPos(start_z_i_, start_grid_center_z);
//     double next_bound_x, next_bound_y, next_bound_z;
//     next_bound_x = start_grid_center_x + expand_dir_x_ * resolution_ * 0.5;
//     next_bound_y = start_grid_center_y + expand_dir_y_ * resolution_ * 0.5;
//     next_bound_z = start_grid_center_z + expand_dir_z_ * resolution_ * 0.5;

//     t_to_bound_x_ = expand_dir_x_ == 0 ? std::numeric_limits<double>::max() : std::fabs(next_bound_x - start_x_d_) / dis_x_over_t;
//     t_to_bound_y_ = expand_dir_y_ == 0 ? std::numeric_limits<double>::max() : std::fabs(next_bound_y - start_y_d_) / dis_y_over_t;
//     t_to_bound_z_ = expand_dir_z_ == 0 ? std::numeric_limits<double>::max() : std::fabs(next_bound_z - start_z_d_) / dis_z_over_t;

//     //                std::cout << "t_to_bound_x_: " << t_to_bound_x_ << " t_to_bound_y_: " << t_to_bound_y_
//     //                          << " t_to_bound_z_: " << t_to_bound_z_ << std::endl;

//     step_num_ = 0;
//     first_point = true;
//     return true;
// }

// bool RayCaster::step(Eigen::Vector3d &ray_pt) {
//     step_num_++;
//     //                std::cout << "Iter " << step_num_ << "========================================" << std::endl;
//     //                std::cout << "start: " << start_x_d_ << ", " << start_y_d_ << ", " << start_z_d_ << std::endl;
//     //                std::cout << "end: " << end_x_d_ << ", " << end_y_d_ << ", " << end_z_d_ << std::endl;
//     //                std::cout << "end_id: " << end_x_i_ << ", " << end_y_i_ << ", " << end_z_i_ << std::endl;
//     //                std::cout << "cur_id: " << cur_ray_pt_id_x_ << ", " << cur_ray_pt_id_y_ << ", " << cur_ray_pt_id_z_
//     //                          << std::endl;
//     //                std::cout << "t_to_bound
//     //                : " << t_to_bound_x_ << ", " << t_to_bound_y_ << ", " << t_to_bound_z_
//     //                          << std::endl;
//     // shape the output
//     indexToPos(cur_ray_pt_id_x_, ray_pt.x());
//     indexToPos(cur_ray_pt_id_y_, ray_pt.y());
//     indexToPos(cur_ray_pt_id_z_, ray_pt.z());
//     if (cur_ray_pt_id_x_ == end_x_i_ && cur_ray_pt_id_y_ == end_y_i_ && cur_ray_pt_id_z_ == end_z_i_) {
//         return false;
//     }

//     // compute firest bound point
//     if (t_to_bound_x_ < t_to_bound_y_) {
//         if (t_to_bound_x_ < t_to_bound_z_) {
//             //                        std::cout << "Expand X" << std::endl;
//             cur_ray_pt_id_x_ += expand_dir_x_;
//             t_to_bound_x_ += t_when_step_x_;
//         } else {
//             //                        std::cout << "Expand Z" << std::endl;
//             cur_ray_pt_id_z_ += expand_dir_z_;
//             t_to_bound_z_ += t_when_step_z_;
//         }
//     } else {
//         if (t_to_bound_y_ < t_to_bound_z_) {
//             //                        std::cout << "Expand Y" << std::endl;
//             cur_ray_pt_id_y_ += expand_dir_y_;
//             t_to_bound_y_ += t_when_step_y_;
//         } else {
//             //                        std::cout << "Expand Z" << std::endl;
//             cur_ray_pt_id_z_ += expand_dir_z_;
//             t_to_bound_z_ += t_when_step_z_;
//         }
//     }
//     return true;
// }

bool RayCaster::step(Eigen::Vector3d &ray_pt) {
    step_num_++;

    // Get current position
    indexToPos(cur_ray_pt_id_x_, ray_pt.x());
    indexToPos(cur_ray_pt_id_y_, ray_pt.y());
    indexToPos(cur_ray_pt_id_z_, ray_pt.z());

    // std::cout << "\n=== Step " << step_num_ << " ===\n";
    // std::cout << "Current position: (" << ray_pt.x() << ", " << ray_pt.y() << ", " << ray_pt.z() << ")\n";
    // std::cout << "Current indices: (" << cur_ray_pt_id_x_ << ", " << cur_ray_pt_id_y_ << ", " << cur_ray_pt_id_z_ << ")\n";
    // std::cout << "Target indices: (" << end_x_i_ << ", " << end_y_i_ << ", " << end_z_i_ << ")\n";

    // Check if we've already reached or passed target in any dimension
    if (expand_dir_x_ > 0 && cur_ray_pt_id_x_ >= end_x_i_)
        return false;
    if (expand_dir_x_ < 0 && cur_ray_pt_id_x_ <= end_x_i_)
        return false;
    if (expand_dir_y_ > 0 && cur_ray_pt_id_y_ >= end_y_i_)
        return false;
    if (expand_dir_y_ < 0 && cur_ray_pt_id_y_ <= end_y_i_)
        return false;
    if (expand_dir_z_ > 0 && cur_ray_pt_id_z_ >= end_z_i_)
        return false;
    if (expand_dir_z_ < 0 && cur_ray_pt_id_z_ <= end_z_i_)
        return false;

    // std::cout << "Times to bounds: (" << t_to_bound_x_ << ", " << t_to_bound_y_ << ", " << t_to_bound_z_ << ")\n";

    // Choose expansion direction but prevent overshooting
    if (t_to_bound_x_ < t_to_bound_y_) {
        if (t_to_bound_x_ < t_to_bound_z_) {
            // Check if next X step would overshoot
            int next_x = cur_ray_pt_id_x_ + expand_dir_x_;
            if ((expand_dir_x_ > 0 && next_x > end_x_i_) || (expand_dir_x_ < 0 && next_x < end_x_i_)) {
                return false;
            }
            // std::cout << "Expanding X\n";
            cur_ray_pt_id_x_ = next_x;
            t_to_bound_x_ += t_when_step_x_;
        } else {
            // Check if next Z step would overshoot
            int next_z = cur_ray_pt_id_z_ + expand_dir_z_;
            if ((expand_dir_z_ > 0 && next_z > end_z_i_) || (expand_dir_z_ < 0 && next_z < end_z_i_)) {
                return false;
            }
            // std::cout << "Expanding Z\n";
            cur_ray_pt_id_z_ = next_z;
            t_to_bound_z_ += t_when_step_z_;
        }
    } else {
        if (t_to_bound_y_ < t_to_bound_z_) {
            // Check if next Y step would overshoot
            int next_y = cur_ray_pt_id_y_ + expand_dir_y_;
            if ((expand_dir_y_ > 0 && next_y > end_y_i_) || (expand_dir_y_ < 0 && next_y < end_y_i_)) {
                return false;
            }
            // std::cout << "Expanding Y\n";
            cur_ray_pt_id_y_ = next_y;
            t_to_bound_y_ += t_when_step_y_;
        } else {
            // Check if next Z step would overshoot
            int next_z = cur_ray_pt_id_z_ + expand_dir_z_;
            if ((expand_dir_z_ > 0 && next_z > end_z_i_) || (expand_dir_z_ < 0 && next_z < end_z_i_)) {
                return false;
            }
            // std::cout << "Expanding Z\n";
            cur_ray_pt_id_z_ = next_z;
            t_to_bound_z_ += t_when_step_z_;
        }
    }

    return true;
}
