#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <utils/trajectory.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include "pid.h"

using namespace Eigen;
using namespace pulsar;

class TrajectoryController {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber poly_traj_sub_;
    ros::Subscriber heartbeat_sub_;
    ros::Subscriber odom_sub_;
    ros::Timer cmd_timer_;

    bool receive_odom_{false};

    // Trajectory
    std::shared_ptr<Trajectory<5>> jerk_traj;
    bool receive_traj_{false};
    double traj_duration_;
    ros::Time start_time_;
    int traj_id_;
    ros::Time heartbeat_time_;
    Vector3d last_pos_;  // xy and yaw

    // PID Controllers for X, Y and Yaw
    PID<Vector3d>::Ptr pid_controller_;

    // Current state
    Vector3d current_pos_;  // xy and yaw
    Vector3d current_vel_;  // xy and yaw_rate
    bool print_logInfo_;

public:
    TrajectoryController() {
        heartbeat_time_ = ros::Time(0);
        ros::Duration(1.0).sleep();
        initializeROS();
        initializePID();
    }

    void initializeROS() {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        poly_traj_sub_ = nh_.subscribe("/planning/trajectory", 10, &TrajectoryController::polyTrajCallback, this);
        heartbeat_sub_ = nh_.subscribe("/planning/heartbeat", 10, &TrajectoryController::heartbeatCallback, this);

        odom_sub_ = nh_.subscribe("/odom", 1, &TrajectoryController::odomCallback, this);
        cmd_timer_ = nh_.createTimer(ros::Duration(0.01), &TrajectoryController::cmdCallback, this);
        nh_.param("traj_server/print_logInfo", print_logInfo_, true);
    }

    void initializePID() {
        pid_controller_.reset(new PID<Vector3d>());

        // // Make yaw control more conservative
        // Vector3d p_gains(1.5, 1.5, 1.2);     // Reduced yaw P gain
        // Vector3d i_gains(0.15, 0.15, 0.07);  // Reduced yaw I gain
        // Vector3d d_gains(0.8, 0.8, 0.45);     // Reduced yaw D gain

            Vector3d p_gains(2.0, 2.0, 0.8);     // Increased XY from 1.5 to 2.0
    Vector3d i_gains(0.2, 0.2, 0.05);    // Increased XY from 0.15 to 0.2
    Vector3d d_gains(1.0, 1.0, 0.6);     // Increased XY from 0.8 to 1.0

        pid_controller_->set_pid_gain(p_gains, i_gains, d_gains);
    }

    void odomCallback(const nav_msgs::OdometryPtr& msg) {
        // Update XY position
        current_pos_[0] = msg->pose.pose.position.x;
        current_pos_[1] = msg->pose.pose.position.y;

        // Extract yaw from quaternion
        Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Matrix3d R(q);
        current_pos_[2] = atan2(R.col(0)[1], R.col(0)[0]);  // Current yaw

        // Update velocities
        current_vel_[0] = msg->twist.twist.linear.x;
        current_vel_[1] = msg->twist.twist.linear.y;
        current_vel_[2] = msg->twist.twist.angular.z;  // Yaw rate

        receive_odom_ = true;
    }

    void heartbeatCallback(const std_msgs::EmptyPtr& msg) { heartbeat_time_ = ros::Time::now(); }

    void polyTrajCallback(const traj_utils::PolyTrajPtr& msg) {
        ROS_INFO("[Traj server]: received trajectory.");

        if (msg->order != 5) {
            ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
            return;
        }

        // Process trajectory pieces
        int piece_nums = msg->duration.size();
        std::vector<double> dura(piece_nums);
        std::vector<Trajectory<5>::CoefficientMat> cMats(piece_nums);

        for (int i = 0; i < piece_nums; ++i) {
            int i6 = i * 6;
            // X and Y coefficients
            cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2], msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
            cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2], msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
            // Yaw coefficients (previously was z)
            cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2], msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

            dura[i] = msg->duration[i];
        }

        jerk_traj.reset(new Trajectory<5>(dura, cMats));
        start_time_ = msg->start_time;
        traj_duration_ = jerk_traj->getTotalDuration();
        traj_id_ = msg->traj_id;
        receive_traj_ = true;

        pid_controller_->clear_integration();

        if (print_logInfo_) {
            std::cout << "\nTrajectory Information:" << std::endl;
            std::cout << std::fixed << std::setprecision(4);

            // Print start position
            Eigen::Vector3d start_pos = jerk_traj->getPos(0);
            std::cout << "Start Position: "
                      << "X: " << start_pos.x() << ", Y: " << start_pos.y() << ", Z: " << start_pos.z() << std::endl;

            // Print waypoints
            std::cout << "\nTrajectory Waypoints:" << std::endl;

            double total_time = 0;
            for (int i = 0; i < piece_nums; ++i) {
                double piece_duration = dura[i];
                int num_samples = std::max(10, int(piece_duration / 0.1));  // At least 10 samples per piece, or one every 0.1 seconds

                for (int j = 0; j <= num_samples; ++j) {
                    double t = j * piece_duration / num_samples;
                    Eigen::Vector3d pos = jerk_traj->getPos(total_time + t);
                    std::cout << "Time: " << total_time + t << ", X: " << pos.x() << ", Y: " << pos.y() << ", Z: " << pos.z() << std::endl;
                }

                total_time += piece_duration;
            }

            // Print final position
            Eigen::Vector3d final_pos = jerk_traj->getPos(total_time);
            std::cout << "\nFinal Position: "
                      << "X: " << final_pos.x() << ", Y: " << final_pos.y() << ", Z: " << final_pos.z() << std::endl;

            std::cout << "Total Duration: " << total_time << " seconds" << std::endl;
        }
    }

    void cmdCallback(const ros::TimerEvent& e) {
        // if (!receive_traj_ || (heartbeat_time_.toSec() <= 1e-5)) {
        //     ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
        //     return;
        // }

        double speed_scale = 1.0;  // Same value as in publishCommand
        ros::Time time_now = ros::Time::now();
        // if ((time_now - heartbeat_time_).toSec() > 0.5) {
        //     ROS_ERROR("[traj_server] Lost heartbeat from the planner!");
        //     receive_traj_ = false;
        //     publishCommand(Vector3d::Zero());  // Stop the vehicle
        //     return;
        // }

        if (!receive_odom_) {
            ROS_ERROR("[traj_server] no odom received!");
            return;
        }

        // Scale the time to effectively slow down the trajectory
        double t_cur = (time_now - start_time_).toSec() * speed_scale;

        if (t_cur < traj_duration_ && t_cur >= 0.0) {
            // Get desired state from trajectory
            Vector3d desired_pos = jerk_traj->getPos(t_cur);
            Vector3d desired_vel = jerk_traj->getVel(t_cur);

            // Normalize both current and desired yaw to [-pi, pi]
            desired_pos[2] = normalizeAngle(desired_pos[2]);
            current_pos_[2] = normalizeAngle(current_pos_[2]);

            // Calculate yaw error considering shortest path
            double yaw_error = normalizeAngle(desired_pos[2] - current_pos_[2]);

            // Create modified desired position with correct yaw error
            Vector3d modified_desired_pos = desired_pos;
            modified_desired_pos[2] = current_pos_[2] + yaw_error;

            // Calculate control using PID with modified desired position
            double dt = 0.01;
            Vector3d control_output = pid_controller_->pid_cal(modified_desired_pos, current_pos_, dt);

            // Only apply yaw velocity if error is significant
            if (std::abs(yaw_error) < 0.05) {  // About 3 degrees
                control_output[2] = 0.0;
                desired_vel[2] = 0.0;
            }

            // Add feedforward from trajectory
            control_output += desired_vel;

            // Publish command
            publishCommand(control_output);
        } else if (t_cur >= traj_duration_) {
            publishCommand(Vector3d::Zero());
        }
    }

    void publishCommand(const Vector3d& cmd) {
        geometry_msgs::Twist cmd_vel;

        // Add a speed scaling factor for desired trajectory
        double speed_scale = 1.0;  // Your slow speed setting

        // Get the desired trajectory velocities from cmd
        Vector3d desired_vel = cmd;

        // Scale the desired trajectory velocity and timing
        // This means we're actually following a slower trajectory
        desired_vel *= speed_scale;

        // Transform XY velocity from world to robot frame
        double cos_yaw = cos(current_pos_[2]);
        double sin_yaw = sin(current_pos_[2]);

        // Apply transformation without additional scaling
        cmd_vel.linear.x = cos_yaw * desired_vel[0] + sin_yaw * desired_vel[1];
        cmd_vel.linear.y = -sin_yaw * desired_vel[0] + cos_yaw * desired_vel[1];

        // Only apply yaw rate if it's significant
        cmd_vel.angular.z = (std::abs(desired_vel[2]) > 0.05) ? desired_vel[2] : 0.0;

        cmd_vel_pub_.publish(cmd_vel);
    }

private:
    double normalizeAngle(double angle) {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_server");
    TrajectoryController controller;
    ROS_INFO("[Traj server]: ready.");
    ros::spin();
    return 0;
}