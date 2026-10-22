#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Eigen/Eigen>

#include <ros/ros.h>

#include <string>
#include <vector>
using namespace Eigen;
using namespace std;

struct Config {
public:
    int threads_num{12};
    vector<double> poly_params;


    double testRate;
    double ts{-1.0};

    double torlerance{0.005};
    int degree{4};
    std::string inputdata;  // swept volume

    double t_min{0};
    double t_max{1};
    double momentum{0};  // Nesterov Acceleration Momentum
    double occupancy_resolution{1.0};
    bool debug_output{true};
    int sta_threshold{1};

    double weight_v;
    double weight_a;
    double weight_p;
    double weight_pr;
    double weight_ar;
    double weight_omg;
    double weight_theta;
    double safety_hor;
    double vmax;
    double omgmax;
    double thetamax;
    double rho;
    double rho_mid_end;
    double inittime{10.0};
    // Use the swept-volume/parallel obstacle-point cost unless a configuration
    // explicitly opts into the legacy discretized benchmark implementation.
    // lbfgs params
    int mem_size{256};
    int past{3};
    double min_step{1.0e-32};
    double g_epsilon{0.0};

    // SE2 LMBM convergence settings, separate from the R2 LBFGS settings.
    int se2_past{5};
    double se2_delta_past{1.0e-4};
    // SE2 early termination, checked once per LMBM iteration (<= 0 disables).
    int se2_max_iter{150};            // iteration cap per solve
    double se2_max_time{3.0};         // wall-clock budget per solve, seconds
    int se2_stuck_window{20};         // stop when still colliding by more than
    double se2_stuck_violation{0.01}; // this many metres (summed safety-band
    double se2_stuck_rel_tol{1e-2};   // intrusion) and it fell < tol over the window

    // flatness
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;

    double smoothingEps;
    int integralIntervs;
    double relCostTolMidEnd;
    double eps{0.2};  // swept volume mesh surface precision

    // Initial (start) pose, set from the launch file as ~planning/start_x,y
    double start_x{0.0};
    double start_y{0.0};

    int kernel_size;  // 必须是奇数，对应小方块的个数
    int kernel_yaw_num;
    double front_end_safeh;
    bool enableearlyExit{false};
    int debugpause{20};

    // topo search
    bool topo_prm_test_mode;

    // Margins of the topo sampling rectangle, world axis aligned and centred
    // on the midpoint between start and goal. The box always spans the
    // start/goal bounding box; these say how far beyond it to sample, so it
    // can never exclude the endpoints however far apart they are.
    double topo_prm_sample_margin_x = 5.0;
    double topo_prm_sample_margin_y = 5.0;
    // Absolute full width/height of the topo sampling rectangle, in metres.
    // A positive value replaces the margin computation on that axis. It is
    // still enlarged when needed so the box contains start and goal. 0 keeps
    // the margin behaviour.
    double topo_prm_sample_width_x = 0.0;
    double topo_prm_sample_width_y = 0.0;

    double topo_prm_max_sample_time = 0.5;
    int topo_prm_max_sample_num = 5000;

    int topo_prm_max_raw_path = 10;
    int topo_prm_max_raw_path2 = 4;
    int topo_prm_reserve_num = 6;
    double topo_prm_ratio_to_short = 6.0;
    int topo_prm_iterator_num = 1;
    // Extra distance, in metres, that a pushed path point is moved beyond the
    // first collision-free candidate when the surroundings allow it. 0 keeps
    // the minimum push, which hugs the obstacle.
    double push_extra_dist = 0.0;

    // occupancy grid map
    double robot_inner_radius;
    int mapSize_x, mapSize_y, mapSize_z;
    double safety_hor_fastCheck;

    //  planner strategy
    int segmentTolerance, min_free_segmentPoints;

    int inflation_size;

    double yaw_angle = 0.0;

#define LOAD_PARAM(x) nh_priv.getParam(#x, x)
    inline void loadParameters(const ros::NodeHandle &nh_priv) {
        LOAD_PARAM(weight_v);
        LOAD_PARAM(weight_a);
        LOAD_PARAM(weight_p);
        LOAD_PARAM(weight_pr);
        LOAD_PARAM(weight_ar);
        LOAD_PARAM(weight_omg);
        LOAD_PARAM(weight_theta);
        LOAD_PARAM(safety_hor);
        LOAD_PARAM(vmax);
        LOAD_PARAM(omgmax);
        LOAD_PARAM(thetamax);
        LOAD_PARAM(rho);
        LOAD_PARAM(rho_mid_end);
        LOAD_PARAM(inittime);
        LOAD_PARAM(mem_size);
        LOAD_PARAM(past);
        LOAD_PARAM(min_step);
        LOAD_PARAM(g_epsilon);
        LOAD_PARAM(se2_past);
        LOAD_PARAM(se2_delta_past);
        LOAD_PARAM(se2_max_iter);
        LOAD_PARAM(se2_max_time);
        LOAD_PARAM(se2_stuck_window);
        LOAD_PARAM(se2_stuck_violation);
        LOAD_PARAM(se2_stuck_rel_tol);
        LOAD_PARAM(relCostTolMidEnd);
        LOAD_PARAM(enableearlyExit);
        LOAD_PARAM(debugpause);

        LOAD_PARAM(inputdata);

        LOAD_PARAM(testRate);

        LOAD_PARAM(ts);

        LOAD_PARAM(eps);
        LOAD_PARAM(t_min);
        LOAD_PARAM(t_max);
        LOAD_PARAM(momentum);
        LOAD_PARAM(torlerance);

        // PCSmap_manager

        LOAD_PARAM(sta_threshold);
        LOAD_PARAM(debug_output);
        LOAD_PARAM(occupancy_resolution);
        LOAD_PARAM(kernel_size);
        LOAD_PARAM(inflation_size);

        nh_priv.param<std::vector<double>>("poly_params", poly_params, std::vector<double>());

        LOAD_PARAM(vehicleMass);
        LOAD_PARAM(gravAcc);
        LOAD_PARAM(horizDrag);
        LOAD_PARAM(vertDrag);
        LOAD_PARAM(parasDrag);
        LOAD_PARAM(speedEps);
        LOAD_PARAM(threads_num);
        LOAD_PARAM(smoothingEps);
        LOAD_PARAM(integralIntervs);

        LOAD_PARAM(front_end_safeh);
        LOAD_PARAM(kernel_yaw_num);

        // topo search
        LOAD_PARAM(topo_prm_test_mode);
        LOAD_PARAM(topo_prm_sample_margin_x);
        LOAD_PARAM(topo_prm_sample_margin_y);
        LOAD_PARAM(topo_prm_sample_width_x);
        LOAD_PARAM(topo_prm_sample_width_y);

        LOAD_PARAM(topo_prm_max_sample_time);
        LOAD_PARAM(topo_prm_max_sample_num);

        LOAD_PARAM(topo_prm_max_raw_path);
        LOAD_PARAM(topo_prm_max_raw_path2);
        LOAD_PARAM(topo_prm_reserve_num);
        LOAD_PARAM(topo_prm_ratio_to_short);
        LOAD_PARAM(topo_prm_iterator_num);
        LOAD_PARAM(push_extra_dist);

        // occupancy grid map
        LOAD_PARAM(robot_inner_radius);
        LOAD_PARAM(mapSize_x);
        LOAD_PARAM(mapSize_y);
        LOAD_PARAM(mapSize_z);

        LOAD_PARAM(safety_hor_fastCheck);

        //  planner strategy
        LOAD_PARAM(segmentTolerance);
        LOAD_PARAM(min_free_segmentPoints);

        nh_priv.param("planning/start_x", start_x, 0.0);
        nh_priv.param("planning/start_y", start_y, 0.0);

        LOAD_PARAM(yaw_angle);


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        ROS_WARN_STREAM("inputdata:" << inputdata);

        ROS_WARN_STREAM("degree:" << degree);
        ROS_WARN_STREAM("threads_num:" << threads_num);
        ROS_WARN_STREAM("ts:" << ts);

        ROS_WARN_STREAM("eps:" << eps);
        ROS_WARN_STREAM("swept volume t_max:" << t_max);


        ROS_WARN_STREAM("momentum:" << momentum);
        ROS_WARN_STREAM("torlerance:" << torlerance);


        ROS_WARN_STREAM("weight_v:" << weight_v);
        ROS_WARN_STREAM("weight_a:" << weight_a);
        ROS_WARN_STREAM("weight_p:" << weight_p);
        ROS_WARN_STREAM("weight_omg:" << weight_omg);
        ROS_WARN_STREAM("weight_theta:" << weight_theta);
        ROS_WARN_STREAM("safety_hor:" << safety_hor);
        ROS_WARN_STREAM("vmax:" << vmax);
        ROS_WARN_STREAM("omgmax:" << omgmax);
        ROS_WARN_STREAM("thetamax:" << thetamax);
        ROS_WARN_STREAM("rho:" << rho);
        ROS_WARN_STREAM("mem_size:" << mem_size);
        ROS_WARN_STREAM("past:" << past);
        ROS_WARN_STREAM("min_step:" << min_step);
        ROS_WARN_STREAM("g_epsilon:" << g_epsilon);
        ROS_WARN_STREAM("enableearlyExit:" << enableearlyExit);
        ROS_WARN_STREAM("debugpause:" << debugpause);
        ROS_WARN_STREAM("testRate:" << testRate);

        ROS_WARN_STREAM("robot_inner_radius:" << robot_inner_radius);
        ROS_WARN_STREAM("mapSize_x:" << mapSize_x);
        ROS_WARN_STREAM("mapSize_y:" << mapSize_y);
        ROS_WARN_STREAM("mapSize_z:" << mapSize_z);
        return;
    }
};

#endif
