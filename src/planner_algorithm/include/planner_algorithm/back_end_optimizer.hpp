#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <string>
#include <thread>
#include <map_manager/PCSmap_manager.h>
#include <omp.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <swept_volume/sw_manager.hpp>
#include <utils/Visualization.hpp>
#include <utils/config.hpp>
#include <utils/debug_publisher.hpp>
#include <utils/flatness.hpp>
#include <utils/lbfgs.hpp>
#include <utils/lmbm.h>
#include <utils/minco.hpp>
#include <visualization_msgs/Marker.h>
#define TRAJ_ORDER 5
#define GNOW ros::Time::now()
#define DEBUGOUT(x) std::cout << x << std::endl

class TrajOptimizer {
public:
  Eigen::VectorXd opt_x;

  double total_opt_time = 0.0;
  double total_sdf_time = 0.0;
  double total_AABB_time = 0.0;

  // ros
  ros::NodeHandle nh;
  ros::Publisher aabb_points_pub;
  visualization_msgs::MarkerArray aabb_group_markers;
  ros::Publisher debug_step_pub;
  ros::Publisher debug_wp_pub;
  ros::Publisher debug_vec_pub;
  ros::Publisher debug_wplists_pub;
  ros::Publisher test_map_pub;
  ros::Publisher debugesdfpub;
  minco::MINCO_S3NU minco;
  flatness::FlatnessMap flatmap;
  SweptVolumeManager::Ptr sv_manager;
  PCSmapManager::Ptr pcsmap_manager;
  Trajectory<TRAJ_ORDER> step_traj;
  vis::Visualization::Ptr vis;

  double rho;
  Eigen::Matrix3d initState;
  Eigen::Matrix3d finalState;
  double *x_variable;

  int pieceN;
  int spatialDim;
  int temporalDim;

  int integralRes;

  Eigen::Matrix3Xd points;
  Eigen::VectorXd times;
  Eigen::Matrix3Xd gradByPoints;
  Eigen::VectorXd gradByTimes;
  Eigen::MatrixX3d partialGradByCoeffs;
  Eigen::VectorXd partialGradByTimes;

  Config conf;

  double vmax;
  double omgmax;
  double thetamax;
  double weight_v;
  double weight_a;
  double weight_p;
  double weight_omg;
  double weight_theta;
  double smooth_fac;
  double safety_hor;

  double bdx; //=config.kernel_size*config.occupancy_resolution;  bounding box x
  double bdy; //=config.kernel_size*config.occupancy_resolution;  bounding box y
  double bdz; //=config.kernel_size*config.occupancy_resolution;  bounding box z

  int threads_num{1}; // 1.5*kernel size of cpu
  int parallel_points_num;
  std::vector<Eigen::Vector3d> parallel_points;
  std::vector<double> lastTstar;

  double cost_pos;
  double cost_other;
  double cost_total;
  double ori_cost_pos;

  // debug purspose
  double energy_cost_global = 0.0;
  double pos_cost_global = 0.0;
  double time_cost_global = 0.0;

  bool exit = false;

  int iter = 0;

  // ---- SE2 early termination, checked in earlyExitLMBM ----
  // LMBM's last evaluation can be a rejected line-search trial, so evaluations
  // are kept with the point they were made at and the current iterate's costs
  // are looked up by its coordinates.
  struct EvalRecord {
    std::vector<double> x;
    double total;
    double pos; // collision term; 0 once every obstacle point clears safety_hor
  };
  std::deque<EvalRecord> recent_evals;
  EvalRecord iterate_eval;
  bool have_iterate_eval = false;
  std::vector<double> iterate_violations; // safety-band intrusion, per iteration
  std::chrono::steady_clock::time_point se2_start_time;
  std::string se2_stop_reason;

  // lmbm.so reads the early-exit flag as a Fortran REAL and stops when it is
  // > 0, but its C wrapper passes our int through bit for bit. Return the bit
  // pattern of 1.0f: a plain 1 is a denormal that flush-to-zero code reads as 0.
  static constexpr int kLmbmStop = 0x3f800000;

  void renderAABBpoints(const std::string &group_namespace);
  void drawStepTraj();
  void drawFinalTraj();

  inline Eigen::Matrix3d getQuatTransDW(const Eigen::Vector4d &quat) {
    Eigen::Matrix3d ret;
    double x = quat(1);
    double y = quat(2);
    double z = quat(3);
    ret << 0, 2 * z, -2 * y, -2 * z, 0, 2 * x, 2 * y, -2 * x, 0;
    return ret;
  }
  inline Eigen::Matrix3d getQuatTransDX(const Eigen::Vector4d &quat) {
    Eigen::Matrix3d ret;
    double w = quat(0);
    double x = quat(1);
    double y = quat(2);
    double z = quat(3);
    ret << 0, 2 * y, 2 * z, 2 * y, -4 * x, 2 * w, 2 * z, -2 * w, -4 * x;
    return ret;
  }
  inline Eigen::Matrix3d getQuatTransDY(const Eigen::Vector4d &quat) {
    Eigen::Matrix3d ret;
    double w = quat(0);
    double x = quat(1);
    double y = quat(2);
    double z = quat(3);
    ret << -4 * y, 2 * x, -2 * w, 2 * x, 0, 2 * z, 2 * w, 2 * z, -4 * y;
    return ret;
  }
  inline Eigen::Matrix3d getQuatTransDZ(const Eigen::Vector4d &quat) {
    Eigen::Matrix3d ret;
    double w = quat(0);
    double x = quat(1);
    double y = quat(2);
    double z = quat(3);
    ret << -4 * z, 2 * w, 2 * x, -2 * w, -4 * z, 2 * y, 2 * x, 2 * y, 0;
    return ret;
  }

  static inline void forwardP(const Eigen::VectorXd &xi, Eigen::Matrix3Xd &P) {
    const int sizeP = xi.size() / 3;
    P.resize(3, sizeP);
    for (int i = 0; i < sizeP; i++) {
      P.col(i) = xi.segment(3 * i, 3);
    }
    return;
  }
  static inline void forwardP(const double *xi, Eigen::Matrix3Xd &P,
                              int Space_dim) {
    P.resize(3, Space_dim);
    for (int i = 0; i < Space_dim; i++) {
      Eigen::Map<const Eigen::VectorXd> vec(xi + 3 * i, 3);
      P.col(i) = vec;
    }
    return;
  }
  template <typename EIGENVEC>
  static inline void backwardP(const Eigen::Matrix3Xd &P, EIGENVEC &xi) {
    const int sizeP = P.cols();
    for (int i = 0; i < sizeP; ++i) {
      xi.segment(3 * i, 3) = P.col(i);
    }
    return;
  }

  // tao--->T
  static inline void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T) {
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    for (int i = 0; i < sizeTau; i++) {
      T(i) = tau(i) > 0.0 ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                          : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
    return;
  }
  // tao--->T
  static inline void forwardT(const double *tau, Eigen::VectorXd &T,
                              const int Time_dim) {
    T.resize(Time_dim);
    double temp;
    for (int i = 0; i < Time_dim; i++) {
      temp = *(tau + i);
      T(i) = temp > 0.0 ? ((0.5 * temp + 1.0) * temp + 1.0)
                        : 1.0 / ((0.5 * temp - 1.0) * (temp) + 1.0);
    }
    return;
  }
  // T--->tao
  template <typename EIGENVEC>
  static inline void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau) {
    const int sizeT = T.size();
    tau.resize(sizeT);
    for (int i = 0; i < sizeT; i++) {
      tau(i) = T(i) > 1.0 ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / T(i) - 1.0));
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradT(const Eigen::VectorXd &tau,
                                   const Eigen::VectorXd &gradT,
                                   EIGENVEC &gradTau) {
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    double denSqrt;

    for (int i = 0; i < sizeTau; i++) {
      if (tau(i) > 0) {
        gradTau(i) = gradT(i) * (tau(i) + 1.0);
      } else {
        denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
        gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
      }
    }

    return;
  }

  static inline void backwardGradT(const double *tau,
                                   const Eigen::VectorXd &gradT,
                                   double *gradTau, const int sizeTau) {
    double denSqrt;

    for (int i = 0; i < sizeTau; i++) {
      if (tau[i] > 0) {
        gradTau[i] = gradT[i] * (tau[i] + 1.0);
      } else {
        denSqrt = (0.5 * tau[i] - 1.0) * tau[i] + 1.0;
        gradTau[i] = gradT[i] * (1.0 - tau[i]) / (denSqrt * denSqrt);
      }
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradP(const Eigen::VectorXd &xi,
                                   const Eigen::Matrix3Xd &gradP,
                                   EIGENVEC &gradXi) {
    const int sizeP = gradP.cols();
    for (int i = 0; i < sizeP; ++i) {
      gradXi.segment(3 * i, 3) = gradP.col(i);
    }
    return;
  }
  static inline void backwardGradP(const Eigen::Matrix3Xd &gradP,
                                   double *gradXi, const int Space_dim) {
    for (int i = 0; i < Space_dim; ++i) {
      Eigen::Map<Eigen::VectorXd>(gradXi + 3 * i, 3) = gradP.col(i);
    }
    return;
  }
  //      smoothedL1(safety_hor - sdf_value, 0.01, sdf_cost, sdf_out_grad);
  static inline bool smoothedL1(const double &x, const double &mu, double &f,
                                double &df) {
    if (x < 0.0) {
      return false;
    } else if (x > mu) {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    } else {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return true;
    }
  }

  // 	Temporal Variables (T_i): Time durations allocated to each piece of the
  // trajectory. 	Spatial Variables (P_i): Control points that define the
  // shape of the trajectory in space.
  //  These variables are collectively stored in x_variable and are updated
  //  during each iteration of the optimizer.
  static inline double costFunctionLmbmParallel(void *ptr,
                                                const double *x_variable,
                                                double *g, const int n) {
    TrajOptimizer &obj = *(TrajOptimizer *)ptr;
    const int dimTau = obj.temporalDim;
    const int dimXi = obj.spatialDim;
    const double weightT = obj.rho;

    // Forward mapping from optimization variables to times and points
    forwardT(x_variable, obj.times, dimTau);
    forwardP((x_variable + dimTau), obj.points, dimXi / 3);

    // Initialize cost components
    double pos_cost = 0.0;
    double time_cost = 0.0;
    double total_cost = 0.0;
    double energy_cost = 0.0;
    // This sets the internal parameters of your trajectory representation to
    // the current optimization variables.
    obj.minco.setParameters(obj.points, obj.times);

    // --------------------
    // Energy Cost Component
    // --------------------
    obj.minco.getEnergy(energy_cost);
    total_cost += energy_cost;

    // Compute energy gradients
    obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs); // ∂E/∂c
    obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes); // ∂E/∂T
    // Get the current trajectory
    obj.minco.getTrajectory(obj.step_traj);
    // Update the trajectory in swept volume manager for collision checking for
    // current trajectory
    obj.sv_manager->updateTraj(obj.step_traj);

    auto start1 = chrono::high_resolution_clock::now();

    // -------------------------------------
    // Position (Collision) Cost Component
    // -------------------------------------

    addSaftyPenaOnSweptVolumeParallelTrueSDF(
        ptr, obj.times, obj.minco.getCoeffs(), pos_cost, obj.partialGradByTimes,
        obj.partialGradByCoeffs);

    auto end1 = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::nanoseconds>(end1 - start1);
    total_cost += pos_cost;

    // -------------------------------------
    // Time Cost Component and Gradient
    // -------------------------------------
    time_cost = weightT * obj.times.sum();
    total_cost += time_cost;

    // -------------------------------------
    // Propagate Gradients to Points and Times
    // -------------------------------------
    obj.minco.propogateGrad(obj.partialGradByCoeffs,
                            obj.partialGradByTimes, // ∂Cost/∂c,T -> ∂Cost/∂q,T
                            obj.gradByPoints, obj.gradByTimes);

    // Add time cost gradient
    obj.gradByTimes.array() += weightT; // ∂(rho * T.sum())/∂T = rho

    // Map gradients back to optimization variables
    backwardGradT(x_variable, obj.gradByTimes, g, dimTau);
    backwardGradP(obj.gradByPoints, g + dimTau, dimXi / 3);

    // Store cost components for logging or debugging
    obj.energy_cost_global = energy_cost;
    obj.pos_cost_global = pos_cost;
    obj.time_cost_global = time_cost;
    obj.cost_total = total_cost;
    obj.recordEvaluation(x_variable, n, total_cost, pos_cost);

    return total_cost;
  }

  static inline void
  addSaftyPenaOnSweptVolume(void *ptr, const Eigen::VectorXd &T,
                            const Eigen::MatrixX3d &coeffs, double &cost,
                            Eigen::VectorXd &gradT, Eigen::MatrixX3d &gradC) {
    TrajOptimizer &obj = *(TrajOptimizer *)ptr;
    int interation = obj.iter;
    double weightPos = obj.weight_p;

    double thr, cos_theta, pena;
    pena = 0;

    Eigen::Vector3d pos, vel, acc, jer, sna, tmp_pos;
    Eigen::Vector4d quat;
    Eigen::Vector3d omg;
    double gradPsiT, gradDPsiT;

    double s1, s2, s3, s4, s5;
    double violaVelPenaD, violaAccPenaD;
    double violaVelPena, violaAccPena, violaPosPena;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double grad_yaw, violaYawGrad;
    Eigen::Vector3d gradVel, gradAcc, gradPos, gradOmg, violaPosPenaGrad;
    Eigen::Vector3d gradVelTotal, gradAccTotal, gradPosTotal, gradJerTotal;
    Eigen::Matrix3d rotate;

    Eigen::Vector3d gradp_rel{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gradp_eva{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d Rt, St;
    Eigen::Vector3d pos_eva;
    Eigen::Vector3d last_pos_eva;
    bool first_obs;
    bool set_ts;
    double neighbor_dis = 2 * obj.conf.occupancy_resolution;
    double sdf_value = 0.0;
    double time_star = 0.0;
    double time_seed_f = 0.0;
    double time_seed_tmp = 0.0;
    double time_local = 0.0;
    double yaw;

    const int pieceNum = T.size();

    first_obs = true;
    vector<Eigen::Vector3d> pos_eva_vec;

    int index = 0;
    obj.ori_cost_pos = 0.0;
    for (auto iter = obj.pcsmap_manager->aabb_points.begin();
         iter != obj.pcsmap_manager->aabb_points.end(); ++iter) {
      pos_eva = iter->second;
      pos_eva(2) = 0;
      set_ts = false;

      sdf_value = obj.sv_manager->getTrueSDFofSweptVolume<true>(
          pos_eva, time_star, gradp_rel, set_ts);

      time_seed_f = time_star;
      time_local = time_seed_f;
      int i = obj.step_traj.locatePieceIdx(
          time_local); // 改变这个esdf_temp，locatePieceIdx为引用，不能传入time_seed_f
      const Eigen::Matrix<double, 6, 3> &c =
          coeffs.block<6, 3>(i * 6, 0); // 确定这段的Id
      s1 = time_local;
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3,
      beta0(4) = s4, beta0(5) = s5;
      beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2,
      beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
      beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1,
      beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
      beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0,
      beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
      beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0,
      beta4(4) = 24.0, beta4(5) = 120.0 * s1;

      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;
      sna = c.transpose() * beta4;

      gradVel.setZero();
      gradAcc.setZero();
      gradPos.setZero();
      gradOmg.setZero();
      grad_yaw = 0.0;
      gradPosTotal.setZero();
      gradVelTotal.setZero();
      gradAccTotal.setZero();
      gradJerTotal.setZero();
      yaw = pos(2);
      violaYawGrad = 0;
      violaPosPena = 0;
      rotate =
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      St = obj.sv_manager->getScale(time_seed_f);
      pena = 0.0;
      pos(2) = 0.0;

      if (sdf_value < 0) {
        gradp_rel = rotate.transpose() * gradp_rel;
      }
      if (obj.grad_cost_p_sw(pos_eva, pos, rotate, yaw, St, sdf_value,
                             gradp_rel, violaPosPenaGrad, violaYawGrad,
                             violaPosPena)) {
        gradPos += weightPos * violaPosPenaGrad;
        grad_yaw += weightPos * violaYawGrad;
        pena += weightPos * violaPosPena;
        cost += pena;
        obj.ori_cost_pos += violaPosPena;
      }

      gradPosTotal = gradPos;
      gradPosTotal(2) = grad_yaw;
      Eigen::Matrix<double, 6, 3> gdC =
          (beta0 * gradPosTotal.transpose() + beta1 * gradVelTotal.transpose() +
           beta2 * gradAccTotal.transpose() + beta3 * gradJerTotal.transpose());

      double gdT = (-gradPosTotal.dot(vel) + -gradVelTotal.dot(acc) +
                    -gradAccTotal.dot(jer) + -gradJerTotal.dot(sna));

      gradC.block<6, 3>(i * 6, 0) += gdC;
      for (int j = 0; j < i; j++) {
        gradT(j) += gdT;
      }
      index++;
    }

    return;
  }

  static inline void addSaftyPenaOnSweptVolumeParallelTrueSDF(
      void *ptr, const Eigen::VectorXd &T, const Eigen::MatrixX3d &coeffs,
      double &cost, Eigen::VectorXd &gradT, Eigen::MatrixX3d &gradC) {
    // cout << "Start to add safety penalty on swept volume" << endl;
    TrajOptimizer &obj = *(TrajOptimizer *)ptr;
    const double weightPos = obj.weight_p;
    const double neighbor_dis = 2 * obj.conf.occupancy_resolution;
    const int pieceNum = T.size();
    // Per evaluation total, same as the serial version keeps. Without the
    // reset it accumulated across every cost evaluation of the solve.
    obj.ori_cost_pos = 0.0;
#pragma omp parallel for num_threads(obj.threads_num) schedule(dynamic)
    for (int pointindex = 0; pointindex < obj.parallel_points_num;
         ++pointindex) {
      // int thread_num = omp_get_thread_num();
      // printf("Thread number: %d\n", thread_num);
      Eigen::Vector3d pos_eva = obj.parallel_points[pointindex];
      pos_eva(2) = 0;
      Eigen::Vector3d gradp_rel;
      double time_star;
      double sdf_value = obj.sv_manager->getTrueSDFofSweptVolume<true>(
          pos_eva, time_star, gradp_rel, false);
      double mag = gradp_rel.norm();

      double time_seed_f = time_star;
      double time_local = time_seed_f;
      int i = obj.step_traj.locatePieceIdx(
          time_local); // 改变这个esdf_temp，locatePieceIdx为引用，不能传入time_seed_f
      const Eigen::Matrix<double, 6, 3> &c =
          coeffs.block<6, 3>(i * 6, 0); // 确定这段的Id
      double s1 = time_local;
      double s2 = s1 * s1;
      double s3 = s2 * s1;
      double s4 = s2 * s2;
      double s5 = s4 * s1;
      Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
      beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3,
      beta0(4) = s4, beta0(5) = s5;
      beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2,
      beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
      beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1,
      beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
      beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0,
      beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
      beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0,
      beta4(4) = 24.0, beta4(5) = 120.0 * s1;

      Eigen::Vector3d pos = c.transpose() * beta0;
      Eigen::Vector3d vel = c.transpose() * beta1;
      Eigen::Vector3d acc = c.transpose() * beta2;
      Eigen::Vector3d jer = c.transpose() * beta3;
      Eigen::Vector3d sna = c.transpose() * beta4;
      Eigen::Vector3d gradPos, gradOmg, violaPosPenaGrad, gradPosTotal,
          gradVelTotal, gradAccTotal, gradJerTotal;

      gradPos.setZero();
      gradOmg.setZero();
      double grad_yaw = 0.0;
      gradPosTotal.setZero();
      gradVelTotal.setZero();
      gradAccTotal.setZero();
      gradJerTotal.setZero();

      double yaw = pos(2);
      Eigen::Matrix3d rotate =
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      Eigen::Matrix3d St = obj.sv_manager->getScale(time_seed_f);
      double pena = 0.0;
      pos(2) = 0.0;
      double violaYawGrad = 0;
      double violaPosPena = 0;
      double ori_pena = 0.0;
      if (sdf_value < 0) {
        gradp_rel = rotate.transpose() * gradp_rel;

      } // 内部世界坐标系下SDF梯度不对
      if (obj.grad_cost_p_sw(pos_eva, pos, rotate, yaw, St, sdf_value,
                             gradp_rel, violaPosPenaGrad, violaYawGrad,
                             violaPosPena)) {
        gradPos += weightPos * violaPosPenaGrad;
        grad_yaw += weightPos * violaYawGrad;
        pena += weightPos * violaPosPena;
        ori_pena = violaPosPena;
      }

      gradPosTotal = gradPos;
      gradPosTotal(2) = grad_yaw;

      Eigen::Matrix<double, 6, 3> gdC =
          (beta0 * gradPosTotal.transpose() + beta1 * gradVelTotal.transpose() +
           beta2 * gradAccTotal.transpose() + beta3 * gradJerTotal.transpose());

      double gdT = (-gradPosTotal.dot(vel) + -gradVelTotal.dot(acc) +
                    -gradAccTotal.dot(jer) + -gradJerTotal.dot(sna));
#pragma omp critical
      {
        cost += pena;
        // Accumulated here, not in the loop body: every thread writes this
        // member, so an unsynchronised += lost updates and tore the double.
        obj.ori_cost_pos += ori_pena;
        gradC.block<6, 3>(i * 6, 0) += gdC;
        for (int j = 0; j < i; j++) {
          gradT(j) += gdT;
        }
      }
    }

    return;
  }

public:
  TrajOptimizer() {}
  ~TrajOptimizer() {}

  void setParam(ros::NodeHandle &nh, Config &config) {
    this->conf = config;
    this->nh = nh;

    vis.reset(new vis::Visualization(nh));

    vmax = config.vmax;
    omgmax = config.omgmax;
    thetamax = config.thetamax;
    threads_num = config.threads_num;
    rho = config.rho;
    weight_a = config.weight_a;
    weight_p = config.weight_p;
    weight_v = config.weight_v;
    weight_omg = config.weight_omg;
    weight_theta = config.weight_theta;

    smooth_fac = config.smoothingEps;
    integralRes = config.integralIntervs;
    safety_hor = config.safety_hor;
    bdx = bdy = bdz = config.kernel_size * config.occupancy_resolution;
    // kernel_size    = config.kernel_size;
    // side_size      = max(max(bdx,bdy),bdz);

    ROS_WARN_STREAM("==[back_end]==");
    ROS_WARN_STREAM("==vmax==" << vmax);
    ROS_WARN_STREAM("==omgmax==" << omgmax);
    ROS_WARN_STREAM("==thetamax==" << thetamax);
    ROS_WARN_STREAM("==rho==" << rho);
    ROS_WARN_STREAM("==weight_v==" << weight_v);
    ROS_WARN_STREAM("==weight_a==" << weight_a);
    ROS_WARN_STREAM("==weight_p==" << weight_p);
    ROS_WARN_STREAM("==smooth_fac==" << smooth_fac);
    ROS_WARN_STREAM("==safety_hor==" << safety_hor);
    ROS_WARN_STREAM(
        "==init time allocate perpiece for minco==" << config.inittime);
    ROS_WARN_STREAM("bounding box bdx" << bdx);
    ROS_WARN_STREAM("bounding box bdy" << bdy);
    ROS_WARN_STREAM("bounding box bdz" << bdz);

    // flatmap;
    flatmap.reset(config.vehicleMass, config.gravAcc, config.horizDrag,
                  config.vertDrag, config.parasDrag, config.speedEps);

    aabb_points_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "aabbpoints", 1, true);
    debug_step_pub = nh.advertise<visualization_msgs::Marker>(
        "/traj_opt/debug_step_path", 10);
    debugesdfpub =
        nh.advertise<std_msgs::Float32MultiArray>("/traj_opt/esdf", 10);
    debug_wplists_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "/traj_opt/debug_wplists", 10);
    debug_wp_pub =
        nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_wp", 10);
    debug_vec_pub =
        nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_vec", 10);
    test_map_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/traj_opt/debug_test_map", 10);
  }

  void setEnvironment(SweptVolumeManager::Ptr sv) { sv_manager = sv; }
  void setGridMap(PCSmapManager::Ptr psm) { pcsmap_manager = psm; }

  void clearvisAABBpoints();

  int optimize_traj_lmbm(const Eigen::Matrix3d &initS,
                         const Eigen::Matrix3d &finalS, Eigen::VectorXd &opt_x,
                         const int N, Trajectory<TRAJ_ORDER> &traj);

  inline void cubic(double x, double &cost_x, double &grad_x) {
    if (x < 0) {
      cost_x = 0;
      grad_x = 0;
      return;
    }
    cost_x = pow(x, 3);
    grad_x = 3 * x * x;
    return;
  }

  bool inline grad_cost_p_sw(const Eigen::Vector3d &pos_eva,
                             const Eigen::Vector3d &pos_obj,
                             const Eigen::Matrix3d &rotate, const double &yaw,
                             const Eigen::Matrix3d &St, const double sdf_value,
                             const Eigen::Vector3d &gradp_rel,
                             Eigen::Vector3d &gradp, double &grad_yaw,
                             double &costp) {
    costp = 0.0;
    gradp.setZero();
    grad_yaw = 0.0;
    double sdf_cost = -1.0;
    Eigen::Vector3d p_minus_x;
    Eigen::Matrix3d VR_theta = Eigen::Matrix3d::Zero();
    double sdf_out_grad = 0.0;
    // safety_hor: 0.7
    // when sdf_value > safety_hor, this function returns false
    // if sdf_value < safety_hor, this function returns true, sdf_cost is almost
    // same with safety_hor - sdf_value, sdf_out_grad = 1
    smoothedL1(safety_hor - sdf_value, 0.01, sdf_cost, sdf_out_grad);
    // it did not invert the gradient, just transform the relative gradient from
    // robot local frame to global frame
    Eigen::Vector3d sdf_grad =
        -sdf_out_grad * (-(St.inverse()).transpose() * rotate * gradp_rel);

    if (sdf_cost > 0) {
      costp += sdf_cost;
      gradp += sdf_grad;
      p_minus_x = pos_eva - pos_obj;
      VR_theta(0, 0) = -sin(yaw);
      VR_theta(0, 1) = -cos(yaw);
      VR_theta(1, 0) = cos(yaw);
      VR_theta(1, 1) = -sin(yaw);
      VR_theta(2, 2) = 1;
      grad_yaw = -sdf_out_grad * gradp_rel.transpose() *
                 (VR_theta.transpose() * p_minus_x);
    }
    // when costp > 0, it returns true
    return (costp > 0);
  }

  inline void resetSE2Termination() {
    recent_evals.clear();
    have_iterate_eval = false;
    iterate_violations.clear();
    se2_stop_reason.clear();
    se2_start_time = std::chrono::steady_clock::now();
  }

  inline void recordEvaluation(const double *x, const int n, double total,
                               double pos) {
    if (recent_evals.size() >= 8) {
      recent_evals.pop_front();
    }
    recent_evals.push_back({std::vector<double>(x, x + n), total, pos});
  }

  // Returns why the SE2 solve should stop at iterate x, or nullptr to go on.
  inline const char *checkSE2Termination(const double *x) {
    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - se2_start_time)
                               .count();
    if (conf.se2_max_time > 0.0 && elapsed >= conf.se2_max_time) {
      return "time budget reached";
    }
    if (conf.se2_max_iter > 0 && iter >= conf.se2_max_iter) {
      return "iteration cap reached";
    }

    // Costs of the current iterate: unchanged after a null step, otherwise
    // one of the latest evaluations.
    const int n = temporalDim + spatialDim;
    const size_t bytes = n * sizeof(double);
    if (!have_iterate_eval ||
        std::memcmp(iterate_eval.x.data(), x, bytes) != 0) {
      auto it = std::find_if(recent_evals.rbegin(), recent_evals.rend(),
                             [&](const EvalRecord &r) {
                               return std::memcmp(r.x.data(), x, bytes) == 0;
                             });
      if (it == recent_evals.rend()) {
        return nullptr; // costs of this iterate unknown, skip the cost test
      }
      iterate_eval = *it;
      have_iterate_eval = true;
    }
    // Collision term in metres: summed intrusion of obstacle points into the
    // safety_hor band around the swept volume, independent of weight_p.
    const double violation = weight_p > 0.0 ? iterate_eval.pos / weight_p : 0.0;
    iterate_violations.push_back(violation);

    // Solves that end up safe settle at a small intrusion; hopeless ones
    // plateau far above it within the first second and then grind on for
    // seconds. Stop once the intrusion is large and no longer shrinking.
    const int w = conf.se2_stuck_window;
    if (w > 0 && violation > conf.se2_stuck_violation &&
        static_cast<int>(iterate_violations.size()) > w) {
      const double past = iterate_violations[iterate_violations.size() - 1 - w];
      if ((past - violation) / past < conf.se2_stuck_rel_tol) {
        return "stuck in collision";
      }
    }
    return nullptr;
  }

  static inline int earlyExitLMBM(void *instance, const double *x,
                                  const int k) {
    TrajOptimizer &obj = *(TrajOptimizer *)instance;
    obj.iter++;
    // std::cout << "iter = " << obj.iter << std::endl;
    if (obj.conf.enableearlyExit) {
      obj.drawStepTraj();
      vector<double> costs;
      costs.push_back(obj.cost_pos);
      costs.push_back(obj.cost_other);
      debug_publisher::DBSendLogCost(costs);
      // Plain sleep only: this runs inside the waypoint callback, and calling
      // ros::spinOnce() here would let a new goal or map callback re-enter the
      // planner and tear down the LMBM state of the solve still in progress.
      std::this_thread::sleep_for(std::chrono::milliseconds(obj.conf.debugpause));
      // std::cout << "[total cost] = " << obj.cost_total << " , [energy cost] =
      // " << obj.energy_cost_global << " , [time cost] = " <<
      // obj.time_cost_global
      //           << " , [pos cost] = " << obj.pos_cost_global << std::endl;
    }

    if (obj.exit) {
      obj.exit = false;
      obj.se2_stop_reason = "exit requested";
      return kLmbmStop;
    }

    // LMBM's own tests (se2_past / se2_delta_past) still apply; these add a
    // budget and a stop once the trajectory is safe and no longer improving.
    if (const char *reason = obj.checkSE2Termination(x)) {
      obj.se2_stop_reason = reason;
      return kLmbmStop;
    }
    return 0;
  }

public:
  typedef shared_ptr<TrajOptimizer> Ptr;
};
