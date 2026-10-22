//
// Created by dji on 24-7-15.
//

#ifndef SRC_PID_H
#define SRC_PID_H

#pragma once

#include "visualization_msgs/MarkerArray.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

// #include "geometry_utils/sd_trajectory.h"
//  #include "geometry_utils/pulsar_trajectory.h"

namespace pulsar {
using namespace Eigen;
using namespace std;
using std::vector;

template <typename T>
class PID {
public:
    PID() {
        gain_P.setZero();
        gain_I.setZero();
        gain_D.setZero();
        err_inte.setZero();
        err.setZero();
        err_last.setZero();
    }
    ~PID() = default;

    typedef std::shared_ptr<PID> Ptr;

    void set_pid_gain(T p_term, T i_term, T d_term) {
        gain_P = p_term;
        gain_I = i_term;
        gain_D = d_term;
    }

    T pid_cal(T sp, T fb, double dt) {
        err = sp - fb;
        err_inte += err * dt;
        limit_integration();
        T err_delta = err - err_last;
        T ctrl_effort = gain_P.cwiseProduct(err) + gain_I.cwiseProduct(err_inte) + gain_D.cwiseProduct(err_delta);
        err_last = err;
        return ctrl_effort;
    }

    T pid_cal_err(T err_in, double dt) {
        err = err_in;
        err_inte += err * dt;
        limit_integration();
        T err_delta = err - err_last;
        T ctrl_effort = gain_P.cwiseProduct(err) + gain_I.cwiseProduct(err_inte) + gain_D.cwiseProduct(err_delta);
        err_last = err;
        return ctrl_effort;
    }

    void clear_integration() { err_inte = Vector3d(0, 0, 0); }

    void limit_integration() {
        float bound = 1;
        for (int i = 0; i < err_inte.size(); i++) {
            if (err_inte[i] > bound)
                err_inte[i] = bound;
            else if (err_inte[i] < -bound)
                err_inte[i] = -bound;
            if (isnan(err_inte[i]))
                err_inte[i] = 0;
        }
        cout << "err_inte = " << err_inte.transpose() << endl;
    }

private:
    T gain_P;
    T gain_I;
    T gain_D;
    T err, err_last;
    T err_inte;
};

}  // namespace pulsar

#endif  // SRC_PID_H
