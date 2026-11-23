#pragma once 
#include <cmath>
#include <vector>
#include <Eigen/Dense>

struct DH {double a, alpha, d, theta_offset, sign;};

// EE OFFSET
constexpr double TCP_X = 0.0877;
constexpr double TCP_LAT = 0.05;

// DH
inline const std::vector<DH>& dhMat(){
    static const std::vector<DH> dh = {
        {0.01857, -M_PI / 2, 0.1168,   0.0,      -1.0},  
        {0.1549,   0.0,      0.0,      0.0,       1.0},  
        {0.16641,  0.0,      0.0,      0.0,       1.0},  
        {TCP_X,    0.0,      TCP_LAT,  0.0,       1.0},  
    };
    return dh;
}

inline Eigen::Vector4d qMin() {return Eigen::Vector4d(-1.57, -3.14, -2.62, -3.14);}
inline Eigen::Vector4d qMax() {return Eigen::Vector4d(1.57,  0.0,   2.62,  3.14);}

// FK MATRIX
inline Eigen::Matrix4d TM(double a, double alpha, double d, double theta){
    double ca = std::cos(alpha), sa = std::sin(alpha);
    double ct = std::cos(theta), st = std::sin(theta);
    Eigen::Matrix4d T;
    T << ct,    -st*ca,  st*sa, a*ct,
         st,     ct*ca, -ct*sa, a*st,
         0.0,    sa,     ca,    d,
         0.0,    0.0,    0.0,   1.0;
    return T;
}

// FK EE
inline Eigen::Matrix4d FK(const std::vector<DH>& dh, const Eigen::Vector4d& q){
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    for(size_t i = 0; i < dh.size(); ++i){
        A *= TM(dh[i].a, dh[i].alpha, dh[i].d, dh[i].sign * q[i] + dh[i].theta_offset);
    }
    return A;
}

// POS AND EE ORIENTATION
inline Eigen::Vector4d task(const std::vector<DH>& dh, const Eigen::Vector4d& q){
    Eigen::Matrix4d T = FK(dh, q);
    return Eigen::Vector4d(T(0,3), T(1,3), T(2,3), T(2,0));
}

// JACOBIAN
inline Eigen::Matrix4d jacobian(const std::vector<DH>& dh, const Eigen::Vector4d& q){
    const double shift = 1e-6;
    Eigen::Vector4d f0 = task(dh, q);
    Eigen::Matrix4d J;
    for(int i = 0; i < 4; ++i){ 
        Eigen::Vector4d qnew = q;
        qnew[i] += shift;
        J.col(i) = (task(dh, qnew) - f0) / shift;
    }
    return J;
}

// PREDICTION
inline Eigen::Vector4d seed(const Eigen::Vector3d& target){
    return Eigen::Vector4d(-std::atan2(target.y(), target.x()), -1.0, 1.5, 0.5);
}

// IK
inline bool IK(const std::vector<DH>& dh, const Eigen::Vector3d& target, Eigen::Vector4d& q){
    const Eigen::Vector4d low = qMin(), high = qMax();
    const double lambda = 0.01;
    const double maxStep = 0.2;

    Eigen::Vector4d qw = q.cwiseMax(low).cwiseMin(high);
    Eigen::Vector4d des;
    des << target, 0.0;

    bool ok = false;
    for(int i = 0; i < 300; ++i){
        Eigen::Vector4d err = des - task(dh, qw);
        if(err.head<3>().norm() < 1e-4 && std::abs(err[3]) < 1e-3) {ok = true; break;}

        Eigen::Matrix4d J = jacobian(dh, qw);
        Eigen::Matrix4d A = J * J.transpose() + lambda * lambda * Eigen::Matrix4d::Identity();
        Eigen::Vector4d dq = J.transpose() * A.ldlt().solve(err);

        double m = dq.cwiseAbs().maxCoeff();
        if(m > maxStep) dq *= maxStep / m;

        qw = (qw + dq).cwiseMax(low).cwiseMin(high);
    }
    if(!ok) return false;

    Eigen::Matrix4d T = FK(dh, qw);
    Eigen::Vector2d go(T(0,0), T(1,0));
    if(go.dot(Eigen::Vector2d(target.x(), target.y())) <= 0.0) return false;

    q = qw;
    return true;
}