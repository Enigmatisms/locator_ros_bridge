#pragma once
#include <vector>
#include <Eigen/Dense>

template <typename T>
struct PoseBase {
    using Vector2 = Eigen::Matrix<T, 2, 1>;
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Matrix2 = Eigen::Matrix<T, 2, 2>;
    using Matrix3 = Eigen::Matrix<T, 3, 3>;
    T x;
    T y;
    T theta;
    PoseBase() : x(0), y(0), theta(0) {}
    // loop: using while loop for normalizing angle to (-pi, pi] 
    PoseBase(T x, T y, T theta, bool loop = false): x(x), y(y) {
        if (loop == false)
            this->theta = goodAngle(theta);
        else
            this->theta = loopNormalize(theta);
    }
    PoseBase(const PoseBase& p): x(p.x), y(p.y), theta(p.theta) {}
    PoseBase(Vector3 p, bool loop = false) : x(p(0)), y(p(1)) {
        if (loop == false)
            this->theta = goodAngle(p(2));
        else
            this->theta = loopNormalize(p(2));
    }
    PoseBase(Vector2 p, T theta):
        x(p(0)), y(p(1)), theta(theta) {}
    PoseBase(const Matrix2& R, const Vector2& t):
        x(t.x()), y(t.y()), theta(atan2(R(1, 0), R(0, 0))) {}
    PoseBase(const Matrix3& M): 
        x(M(0, 2)), y(M(1, 2)), 
        theta(atan2(M(1, 0), M(0, 0))) {}
    Vector3 eigen() const {
        return Vector3(x, y, theta);
    }
    Vector3 weighted_eigen(T theta_weight) const {
        return Vector3(x, y, theta * theta_weight);
    }
    T norm2d() const {
        return std::sqrt(x * x + y * y);
    }
    void operator+=(Vector3 p) {
        x += p(0);
        y += p(1);
        theta = goodAngle(theta + p(2));
    }
    void operator-=(Vector3 p) {
        x -= p(0);
        y -= p(1);
        theta = goodAngle(theta - p(2));
    }
    PoseBase operator-(const PoseBase& p) const {
        return PoseBase(x - p.x, y - p.y, goodAngle(theta - p.theta));
    }
    PoseBase operator*(const PoseBase& p) const {
        return PoseBase(rotation() * p.translation() + translation(), goodAngle(theta + p.theta));
    }
    Vector2 translation() const {
        return Vector2(x, y);
    }
    Matrix2 rotation() const {
        Matrix2 rotate_mat;
        const T cosa = cos(theta), sina = sin(theta);
        rotate_mat << cosa, -sina, sina, cosa;
        return rotate_mat;
    }
    PoseBase inverse() const {
        T cos_theta = cos(theta);
        T sin_theta = sin(theta);
        T trans_x = x * cos_theta + y * sin_theta;
        T trans_y = x * sin_theta - y * cos_theta;
        return PoseBase(-trans_x, trans_y, -theta);
    }
    Matrix3 transform() const {
        const T cosa = cos(theta), sina = sin(theta);
        const Matrix3 res = {cosa, -sina, x, sina, cosa, y, 0, 0, 1};
        return res;
    }
    bool isnan() const {
        return (std::isnan(x) || std::isnan(y) || std::isnan(theta));
    }
    static PoseBase Identity() {
        return PoseBase(0, 0, 0);
    }
    static T goodAngle(T angle) {
        if (angle > M_PI) 
            angle -= 2 * M_PI;
        if (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }
    static T loopNormalize(T angle) {
        while (angle > M_PI) 
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }
};

typedef PoseBase<double> Pose;