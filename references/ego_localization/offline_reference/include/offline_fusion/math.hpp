#pragma once

#include <cmath>

#include "offline_fusion/types.hpp"

namespace offline_fusion {

inline Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 operator*(const Vec3& a, double s) {
    return {a.x * s, a.y * s, a.z * s};
}

inline Vec3 operator*(double s, const Vec3& a) {
    return a * s;
}

inline Vec3 operator/(const Vec3& a, double s) {
    return {a.x / s, a.y / s, a.z / s};
}

inline double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double norm(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline Quat quat_multiply(const Quat& a, const Quat& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

inline Quat quat_conjugate(const Quat& q) {
    return {q.w, -q.x, -q.y, -q.z};
}

inline Quat quat_normalized(const Quat& q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n <= 0.0) {
        return {};
    }
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

inline Quat quat_from_delta_angle(const Vec3& dtheta) {
    const double angle = norm(dtheta);
    if (angle < 1e-12) {
        return quat_normalized({1.0, 0.5 * dtheta.x, 0.5 * dtheta.y, 0.5 * dtheta.z});
    }
    const double half = 0.5 * angle;
    const double s = std::sin(half) / angle;
    return quat_normalized({std::cos(half), s * dtheta.x, s * dtheta.y, s * dtheta.z});
}

inline Vec3 quat_to_delta_angle(const Quat& q_in) {
    Quat q = quat_normalized(q_in);
    if (q.w < 0.0) {
        q = {-q.w, -q.x, -q.y, -q.z};
    }
    const double s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (s < 1e-12) {
        return {2.0 * q.x, 2.0 * q.y, 2.0 * q.z};
    }
    const double angle = 2.0 * std::atan2(s, q.w);
    return {angle * q.x / s, angle * q.y / s, angle * q.z / s};
}

inline Vec3 rotate(const Quat& q_body_to_world, const Vec3& v_body) {
    const Quat vq{0.0, v_body.x, v_body.y, v_body.z};
    const Quat rq = quat_multiply(quat_multiply(q_body_to_world, vq), quat_conjugate(q_body_to_world));
    return {rq.x, rq.y, rq.z};
}

inline Vec3 rotate_inverse(const Quat& q_body_to_world, const Vec3& v_world) {
    return rotate(quat_conjugate(q_body_to_world), v_world);
}

inline Vec3 lerp(const Vec3& a, const Vec3& b, double alpha) {
    return a * (1.0 - alpha) + b * alpha;
}

}  // namespace offline_fusion
