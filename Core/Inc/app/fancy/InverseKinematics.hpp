/*
 * InverseKinematics.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: lrieff
 */

#ifndef INC_APP_FANCY_INVERSEKINEMATICS_HPP_
#define INC_APP_FANCY_INVERSEKINEMATICS_HPP_


#include <vector>
#include <memory>
#include <exception>
#include <string>
#include <chrono>

#include "stm32f7xx_hal.h"

#include "app/fancy/Quaternion.hpp"
#include "app/fancy/Vec3.hpp"
#include "app/fancy/Joint.hpp"
#include "app/fancy/ForwardKinematics.hpp"

class IKUnreachable : public std::runtime_error {
public:
    IKUnreachable (const std::string &msg):
        std::runtime_error (msg)
    {}
};

template <typename T>
inline T __IKDistance (std::vector<std::shared_ptr<Joint<T>>> &joints, const Vec3<T> &target) noexcept {
    const Vec3<T> pos = std::get<1>(FK (joints));
    return target.DistanceFrom (pos);
}

template <typename T>
inline T __IKSlope (T prev, T curr) {
    return (curr > prev ? 2. : -2.) * curr;
}

template <typename T>
inline T __IKOptimize (std::vector<std::shared_ptr<Joint<T>>> &joints, const Vec3<T> &target, double thresh, size_t n, size_t lineSearchIter) noexcept {
    std::shared_ptr<Joint<T>> &joint = joints.at (n);

    T previousDistance = __IKDistance (joints, target),
      currentDistance = 0.,
      slope;

    for (size_t i = 0; i < lineSearchIter; ++i) {
        currentDistance = __IKDistance (joints, target);
        slope = __IKSlope (previousDistance, currentDistance);

        if (currentDistance < thresh) break;
        if (joint->Update (slope)) break;

        previousDistance = currentDistance;
    }

    return currentDistance;
}

template <typename T>
std::tuple<T, uint32_t> IK (std::vector<std::shared_ptr<Joint<T>>> &joints, const Vec3<T> &target, double thresh, size_t iter, size_t lineSearchIter) noexcept {
    const uint32_t START = HAL_GetTick ();
    T error = 0.;

    for (size_t i = 0; i < iter; ++i) {
        for (ssize_t n = joints.size () - 1; n >= 0; --n) {
            error = __IKOptimize (joints, target, thresh, n, lineSearchIter);
            if (error < thresh) {
                i = iter;
                break;
            }
        }
    }

    const uint32_t END = HAL_GetTick ();
    return std::tuple<T, uint64_t> (error, END - START);
}

#endif /* INC_APP_FANCY_INVERSEKINEMATICS_HPP_ */
