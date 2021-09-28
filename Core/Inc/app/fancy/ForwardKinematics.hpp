/*
 * ForwardKinematics.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: lrieff
 */

#ifndef INC_APP_FANCY_FORWARDKINEMATICS_HPP_
#define INC_APP_FANCY_FORWARDKINEMATICS_HPP_

#include <vector>
#include <memory>

#include "app/fancy/Quaternion.hpp"
#include "app/fancy/Vec3.hpp"
#include "app/fancy/Joint.hpp"

template <typename T>
std::tuple<Quaternion<T>, Vec3<T>> FK (const std::vector<std::shared_ptr<Joint<T>>> &joints) {
    QuaternionRotation<T> rotation = QuaternionRotation<T>::Identity ();
    Vec3<T> position = Vec3<T> (0., 0., 0.);

    for (size_t n = 0; n < joints.size (); ++n) {
        const std::shared_ptr<Joint<T>> &joint = joints.at (n);
        joint->FKNext (rotation, position);
    }

    return std::tuple<Quaternion<T>, Vec3<T>> (rotation.q (), position);
}

#endif /* INC_APP_FANCY_FORWARDKINEMATICS_HPP_ */
