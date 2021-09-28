/*
 * QuaternionRotation.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: lrieff
 */

#ifndef INC_APP_FANCY_QUATERNIONROTATION_HPP_
#define INC_APP_FANCY_QUATERNIONROTATION_HPP_

#include <cstdint>
#include <iostream>
#include <iomanip>

#include "app/fancy/Quaternion.hpp"
#include "app/fancy/Vec3.hpp"

template <typename T>
class QuaternionRotation
{
protected:
    Quaternion<T>   m_Q,
                    m_QInverse;
public:
    QuaternionRotation (const Quaternion<T> &q, const Quaternion<T> &qInverse) noexcept:
        m_Q (q),
        m_QInverse (qInverse)
    {}

    ~QuaternionRotation (void) = default;
public:
    Vec3<T> Perform (const Vec3<T> &vec)
    {
        return Vec3<T>::FromQuaternion (m_Q * vec.AsQuaternion () * m_QInverse);
    }

    void Merge (const QuaternionRotation<T> &rotation) {
        m_Q *= rotation.m_Q;
        m_QInverse = m_Q.Inverse ();
    }
public:
    inline Quaternion<T> q (void) const noexcept { return m_Q; }
public:
    static QuaternionRotation<T> Create (T angle, const Vec3<T> &axis) noexcept
    {
        angle /= 2.;

        const T ANGLE_SIN = static_cast<T>(std::sin (angle));

        const Quaternion<T> Q (
            static_cast<T>(std::cos (angle)),
            ANGLE_SIN * axis.i (),
            ANGLE_SIN * axis.j (),
            ANGLE_SIN * axis.k ()
        );

        return QuaternionRotation<T> (Q, Q.Inverse ());
    }

    static QuaternionRotation<T> Identity (void) noexcept {
        const Quaternion<T> Q (1., 0., 0., 0.);
        return QuaternionRotation<T> (Q, Q.Inverse ());
    }
};

typedef QuaternionRotation<double> QuaternionRotationD;

#endif /* INC_APP_FANCY_QUATERNIONROTATION_HPP_ */
