/*
 * Vec3.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: lrieff
 */

#ifndef INC_APP_FANCY_VEC3_HPP_
#define INC_APP_FANCY_VEC3_HPP_

#include <cstdint>
#include <iostream>
#include <iomanip>

#include "app/fancy/Quaternion.hpp"

template <typename T>
class Vec3
{
protected:
    T           m_I,
                m_J,
                m_K;
public:
    Vec3 (T i, T j, T k) noexcept:
        m_I (i),
        m_J (j),
        m_K (k)
    {}

    Vec3<T> &operator += (const Vec3<T> &other) {
        m_I += other.m_I;
        m_J += other.m_J;
        m_K += other.m_K;

        return *this;
    }

    ~Vec3 (void) = default;
public:
    Quaternion<T> AsQuaternion (void) const noexcept {
        return Quaternion<T> (0., m_I, m_J, m_K);
    }

    T DistanceFrom (const Vec3<T> &other) const noexcept {
        T res = 0., temp;

        temp = m_I - other.m_I;
        res += temp * temp;

        temp = m_J - other.m_J;
        res += temp * temp;

        temp = m_K - other.m_K;
        res += temp * temp;

        return static_cast<T>(std::sqrt (res));
    }
public:
    static Vec3<T> FromQuaternion (const Quaternion<T> &quat) {
        return Vec3<T> (quat.i (), quat.j (), quat.k () );
    }
public:
    inline T i (void) const noexcept { return m_I; }
    inline T j (void) const noexcept { return m_J; }
    inline T k (void) const noexcept { return m_K; }
};

typedef Vec3<double> Vec3D;
typedef Vec3<float> Vec3F;

#endif /* INC_APP_FANCY_VEC3_HPP_ */
