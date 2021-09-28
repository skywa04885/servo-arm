/*
 * Quaternion.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: lrieff
 */

#ifndef INC_APP_FANCY_QUATERNION_HPP_
#define INC_APP_FANCY_QUATERNION_HPP_

#include <cstdint>
#include <cmath>

template <typename T>
class Quaternion
{
protected:
    T       m_R,
            m_I,
            m_J,
            m_K;
public:
    Quaternion (const Quaternion &other) noexcept:
        m_R (other.m_R),
        m_I (other.m_I),
        m_J (other.m_J),
        m_K (other.m_K)
    {}

    Quaternion (T r, T i, T j, T k) noexcept:
        m_R (r),
        m_I (i),
        m_J (j),
        m_K (k)
    {}

    Quaternion<T> operator + (const Quaternion<T> &other) const noexcept
    {
        return Quaternion<T> (
            m_R + other.m_R,
            m_I + other.m_I,
            m_J + other.m_J,
            m_K + other.m_K
        );
    }

    void operator += (const Quaternion<T> &other) noexcept
    {
        m_R += other.m_R;
        m_I += other.m_I;
        m_J += other.m_J;
        m_K += other.m_K;
    }

    Quaternion<T> operator * (const Quaternion<T> &other) const noexcept
    {
        return Quaternion<T> (
            m_R * other.m_R - m_I * other.m_I - m_J * other.m_J - m_K * other.m_K, // (x₀y₀ - x₁y₁ - x₂y₂ - x₃y₃)
            m_I * other.m_R + m_R * other.m_I - m_K * other.m_J + m_J * other.m_K, // (x₁y₀ + x₀y₁ - x₃y₂ + x₂y₃)
            m_J * other.m_R + m_K * other.m_I + m_R * other.m_J - m_I * other.m_K, // (x₂y₀ + x₃y₁ + x₀y₂ - x₁y₃)
            m_K * other.m_R - m_J * other.m_I + m_I * other.m_J + m_R * other.m_K  // (x₃y₀ - x₂y₁ + x₁y₂ + x₀y₃)
        );
    }

    void operator *= (const Quaternion<T> &other) noexcept
    {
        m_R = m_R * other.m_R - m_I * other.m_I - m_J * other.m_J - m_K * other.m_K; // (x₀y₀ - x₁y₁ - x₂y₂ - x₃y₃)
        m_I = m_I * other.m_R + m_R * other.m_I - m_K * other.m_J + m_J * other.m_K; // (x₁y₀ + x₀y₁ - x₃y₂ + x₂y₃)
        m_J = m_J * other.m_R + m_K * other.m_I + m_R * other.m_J - m_I * other.m_K; // (x₂y₀ + x₃y₁ + x₀y₂ - x₁y₃)
        m_K = m_K * other.m_R - m_J * other.m_I + m_I * other.m_J + m_R * other.m_K; // (x₃y₀ - x₂y₁ + x₁y₂ + x₀y₃)
    }

    ~Quaternion (void) = default;
public:
    T Magnitude (void) const noexcept
    {
        return static_cast<T>(std::sqrt (m_R * m_R + m_I * m_I + m_J * m_J + m_K * m_K));
    }

    Quaternion<T> Conjugate (void) const noexcept
    {
        return Quaternion<T> (m_R, - m_I, - m_J, - m_K);
    }

    Quaternion<T> Inverse (void) const noexcept
    {
        const T MAG = static_cast<T>(std::pow(Magnitude (), 2.));
        return Quaternion<T> (
              m_R / MAG,
            - m_I / MAG,
            - m_J / MAG,
            - m_K / MAG
        );
    }
public:
    inline T r (void) const noexcept { return m_R; }
    inline T i (void) const noexcept { return m_I; }
    inline T j (void) const noexcept { return m_J; }
    inline T k (void) const noexcept { return m_K; }
};

typedef Quaternion<double> QuaternionD;

#endif /* INC_APP_FANCY_QUATERNION_HPP_ */
