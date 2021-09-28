/*
 * Joint.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: lrieff
 */

#ifndef INC_APP_FANCY_JOINT_HPP_
#define INC_APP_FANCY_JOINT_HPP_

#include <cstdint>
#include <cmath>

#include "app/fancy/QuaternionRotation.hpp"

template <typename T>
class Joint
{
protected:
public:
    virtual bool Update (T slope) noexcept = 0;
    virtual void FKNext (QuaternionRotation<T> &rotation, Vec3<T> &position) noexcept = 0;
};

typedef Joint <double> JointD;
typedef Joint <float> JointF;

template <typename T>
class __RotatingJoint : public Joint <T>
{
protected:
    T                       m_Angle,
                            m_MinAngle,
                            m_MaxAngle;
    bool                    m_PreviouslyHitLimit;
    // -- //
    Vec3<T>                 m_LengthVector;
    QuaternionRotation<T>   m_QuaternionRotation;
    // -- //
    const Vec3<T>           m_RotationAxis;
    T                       m_LearningRate;
public:
    __RotatingJoint (T angle, T minAngle, T maxAngle, const Vec3<T> &lengthVector, const Vec3<T> &rotationAxis, T unitStep):
        m_Angle (angle),
        m_MinAngle (minAngle),
        m_MaxAngle (maxAngle),
        m_PreviouslyHitLimit (false),
        // -- //
        m_LengthVector (lengthVector),
        m_QuaternionRotation (QuaternionRotation<T>::Create (m_Angle, m_RotationAxis)),
        // -- //
        m_RotationAxis (rotationAxis),
        m_LearningRate (unitStep)
    {}

    ~__RotatingJoint (void) = default;
public:
    inline void UpdateQuaternionRotation (void) noexcept
    {
        m_QuaternionRotation = QuaternionRotation<T>::Create (m_Angle, m_RotationAxis);
    }
public:
    virtual bool Update (T slope) noexcept
    {
        if (m_PreviouslyHitLimit) {
            slope *= -1.;
        }

        const T newAngle = m_Angle + (m_LearningRate * slope);
        if (!(newAngle >= m_MinAngle && newAngle <= m_MaxAngle)) {
            m_PreviouslyHitLimit = true;
            return true;
        }

        m_Angle = newAngle;
        m_PreviouslyHitLimit = false;

        UpdateQuaternionRotation ();
        return false;
    }

    virtual void FKNext (QuaternionRotation<T> &rotation, Vec3<T> &position) noexcept {
        rotation.Merge (m_QuaternionRotation);
        position += rotation.Perform (m_LengthVector);
    }
};

template <typename T>
class RotationalJoint : public __RotatingJoint <T>
{
protected:
public:
    RotationalJoint (T length, T angle, T minAngle, T maxAngle):
        __RotatingJoint<T> (angle, minAngle, maxAngle, Vec3<T>(length, 0., 0.), Vec3<T> (0., 0., 1.), M_PI / 1000.)
    {}
};

typedef RotationalJoint <double> RotationalJointD;
typedef RotationalJoint <float> RotationalJointF;

template <typename T>
class TwistingJoint : public __RotatingJoint <T>
{
protected:
public:
    TwistingJoint (T length, T angle, T minAngle, T maxAngle):
        __RotatingJoint<T> (angle, minAngle, maxAngle, Vec3<T>(length, 0., 0.), Vec3<T> (1., 0., 0.), M_PI / 1000.)
    {}
};

typedef TwistingJoint <double> TwistingJointD;

template <typename T>
class RevolvingJoint : public __RotatingJoint <T>
{
protected:
public:
    RevolvingJoint (T length, T angle, T minAngle, T maxAngle):
        __RotatingJoint<T> (angle, minAngle, maxAngle, Vec3<T>(0., length, 0.), Vec3<T> (1., 0., 0.), M_PI / 1000.)
    {}
};

typedef RevolvingJoint <double> RevolvingJointD;
typedef RevolvingJoint <float> RevolvingJointF;

template <typename T>
class LinearJoint : public Joint<T>
{
protected:
    T                       m_Extend,
                            m_MinExtend,
                            m_MaxExtend;
    bool                    m_PreviouslyHitLimit;
    // -- //
    Vec3<T>                 m_ExtendVector;
    // -- //
    T                       m_LearningRate;
public:
    LinearJoint (T extend, T minExtend, T maxExtend):
        m_Extend (extend),
        m_MinExtend (minExtend),
        m_MaxExtend (maxExtend),
        m_PreviouslyHitLimit (false),
        // -- //
        m_ExtendVector (Vec3<T>(0., 0., 0.)),
        // -- //
        m_LearningRate (.1)
    {
        UpdateExtendVector ();
    }
public:
    inline void UpdateExtendVector (void) noexcept {
        m_ExtendVector = Vec3<T> (m_Extend, 0., 0.);
    }
public:
    virtual bool Update (T slope) noexcept
    {
        if (m_PreviouslyHitLimit) {
            slope *= -1.;
        }

        const T newExtend = m_Extend + (m_LearningRate * slope);
        if (!(newExtend >= m_MinExtend && newExtend <= m_MaxExtend)) {
            m_PreviouslyHitLimit = true;
            return true;
        }

        m_Extend = newExtend;
        m_PreviouslyHitLimit = false;

        UpdateExtendVector ();
        return false;
    }

    virtual void FKNext (QuaternionRotation<T> &rotation, Vec3<T> &position) noexcept {
        position += rotation.Perform (m_ExtendVector);
    }
};

typedef LinearJoint <double> LinearJointD;
typedef LinearJoint <float> LinearJointF;

#endif /* INC_APP_FANCY_JOINT_HPP_ */
