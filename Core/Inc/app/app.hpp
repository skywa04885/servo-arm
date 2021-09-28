/*
 * app.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: lrieff
 */

#pragma once

#include "app/fancy/Joint.hpp"
#include "app/fancy/Quaternion.hpp"
#include "app/fancy/QuaternionRotation.hpp"
#include "app/fancy/Vec3.hpp"
#include "app/fancy/ForwardKinematics.hpp"
#include "app/fancy/InverseKinematics.hpp"

extern "C" void AppSetup (void) noexcept;

extern "C" void AppLoop (void) noexcept;
