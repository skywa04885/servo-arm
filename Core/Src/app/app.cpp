/*
 * app.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: lrieff
 */


#include "app/app.hpp"
#include "app/Servo.hpp"

static std::vector<std::shared_ptr<JointF>> g_Joints;

static Servo ();

extern "C" void AppSetup (void) noexcept
{
	g_Joints.reserve (6);
	g_Joints.push_back (std::make_shared<RevolvingJointF>(10., 0., -M_PI, M_PI));
	g_Joints.push_back (std::make_shared<RotationalJointF>(10., 0., -M_PI, M_PI));
	g_Joints.push_back (std::make_shared<RevolvingJointF>(10., 0., -M_PI, M_PI));
    g_Joints.push_back (std::make_shared<RevolvingJointF>(10., 0., -M_PI, M_PI));
    g_Joints.push_back (std::make_shared<RotationalJointF>(10., 0., -M_PI, M_PI));
    g_Joints.push_back (std::make_shared<RevolvingJointF>(10., 0., -M_PI, M_PI));
}

extern "C" void AppLoop (void) noexcept
{
	g_Joints.clear ();
	g_Joints.push_back (std::make_shared<RevolvingJointF>(10., 0., -M_PI, M_PI));
	g_Joints.push_back (std::make_shared<RotationalJointF>(10., 0., -M_PI, M_PI));
	g_Joints.push_back (std::make_shared<RevolvingJointF>(10., 0., -M_PI, M_PI));
    g_Joints.push_back (std::make_shared<RevolvingJointF>(10., 0., -M_PI, M_PI));
    g_Joints.push_back (std::make_shared<RotationalJointF>(10., 0., -M_PI, M_PI));
    g_Joints.push_back (std::make_shared<RevolvingJointF>(10., 0., -M_PI, M_PI));

	Vec3F target (10., 10., 10.);

	// Performs the IK Solving.
	const std::tuple<float, uint32_t> RES = IK<float> (g_Joints, target, .001, 100, 60);
    const float ACCURACY = std::get<0>(RES);
    const uint32_t DURATION = std::get<1>(RES);

    // Prints the result.
    printf ("Solved: %f, %lu\r\n", ACCURACY, DURATION);
}
