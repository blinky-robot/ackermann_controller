/*
 * Copyright (c) 2015, Scott K Logan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.h>

#include "ackermann_controller/ackermann_controller.h"

PLUGINLIB_EXPORT_CLASS(ackermann_controller::AckermannController, controller_interface::ControllerBase);

namespace ackermann_controller
{
	AckermannController::AckermannController()
		: nh(NULL),
		  nh_priv(NULL),
		  drive_joint_name("drive_motor"),
		  steering_joint_name("steering_servo"),
		  wheel_diameter(0.109),
		  running_wheel_position(0.0)
	{
	}

	AckermannController::~AckermannController()
	{
	}

	bool AckermannController::init(hardware_interface::PositionJointInterface *hw_a, hardware_interface::VelocityJointInterface *hw_b, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
	{
		nh = &root_nh;
		nh_priv = &controller_nh;

		nh_priv->param("drive_joint_name", drive_joint_name, drive_joint_name);
		nh_priv->param("steering_joint_name", steering_joint_name, steering_joint_name);
		nh_priv->param("wheel_diameter", wheel_diameter, wheel_diameter);

		drive_joint = hw_b->getHandle(drive_joint_name);
		steering_joint = hw_a->getHandle(steering_joint_name);

		return true;
	}

	void AckermannController::starting(const ros::Time &time)
	{
		if (!ackermann_sub)
		{
			ackermann_sub = nh->subscribe("ackermann_cmd", 1, &AckermannController::ackermannCmdCallback, this);
		}

		if (!odom_pub)
		{
			odom_pub = nh->advertise<nav_msgs::Odometry>("odom", 1);
		}
	}

	void AckermannController::stopping(const ros::Time &time)
	{
		if (ackermann_sub)
		{
			ackermann_sub.shutdown();
		}

		if (odom_pub)
		{
			odom_pub.shutdown();
		}
	}

	void AckermannController::update(const ros::Time& time, const ros::Duration& period)
	{
		nav_msgs::Odometry msg;
		double drive_pos;
		double drive_vel;
		//double steering_pos;
		//double steering_vel;

		drive_pos = drive_joint.getPosition();
		drive_vel = drive_joint.getVelocity();
		//steering_pos = steering_joint.getPosition();
		//steering_vel = steering_joint.getVelocity();

		msg.twist.twist.linear.x = drive_vel * wheel_diameter / 2.0;

		odom_pub.publish(msg);

		running_wheel_position = drive_pos;
	}

	void AckermannController::ackermannCmdCallback(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
	{
		drive_joint.setCommand(2.0 * msg->speed / wheel_diameter);

		// Hack
		steering_joint.setCommand(msg->steering_angle);
	}
}
