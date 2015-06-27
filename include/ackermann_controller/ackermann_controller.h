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

#ifndef _ackermann_controller_h
#define _ackermann_controller_h

#include <ackermann_msgs/AckermannDrive.h>
#include <dual_controller_interface/dual_controller_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>

namespace ackermann_controller
{
	class AckermannController : public dual_controller_interface::DualController<hardware_interface::PositionJointInterface, hardware_interface::VelocityJointInterface>
	{
	public:
		AckermannController();
		~AckermannController();

		bool init(hardware_interface::PositionJointInterface *hw_a, hardware_interface::VelocityJointInterface *hw_b, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
		void starting(const ros::Time &time);
		void stopping(const ros::Time &time);
		void update(const ros::Time& time, const ros::Duration& period);

	private:
		void ackermannCmdCallback(const ackermann_msgs::AckermannDrive::ConstPtr &msg);

		ros::NodeHandle *nh;
		ros::NodeHandle *nh_priv;

		ros::Subscriber ackermann_sub;
		ros::Publisher odom_pub;

		std::vector<hardware_interface::JointHandle> drive_joints;
		std::vector<std::string> drive_joint_names;
		std::vector<hardware_interface::JointHandle> steering_joints;
		std::vector<std::string> steering_joint_names;

		double base_length;
		double wheel_diameter;

		double last_theta;
		double last_wheel_pos;
		nav_msgs::Odometry odom_msg;

		double cmd_timeout;
		ros::Duration since_last_cmd;
		bool have_cmd;
		bool have_last;
	};
}

#endif /* _ackermann_controller_h */
