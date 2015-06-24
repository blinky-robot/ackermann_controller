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

#include <boost/assign.hpp>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

#include "ackermann_controller/ackermann_controller.h"

PLUGINLIB_EXPORT_CLASS(ackermann_controller::AckermannController, controller_interface::ControllerBase);

namespace ackermann_controller
{
	AckermannController::AckermannController()
		: nh(NULL),
		  nh_priv(NULL),
		  drive_joint_name("drive_motor"),
		  steering_joint_name("steering_servo"),
		  base_length(0.33),
		  wheel_diameter(0.109),
		  last_theta(-M_PI / 2.0),
		  last_wheel_pos(0.0),
		  cmd_timeout(0.25),
		  since_last_cmd(0.0),
		  have_cmd(false),
		  have_last(false)
	{
	}

	AckermannController::~AckermannController()
	{
	}

	bool AckermannController::init(hardware_interface::PositionJointInterface *hw_a, hardware_interface::VelocityJointInterface *hw_b, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
	{
		int i;
		XmlRpc::XmlRpcValue pose_cov_list;
		XmlRpc::XmlRpcValue twist_cov_list;

		nh = &root_nh;
		nh_priv = &controller_nh;

		nh_priv->param("drive_joint_name", drive_joint_name, drive_joint_name);
		nh_priv->param("base_length", base_length, base_length);
		nh_priv->param("cmd_timeout", cmd_timeout, cmd_timeout);
		nh_priv->param("child_frame_id", odom_msg.child_frame_id, std::string("base_link"));
		nh_priv->param("frame_id", odom_msg.header.frame_id, std::string("odom"));
		nh_priv->param("steering_joint_name", steering_joint_name, steering_joint_name);
		nh_priv->param("wheel_diameter", wheel_diameter, wheel_diameter);

		nh_priv->getParam("pose_covariance_diagonal", pose_cov_list);
		ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
		ROS_ASSERT(pose_cov_list.size() == 6);
		for (i = 0; i < pose_cov_list.size(); ++i)
		{
			ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		}

		nh_priv->getParam("twist_covariance_diagonal", twist_cov_list);
		ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
		ROS_ASSERT(twist_cov_list.size() == 6);
		for (i = 0; i < twist_cov_list.size(); ++i)
		{
			ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		}

		odom_msg.pose.covariance = boost::assign::list_of
			(static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
			(0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
			(0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
			(0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
			(0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
			(0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));

		odom_msg.twist.covariance = boost::assign::list_of
			(static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
			(0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
			(0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
			(0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
			(0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
			(0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));

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
		double steering_pos;
		double d_pos;
		double d_dist;
		double d_theta;
		double d_x;
		double d_y;

		drive_pos = drive_joint.getPosition();
		drive_vel = drive_joint.getVelocity();
		steering_pos = steering_joint.getPosition();

		if (have_last)
		{
			d_pos = drive_pos - last_wheel_pos;
			d_dist = d_pos * wheel_diameter / 2.0;
			d_theta = tan(steering_pos) * d_dist / base_length;
			d_x = sin(d_theta) * d_dist;
			d_y = cos(d_theta) * d_dist;

			if (d_pos < 2 * M_PI && d_pos > -2 * M_PI)
			{
				odom_msg.pose.pose.position.x += d_x * cos(last_theta) - d_y * sin(last_theta);
				odom_msg.pose.pose.position.y += d_x * sin(last_theta) + d_y * cos(last_theta);

				last_theta += d_theta;

				odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(last_theta + M_PI / 2.0);

				odom_msg.twist.twist.angular.z = d_theta / period.toSec();
				odom_msg.twist.twist.linear.x = drive_vel * wheel_diameter / 2.0;

				odom_msg.header.stamp = time;

				odom_pub.publish(nav_msgs::OdometryPtr(new nav_msgs::Odometry(odom_msg)));
			}
			else
			{
				ROS_WARN("Detected a large change in wheel position (%lf). Discarding this result...", d_pos);
			}

			if (have_cmd && since_last_cmd.toSec() > cmd_timeout)
			{
				ROS_WARN_THROTTLE(1, "Timeout receiving commands");
				drive_joint.setCommand(0.0);
			}
		}

		last_wheel_pos = drive_pos;
		since_last_cmd += period;
	}

	void AckermannController::ackermannCmdCallback(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
	{
		drive_joint.setCommand(2.0 * msg->speed / wheel_diameter); // Convert to angular
		steering_joint.setCommand(msg->steering_angle);
		since_last_cmd.fromSec(0.0);
		have_cmd = true;
	}
}
