#include <ros/ros.h>

#include "ackermann_controller/ackermann_controller.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "ackermann_controller_test");

	ackermann_controller::AckermannController ctrl;

	ros::spin();

	return 0;
}
