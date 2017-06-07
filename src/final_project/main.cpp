/*
	File Name: Start_up.cpp
	Project: CDIO - AI to a AR.Drone
	Function:
		- Setup the ROS Node and send it to main fil
*/
#include <ros/ros.h>
#include "Drone_controllor.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "CDIO-Gruppe-15");

	Drone_controllor controllor;

	return 0;
}
