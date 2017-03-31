#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

class Drone_controllor {
protected:
	double dronbe_battery = -1.0;
	int drone_altd = 0; //in mm
	unsigned int drone_state = 0; 
	string drone_state_mening[10];

	double drone_linear_x = 0.0; 
	double drone_linear_y = 0.0;
	double drone_linear_z = 0.0;

	double drone_angular_x = 0.0;
	double drone_angular_y = 0.0;
	double drone_angular_z = 0.0;

	ros::NodeHandle nh;
}