#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <ros/package.h>

class Drone_controllor {
protected:
	//Navdata
	double drone_battery; // in %
	int drone_altd; //in mm
	unsigned int drone_state; 
	std::string drone_state_mening[10];
	//Flight vector
	double drone_linear_x;
	double drone_linear_y;
	double drone_linear_z;
	//Angle for the vector (only use Z for spining around it self)
	double drone_angular_x;
	double drone_angular_y;
	double drone_angular_z;
	//Write to a log
	int log_time;
	//string log_msg;
	//Ros Nodehandler
	ros::NodeHandle nh;

	//sub og pub
	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher fly_pub;
	ros::Publisher reset_pub;
	ros::Subscriber nav_sub;

	//Functions 
	geometry_msgs::Twist control_drone(double vx,double vy,double vz, double ax, double ay, double az);
	void navdata_get(const ardrone_autonomy::Navdata& msg_in);
	
public:
	Drone_controllor();
	void takeoff();
	void land();
	void reset();
	//void write_log();
	double get_time_now();
	void loop();
};
