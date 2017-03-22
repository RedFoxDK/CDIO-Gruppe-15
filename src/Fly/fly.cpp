#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <cstdlib>

void chatterCallback(const ardrone_autonomy::Navdata::ConstPtr& msg) {
  ROS_INFO("I heard: [%f]", msg->batteryPercent);
}


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "Fly1234");
	ros::NodeHandle n;

  	std::string takeoff_channel;
  	std::string land_channel;

  	ros::Publisher takeoff_pub;// = n.advertise<std_msgs::Empty>("ardrone/land", 1);
  	ros::Publisher land_pub;// = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);

	takeoff_channel = n.resolveName("ardrone/takeoff");
 	land_channel = n.resolveName("ardrone/land");

  	takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff",1, true);
  	land_pub = n.advertise<std_msgs::Empty>("ardrone/land",1, true);

	ros::Subscriber battery_sub = n.subscribe("ardrone/navdata", 1000, chatterCallback);

	
	ROS_INFO("Prepard to take off");
	ros::Duration(0.5).sleep();
  	takeoff_pub.publish(std_msgs::Empty());
  	ROS_INFO("TAKE OFF");
  	ros::Duration(10).sleep();
  	ROS_INFO("Prepard to land");
  	land_pub.publish(std_msgs::Empty());
  	ROS_INFO("LAND");
  	return 0;
}
