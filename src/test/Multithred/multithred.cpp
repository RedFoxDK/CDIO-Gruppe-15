#include "multithred.h"

multithred::multithred(ros::NodeHandle &nh1) 
{
	nh = nh1;
	navdata_sub = nh.subscribe("ardrone/navdata", 1000, &multithred::Callback1, this);
}

//Callback
void multithred::Callback1(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
	ROS_INFO("Battery: [%f] procent", msg->batteryPercent);
	ROS_INFO("Temperature: [%d]", msg->temp);
	ROS_INFO("Pressure: [%d] Pa", msg->pressure);
	ROS_INFO("Altitude: [%d] mm", msg->altd);
}

bool multithred::run()
{
	while(nh.ok())
	{
		ros::spinOnce();
	}
	
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "multi");
	ros::NodeHandle n;
	
	

	ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff",1, true);
  	ros::Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land",1, true);

	ROS_INFO("Prepard to take off");
	ros::Duration(0.5).sleep();
  	takeoff_pub.publish(std_msgs::Empty());
  	ROS_INFO("TAKE OFF");
  	ros::Duration(5).sleep();
  	ROS_INFO("Prepard to land");
  	land_pub.publish(std_msgs::Empty());
  	ROS_INFO("LAND");

	multithred mysupernode(n);
	mysupernode.run();
  	return 0;
}
