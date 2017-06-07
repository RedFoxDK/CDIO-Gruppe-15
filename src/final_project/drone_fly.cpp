#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/String.h"
#include <cstdlib>

void chatterCallback(const ardrone_autonomy::Navdata::ConstPtr& msg) {
  ROS_INFO("Battery: %f", msg->batteryPercent);
  ROS_INFO("Temperature: %d", msg->temp);
  ROS_INFO("pressure: %d", msg->pressure);
  ROS_INFO("Altitude: %d", msg->altd);
  ROS_INFO("left/right tilt (rotX): %f", msg->rotX);
  ROS_INFO("forward/backward tilt (rotY): %f", msg->rotY);
  ROS_INFO("rotation (rotZ): %f", msg->rotZ);
  ROS_INFO("linear velocity (vx): %f", msg->vx);
  ROS_INFO("linear velocity (vy): %f", msg->vy);
  ROS_INFO("linear velocity (vz): %f", msg->vz);
  ROS_INFO("acceleration (ax): %f", msg->ax);
  ROS_INFO("acceleration (ay): %f", msg->ay);
  ROS_INFO("acceleration (az): %f", msg->az);
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "drone_fly");
  
  ros::Subscriber battery_sub;
  std::string battery_channel;

  ros::NodeHandle n;
  
  battery_channel = n.resolveName("ardrone/navdata"); // bruges ikke
  battery_sub = n.subscribe("ardrone/navdata", 1000, chatterCallback);
  

  
  ros::spin();
  return 0;
}
