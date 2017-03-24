#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/String.h"
#include <cstdlib>

void chatterCallback(const ardrone_autonomy::Navdata::ConstPtr& msg) {
  ROS_INFO("Battery: %f", msg->batteryPercent);
  ROS_INFO("Temperature: %d", msg->temp);
  ROS_INFO("pressure: %d", msg->pressure);
  ROS_INFO("Altitude: %d", msg->altd);
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "Get_Battery");
  
  ros::Subscriber battery_sub;
  std::string battery_channel;

  ros::NodeHandle n;
  
  battery_channel = n.resolveName("ardrone/navdata"); // bruges ikke
  battery_sub = n.subscribe("ardrone/navdata", 1000, chatterCallback);
  

  
  ros::spin();
  return 0;
}
