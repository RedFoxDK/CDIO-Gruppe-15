#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <cstdlib>

void takeoff(ros::Publisher takeoff_pub, ros::Rate loop_rate);
void land(ros::Publisher land_pub);

float batteryLevel;
float x = 0, y = 0, z = 0;
float vx, vy, vz;


void chatterCallback(const ardrone_autonomy::Navdata::ConstPtr& msg) {
  ROS_INFO("Battery: %f", msg->batteryPercent);
  batteryLevel = msg->batteryPercent;

  ROS_INFO("Temperature: %d", msg->temp);
  ROS_INFO("pressure: %d", msg->pressure);
  ROS_INFO("Altitude: %d", msg->altd);
  ROS_INFO("left/right tilt (rotX): %f", msg->rotX);
  ROS_INFO("forward/backward tilt (rotY): %f", msg->rotY);
  ROS_INFO("rotation (rotZ): %f", msg->rotZ);
  ROS_INFO("linear velocity (vx): %f", msg->vx);
  vx = msg->vx;
  vy = msg->vy;
  vz = msg->vz;
  ROS_INFO("linear velocity (vy): %f", msg->vy);
  ROS_INFO("linear velocity (vz): %f", msg->vz);
  ROS_INFO("acceleration (ax): %f", msg->ax);
  ROS_INFO("acceleration (ay): %f", msg->ay);
  ROS_INFO("acceleration (az): %f", msg->az);
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "drone_fly");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);

  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;

  takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff",1);
  land_pub = n.advertise<std_msgs::Empty>("ardrone/land",1);
  
  ros::Subscriber battery_sub;
  std::string battery_channel;

  

  
  battery_channel = n.resolveName("ardrone/navdata"); // bruges ikke
  battery_sub = n.subscribe("ardrone/navdata", 1000, chatterCallback);

  //Main loop
  while(ros::ok())
  {
    if (batteryLevel < 10){
      //ROS_INFO("Low battery level (%f), change battery", batteryLevel);
      //exit(0);
    }
    takeoff(takeoff_pub, loop_rate);
    land(land_pub);



  }
  

  
  ros::spin();
  return 0;
}

void takeoff(ros::Publisher takeoff_pub, ros::Rate loop_rate)
{
  ROS_INFO("Entered takeoff");
  double start_time = (double)ros::Time::now().toSec();
  while((float)ros::Time::now().toSec()< start_time+5.00)
  {
    takeoff_pub.publish(std_msgs::Empty());
    ROS_INFO("x: %f, y: %f, z: %f", x+=vx,y+=vy,z+=vz);
    loop_rate.sleep();
  }
}

void land(ros::Publisher land_pub)
{
  ROS_INFO("Entered Land");
  double start_time = (double)ros::Time::now().toSec();
  while((float)ros::Time::now().toSec()< start_time+5.00)
  {
    land_pub.publish(std_msgs::Empty());
    ROS_INFO("Shutting down");
    exit(0);
  }
}