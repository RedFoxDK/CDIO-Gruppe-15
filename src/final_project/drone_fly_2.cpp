#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <cstdlib>

bool wait_for_navdata = true;
bool isTakeOff = false;
bool isRunning = false;
bool isLanded = false;

uint state;

float batteryLevel;
float x = 0, y = 0, z = 0, altitude = 0;
float vx = 0, vy = 0, vz = 0;

double actionStart = NULL;
double drone_linear_x = 0.0;
double drone_linear_y = 0.0;
double drone_linear_z = 0.0;
double drone_angular_z = 0.0;

void takeoff(ros::Publisher takeoff_pub, ros::Rate loop_rate);
void increaseAltitude(ros::Publisher takeoff_pub, ros::Rate loop_rate);
void land(ros::Publisher land_pub);

 //vx, vy, vz, az = bewteen -1 and 1 (nothing- more)
geometry_msgs::Twist drone_vector(double new_vx, double new_vy, double new_vz, 
	double new_ax, double new_ay, double new_az, double K) {
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x += new_vx;
  twist_msg.linear.y += new_vy;
  twist_msg.linear.z += new_vz;
  twist_msg.angular.x += new_ax;
  twist_msg.angular.y += new_ay;
  twist_msg.angular.z += new_az;
  return twist_msg;
}

void navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& msg) {
 	
  batteryLevel = msg->batteryPercent;
  vx = msg->vx;
  vy = msg->vy;
  vz = msg->vz;
  altitude = msg->altd;
  wait_for_navdata = false;
  if (msg->state != state) {
  	ROS_INFO("State is: %u", state);
  	state = msg->state;
  }
  

  //ROS_INFO("Battery: %f", msg->batteryPercent);
  //ROS_INFO("Temperature: %d", msg->temp);
  //ROS_INFO("pressure: %d", msg->pressure);
  //ROS_INFO("Altitude: %d", msg->altd);
  //ROS_INFO("left/right tilt (rotX): %f", msg->rotX);
  //ROS_INFO("forward/backward tilt (rotY): %f", msg->rotY);
  //ROS_INFO("rotation (rotZ): %f", msg->rotZ);
  //ROS_INFO("linear velocity (vx): %f", msg->vx);
  //ROS_INFO("linear velocity (vy): %f", msg->vy);
  //ROS_INFO("linear velocity (vz): %f", msg->vz);
  //ROS_INFO("acceleration (ax): %f", msg->ax);
  //ROS_INFO("acceleration (ay): %f", msg->ay);
  //ROS_INFO("acceleration (az): %f", msg->az);

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "drone_fly_2");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);
  ros::Subscriber nav_sub = n.subscribe("ardrone/navdata", 10, navdata_callback);
  ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  ros::Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 1);
  ros::Publisher fly_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  
  while(wait_for_navdata) {
  	ros::spinOnce();
  }
  fly_pub.publish(drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  // Begin main loop
  while (ros::ok()) {
  	if (batteryLevel < 20) {
  	  // Begin landing procedure drone doesn't crash and burn due to lacking battery
  	  ROS_INFO("Battery level is too low, %f", batteryLevel);
      // landingProcedure()
      exit(0);
    }
    // Drone hasn't taken off an is attempting too
    if (!isTakeOff) {
      takeoff(takeoff_pub, loop_rate);
    }
    // Drone has taken off and is checking if it is still running
    else if (isRunning){
      increaseAltitude(fly_pub, loop_rate);
    }
    else if (!isLanded) {
      land(land_pub);
    }
    x += vx;
    y += vy;
    z += vz;
    ros::spinOnce();
  }
}

void takeoff(ros::Publisher takeoff_pub, ros::Rate loop_rate) {
	if (actionStart == NULL) {
		ROS_INFO("Has started taking off");
		actionStart = ros::Time::now().toSec();
	}
	if (actionStart + 8.0 < ros::Time::now().toSec()) {
		ROS_INFO("Has finished taking off");
		isTakeOff = true;
		isRunning = true;
		actionStart = NULL;
	}
  	takeoff_pub.publish(std_msgs::Empty());
  	loop_rate.sleep();
  	if (altitude > 0) {
  	  //ROS_INFO("x: %f, y: %f, a: %f", x, y, a);
  	}
}

void increaseAltitude(ros::Publisher publisher, ros::Rate loop_rate) {
	if (actionStart == NULL) {
		ROS_INFO("Is increasing Altitude");
		actionStart = ros::Time::now().toSec();
	}
	if (actionStart + 2.0 < ros::Time::now().toSec()) {
		ROS_INFO("Has finsihed increasing Altitude");
		publisher.publish(drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
		isRunning = false;
		actionStart = NULL;
	}                            //vx,  vy,  vz,  ax,  ay,  az,  k
	publisher.publish(drone_vector(0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0));
  	loop_rate.sleep();
  	if (altitude > 0) {
  	  //ROS_INFO("x: %f, y: %f, a: %f", x,y, a);
  	}
}

void land(ros::Publisher land_pub) {
  if (actionStart == NULL) {
  	ROS_INFO("Has started landing");
    actionStart = ros::Time::now().toSec();
  }
  if (actionStart + 5.0 < ros::Time::now().toSec()) {
  	ROS_INFO("Has finished landing");
    isLanded = true;
    actionStart = NULL;
    exit(0);
  }
  if (altitude > 0) {
  	//ROS_INFO("x: %f, y: %f, a: %f", x, y, a);
  }
  
  land_pub.publish(std_msgs::Empty());
}
