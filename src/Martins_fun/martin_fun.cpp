#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <cstdlib>
#include "CDIO/circle_msg.h"

bool wait_for_navdata = true;
bool isTakeOff = false;
bool isRunning = false;
bool isPath = false;
bool isflight = false;
bool isLanded = false;

bool hasFly = false;

uint state;

double Xcenter = 0.0;
double Ycenter = 0.0;

float batteryLevel;
float x = 0, y = 0, z = 0, altitude = 0;
float vx = 0, vy = 0, vz = 0;
float max_altitude = 0;

double actionStart = NULL;

void takeoff(ros::Publisher takeoff_pub, ros::Rate loop_rate);
void increaseAltitude(ros::Publisher takeoff_pub, ros::Rate loop_rate);
void land(ros::Publisher land_pub);
void path(ros::Publisher publisher, ros::Rate loop_rate);
void flight(ros::Publisher publisher, ros::Rate loop_rate);

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

geometry_msgs::Twist reset_vector() {
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = 0;
  twist_msg.linear.y = 0;
  twist_msg.linear.z = 0;
  twist_msg.angular.x = 0;
  twist_msg.angular.y = 0;
  twist_msg.angular.z = 0;
  return twist_msg;
}

void circle_callback(const CDIO::circle_msg::ConstPtr& msg) {
	Xcenter = msg->centerX;
	Ycenter = msg->centerY;
  //std::cout << Xcenter << " : " << Ycenter << std::endl;
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

  ros::init(argc, argv, "martin_fun");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);
  ros::Subscriber nav_sub = n.subscribe("ardrone/navdata", 100, navdata_callback);
  ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  ros::Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 1);
  ros::Publisher fly_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Subscriber circle_sub = n.subscribe("CDIO/circle_finder", 1000, circle_callback);
  
  while(wait_for_navdata) {
  	ros::spinOnce();
  }
  fly_pub.publish(drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  // Begin main loop
  while (ros::ok()) {
  	if (batteryLevel < 20) {
  	  // Begin landing procedure drone doesn't crash and burn due to lacking battery
  	  ROS_INFO("Battery level is too low, %f", batteryLevel);
      land(land_pub);
      exit(0);
    }

    // Drone hasn't taken off and is attempting too
    if (!isTakeOff) {
      takeoff(takeoff_pub, loop_rate);
    }
    // Drone has taken off and is checking if it is still running
    else if (isRunning){
      increaseAltitude(fly_pub, loop_rate);
    }
    else if (isPath) {
      path(fly_pub, loop_rate);
    }
    else if (isflight) {
      flight(fly_pub, loop_rate);
    }
    else if (!isLanded) {
      land(land_pub);
    }
    x += vx;
    y += vy;
    z += vz;
    if (altitude > max_altitude) {
    	max_altitude = altitude;
    }
    if ((actionStart != NULL && actionStart + 15.0 < ros::Time::now().toSec()) 
    	|| state == 8) {
    	ROS_INFO("Terminating drone due to inactivity");
      fly_pub.publish(reset_vector());
    	isTakeOff = true;
    	isRunning = false;
    	isLanded = false;
      land_pub.publish(std_msgs::Empty());
    }
    ros::spinOnce();
  }
}

void takeoff(ros::Publisher takeoff_pub, ros::Rate loop_rate) {
	if (actionStart == NULL) {
		ROS_INFO("Has started taking off");
		actionStart = ros::Time::now().toSec();
	}
	if (altitude > 750) {
		ROS_INFO("Has finished taking off");
		isTakeOff = true;
		isRunning = true;
		actionStart = NULL;
	}
  	takeoff_pub.publish(std_msgs::Empty());
  	//loop_rate.sleep();
  	if (altitude > 0) {
  	  //ROS_INFO("x: %f, y: %f, a: %f", x, y, a);
  	}
}

void increaseAltitude(ros::Publisher publisher, ros::Rate loop_rate) {
	if (actionStart == NULL) {
		ROS_INFO("Is increasing Altitude");
		actionStart = ros::Time::now().toSec();
	}

	if (Ycenter > 160 && Ycenter < 350) {
		ROS_INFO("Circle fund");
		publisher.publish(reset_vector());
    int cake4 = 2;
		isRunning = false;
    isPath = true;
		actionStart = NULL;
    hasFly = false;
	}

  if (altitude >= 2500) {
    ROS_INFO("2500 mm");
    publisher.publish(reset_vector());
    isRunning = false;
    actionStart = NULL;
    int cake2 = 2;
  }

  if (!hasFly) {  
                              // vx,  vy,  vz,  ax,  ay,  az,   k
	 publisher.publish(drone_vector(-0.00, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0));
   hasFly = true;
   int cake1 = 2;
  }
}

void path(ros::Publisher publisher, ros::Rate loop_rate) {
  if (actionStart == NULL) {
    ROS_INFO("Getting ready for inpact");
    actionStart = ros::Time::now().toSec();
  }

  if (Xcenter >= 315 && Xcenter <= 335) {
    ROS_INFO("READY FOR IMPACT");
    publisher.publish(reset_vector());
    ros::Duration(3).sleep();
    int cake = 2;
    isPath = false;
    isflight = true;
    actionStart = NULL;
    hasFly = false;
  }

  if (Xcenter < 315 && Xcenter > 0) {
    publisher.publish(drone_vector(0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0));
    int cake = 2;
    actionStart = NULL;
  }

  if (Xcenter > 335) {
    publisher.publish(drone_vector(0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0));
    int cake = 2;
    actionStart = NULL;
  }

  ros::Duration(0.2).sleep();

  publisher.publish(reset_vector());
  ros::Duration(0.5).sleep();

}

void flight(ros::Publisher publisher, ros::Rate loop_rate) {
  if (actionStart == NULL) {
    ROS_INFO("Wish me good luck!");
    actionStart = ros::Time::now().toSec();
  }
    
  if (false
    ) {
    ROS_INFO("I did it!");
    actionStart = NULL;
    ros::Duration(0.5).sleep();
    publisher.publish(reset_vector());
    ros::Duration(0.5).sleep();
    isflight = false;
  }

  if (!hasFly) {
    ROS_INFO("Im fliyng!");
    publisher.publish(drone_vector(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    int cake = 2;
    actionStart = NULL;
    hasFly = true;
  }

}

void land(ros::Publisher land_pub) {
  if (actionStart == NULL) {
  	ROS_INFO("Has started landing");
    actionStart = ros::Time::now().toSec();
  }
  if (altitude < 20) {
  	ROS_INFO("Has finished landing");
    isLanded = true;
    actionStart = NULL;
    ROS_INFO("Highest altitude is %f", max_altitude);
    exit(0);
  }
  if (altitude > 0) {
  	//ROS_INFO("x: %f, y: %f, a: %f", x, y, a);
  }
  
  land_pub.publish(std_msgs::Empty());
}
