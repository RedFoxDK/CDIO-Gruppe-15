#include "Drone_controllor.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <ros/package.h>

Drone_controllor::Drone_controllor() {

	drone_battery = -1.0;
	drone_altd = 0;
	drone_linear_x = 0.0; 
	drone_linear_y = 0.0;
	drone_linear_z = 0.0;
	drone_angular_x = 0.0;
	drone_angular_y = 0.0;
	drone_angular_z = 0.0;

	//Write the diffent state for the drone
	drone_state_mening[0] = "Unknow / Error";
	drone_state_mening[1] = "Inited";
	drone_state_mening[2] = "Landed";
	drone_state_mening[3] = "Flying";
	drone_state_mening[4] = "Hovering";
	drone_state_mening[5] = "Test (?)";
	drone_state_mening[6] = "Take off";
	drone_state_mening[7] = drone_state_mening[3];
	drone_state_mening[8] = "Landing";
	drone_state_mening[9] = "Looping (?)";
	
	//Get Ros time right now for the log
	log_time = ros::Time::now().toSec();
	
	//Communication with the AR Drone
	takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff",1);
	land_pub = nh.advertise<std_msgs::Empty>("ardrone/land",1);
	reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset",1);
	fly_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	nav_sub = nh.subscribe("/ardrone/navdata", 1, &Drone_controllor::navdata_get, this);
	//image subscriber
	
	//our Subscribe and Publisher 
	//Send Navdata og state informatioenr 
	//Get shutdown command 

}

void Drone_controllor::navdata_get(const ardrone_autonomy::Navdata& msg_in)
{
	//Get Battery
	drone_battery = msg_in.batteryPercent;
	//Get State
	drone_state = msg_in.state;
	//Get altitude
	drone_altd = msg_in.altd;
	//Get Vekor
	drone_linear_x = msg_in.vx*0.001;
	drone_linear_y = msg_in.vy*0.001;	
	drone_linear_z = msg_in.vz*0.001;
	//Get angles
	drone_angular_x = msg_in.ax*0.001;
	drone_angular_y = msg_in.ay*0.001;
	drone_angular_z = msg_in.az*0.001;
	/*log_msg = "Getting navdata";
	write_log();*/
}

double Drone_controllor::get_time_now() {
	return (double)ros::Time::now().toSec();
}

/*void Drone_controllor::write_log() {
	string msg = log_msg;
	String path = ros::package::getPath("CDIO");
	path = path + "log/CDIO_log_" + log_time + ".txt";
	
	ofstream file;
	file.open(path); //input methoed: Ros time - Msg
	file << get_time_now() + " - " + msg;
	file.close();
}*/
void Drone_controllor::loop() {
	while (ros::ok()) {
		
		ros::spinOnce();
	}
}
void Drone_controllor::takeoff() {
	takeoff_pub.publish(std_msgs::Empty());
	/*log_msg = "Taking off";
	write_log();*/
}
void Drone_controllor::land() {
	land_pub.publish(std_msgs::Empty());
	/*log_msg = "Landing";
	write_log();*/
}
void Drone_controllor::reset() {
	/*if (drone_state != 0) {
		log_msg ="Emergency shutdown of drone";
	}else{
		log_msg ="Emergency is over";
	}
	write_log();*/
	reset_pub.publish(std_msgs::Empty());
}
geometry_msgs::Twist Drone_controllor::control_drone(double vx,double vy,double vz, double ax, double ay, double az)
{
	geometry_msgs::Twist twist_msg_gen;
	twist_msg_gen.linear.x=(vx - drone_linear_x);
	twist_msg_gen.linear.y=(vy - drone_linear_y);
	twist_msg_gen.linear.z=(vz - drone_linear_z);
	twist_msg_gen.angular.x=ax; 
	twist_msg_gen.angular.y=ay;
	twist_msg_gen.angular.z=az;
	return twist_msg_gen;
}
