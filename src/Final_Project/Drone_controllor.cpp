#include <Drone_controllor.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>


void Drone_controllor::navdata_get(const ardrone_autonomy::Navdata& msg_in)
{
	//Get Battery
	drone_battery = msg_in.batteryPercent
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
	write_log("Getting navdata");
}

void Drone_controllor::write_log(String msg) 
{
	double time = (double)ros::Time::now().toSec();

}
void 