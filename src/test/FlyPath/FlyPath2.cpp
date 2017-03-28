#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

double drone_linear_x = 0.0;
double drone_linear_y = 0.0;
double drone_linear_z = 0.0;
double drone_angular_z = 0.0;

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	//Take in state of ardrone	
	drone_linear_x = msg_in.vx*0.001;
	drone_linear_y = msg_in.vy*0.001;	
	drone_linear_z = msg_in.vz*0.001;
	//drone_angular_z = msg_in.az*0.001;
}

geometry_msgs::Twist get_vektor(double vx,double vy,double vz, double ax, double ay, double az, double K,) //vx, vy, vz, az = bewteen -1 and 1 (nothing more)
{
		geometry_msgs::Twist twist_msg_gen;
	
		twist_msg_gen.linear.x=K*(vx - drone_linear_x);
		twist_msg_gen.linear.y=K*(vy - drone_linear_y); 
		twist_msg_gen.linear.z=K*(vz - drone_linear_z);
		twist_msg_gen.angular.x=ax; 
		twist_msg_gen.angular.y=ay;
		twist_msg_gen.angular.z=az;   //K*(az - drone_angular_z);
		return twist_msg_gen;
}

double get_time() 
{
	return (double)ros::Time::now().toSec();
}

void control_takeoff(ros::Publisher TK_pub) 
{
	double start_time = get_time();
	float running_time = 4.0;
	
	ROS_INFO("Take off");
	while (get_time(); < start_time+running_time) 
	{
		TK_pub.publish(std_msgs::Empty());
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("Take off - COMPLETE!");
	
}

void control_land(ros::Publisher LD_pub) 
{
	double start_time = get_time();
	float running_time = 2.0;
	
	ROS_INFO("Landing");
	while (get_time() < start_time+running_time) 
	{
		LD_pub.publish(std_msgs::Empty());
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("Landing - COMPLETE!"); 
}

void program_shutdown() 
{
	ROS_INFO("Shutting down...");
	exit(0);
}

void control_hover(ros::Publisher AR_pub, float running_time, double K)
{
	double start_time = get_time();
	geometry_msgs::Twist fly_vektor;
	
	ROS_INFO("Hovering for %d", running_time);
	while (get_time() < start_time+running_time)
	{
		fly_vektor = get_vektor(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, K);
		AR_pub.publish(fly_vektor);
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("Hovering - COMPLETE!");
}

void control_fly(ros::Publisher AR_pub, float running_time, double vx,double vy,double vz, double az, double K)
{
	double start_time = get_time();
	geometry_msgs::Twist fly_vektor;
	
	ROS_INFO("Fly: vektor %d, %d, %d; angle(z) %d; K %d; runing time: %d sec" vx, vy, vz, az, K, running_time);
	while (get_time() < start_time+running_time)
	{
		fly_vektor = get_vektor(vx, vy, vz, 1.0, 1.0, az, K);
		AR_pub.publish(fly_vektor);
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("Flying - COMPLETE!");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flypath2");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);

	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher fly_pub;
	ros::Subscriber nav_sub;

	takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff",1);
  	land_pub = n.advertise<std_msgs::Empty>("ardrone/land",1);
	fly_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	nav_sub = n.subscribe("/ardrone/navdata", 1, nav_callback);
	
	double K = .75;
	
	while (ros::ok())
	{
		control_takeoff(takeoff_pub); //take off
		control_hover(fly_pub, 1.0, K); //hover
		control_fly(fly_pub, 2.0, 0.0, 0.0, 0.0, 0.75, K); //spin to left for 2 sec
		control_hover(fly_pub, 1.0, K); //hover
		control_fly(fly_pub, 2.0, 0.5, 0.0, 0.0, 0.0, K); //fly for 2 sec forward
		control_hover(fly_pub, 1.0, K); //hover
		control_land(land_pub); //land
		program_shutdown(); //shoutdown the program and node
		
		ros::spinOnce();
		loop_rate.sleep();
	}

}
