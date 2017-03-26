#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>


double drone_x = 0.0;
double drone_y = 0.0;
double drone_z = 0.0;

float takeoff_time = 5.0;
float fly_time = 2.5;
float hover_time = 2.0;
float spin_time = 2.0;
float land_time = 1.5;

geometry_msgs::Twist fly_path_hover;


void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	//Take in state of ardrone	
	drone_x = msg_in.vx*0.001;
	drone_y = msg_in.vy*0.001;	
	drone_z = msg_in.vz*0.001;
}


geometry_msgs::Twist control_drone(double vx,double vy,double vz,double K)
{
		geometry_msgs::Twist twist_msg_gen;
	
		twist_msg_gen.linear.x=K*(vx - drone_x);
		twist_msg_gen.linear.y=K*(vy - drone_y); 
		twist_msg_gen.linear.z=K*(vz - drone_z);
		twist_msg_gen.angular.x=1.0; 
		twist_msg_gen.angular.y=1.0;
		twist_msg_gen.angular.z=0.0;
		return twist_msg_gen;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "flypath1");
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


	geometry_msgs::Twist fly_path;
	double start_time;
	double K = .75;
	double x_des = 0.5;

	fly_path_hover.linear.x=0.0; 
	fly_path_hover.linear.y=0.0;
	fly_path_hover.linear.z=0.0;
	fly_path_hover.angular.x=0.0;
	fly_path_hover.angular.y=0.0;
	fly_path_hover.angular.z=0.0;  

	start_time = (double)ros::Time::now().toSec();




	while(ros::ok()) {

		while ((double)ros::Time::now().toSec()< start_time+takeoff_time) //take off state
		{
			takeoff_pub.publish(std_msgs::Empty());
			//fly_pub.publish(fly_path_hover);
			ROS_INFO("Stat: Take off");
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		while (start_time+takeoff_time+fly_time+hover_time+spin_time < (double)ros::Time::now().toSec()) // landing and turn off state
		{
			fly_pub.publish(fly_path_hover);
			ROS_INFO("Stat: Landing - Hover");
			
			if (start_time+takeoff_time+fly_time+hover_time+spin_time+land_time < (double)ros::Time::now().toSec())
			{
				land_pub.publish(std_msgs::Empty());
				ROS_INFO("Shutting down...");
				exit(0);
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		while ((double)ros::Time::now().toSec() > start_time+takeoff_time && (double)ros::Time::now().toSec() < start_time+takeoff_time+fly_time)
		{
			fly_path = control_drone(x_des, 0.0, 0.0, K);
			ROS_INFO("State: Flying");
			fly_pub.publish(fly_path);

			ros::spinOnce();
			loop_rate.sleep();
		}

		while ((double)ros::Time::now().toSec() > start_time+takeoff_time+fly_time && (double)ros::Time::now().toSec() < start_time+takeoff_time+fly_time+hover_time)
		{
			fly_pub.publish(fly_path_hover);
			ROS_INFO("State: Hover");
			ros::spinOnce();
			loop_rate.sleep();
		}

		while ((double)ros::Time::now().toSec() > start_time+takeoff_time+fly_time+hover_time && (double)ros::Time::now().toSec() < start_time+takeoff_time+fly_time+hover_time+spin_time)
		{
			fly_path = control_drone(0.0, 0.0, 0.5, K);
			fly_pub.publish(fly_path);
			ROS_INFO("State: Spinning");
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		ros::spinOnce();
		loop_rate.sleep();		
	}



}
