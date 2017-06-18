#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ardrone_autonomy/Navdata.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <zbar.h>
#include "std_srvs/Empty.h"

#include "qr_reader.hpp"

using namespace cv;
using namespace cv_bridge;

#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

typedef struct _POSITION {
  _POSITION() : x(0.0), y(0.0) {}

  double x;
  double y;
} POSITION;

uint state;

int ringIndex = 0;
bool foundQR = false;

float batteryLevel;
float vx = 0, vy = 0, vz = 0;
float rotz = 0;
bool wait_for_navdata = true;
int altitude = 0;
POSITION pos;

bool isTakingOff = true;
bool isLanding = false;
bool isEmergencyLanding = false;

ros::Subscriber nav_sub;
ros::Publisher takeoff_pub;
ros::Publisher land_pub;
ros::Publisher fly_pub;
ros::Publisher flatTrim_pub;

pthread_t key_thread;
pthread_mutex_t key_mutex;



void navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& msg) {
  batteryLevel = msg->batteryPercent;
  vx = msg->vx;
  vy = msg->vy;
  vz = msg->vz;
  rotz = msg->rotZ;
  altitude = msg->altd;
  wait_for_navdata = false;
  if (msg->state != state) {
    ROS_INFO("State is: %u", state);
    state = msg->state;
  }
}

 //vx, vy, vz, az = bewteen -1 and 1 (nothing- more)
geometry_msgs::Twist drone_vector(double new_vx, double new_vy, double new_vz, 
  double new_ax, double new_ay, double new_az, double K) {
  geometry_msgs::Twist twist_msg;
  memset(&twist_msg, 0, sizeof(geometry_msgs::Twist));

  twist_msg.linear.x = new_vx;
  twist_msg.linear.y = new_vy;
  twist_msg.linear.z = new_vz;
  twist_msg.angular.x = new_ax;
  twist_msg.angular.y = new_ay;
  twist_msg.angular.z = new_az;
  return twist_msg;
}

geometry_msgs::Twist reset_vector() 
{
	return drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void calculatePosition(double dist)
{
  double dx = cosd(rotz) * dist;
  double dy = (rotz >= 0 ? 1 : -1) * sqrt(pow(dist, 2) - pow(dx, 2));

  std::cout << "dist: " << dist << std::endl;
  std::cout << "dx: " << dx << std::endl;
  std::cout << "dy: " << dy << std::endl;

  pos.x += dx;
  pos.y += dy;
}

void updatePosition() {
  static uint64_t first_tick = (ros::Time::now().toNSec() / 1000000); 
  static uint64_t last_tick = first_tick;

  uint64_t tick_now = (ros::Time::now().toNSec() / 1000000);  // T in milliseconds (10^3)
  uint64_t dt = (tick_now - last_tick);           // delta T

  double dist = static_cast<double>(dt / 1000.0) * vx;

  calculatePosition(dist);
  
  std::cout << "Position: " << pos.x << ", " << pos.y << std::endl;
  std::cout << "Rotation: " << rotz << std::endl;
  std::cout << "Velocities: " << vx << ", " << vy << std::endl;
  std::cout << "Since first: " << ((tick_now - first_tick) / 1000.0) << std::endl << std::endl;

  last_tick = (ros::Time::now().toNSec() / 1000000);
}

void* wait_for_input_key(void* arg) 
{
  char key = 0;

  while (!isLanding && !isEmergencyLanding) {
    std::cin >> key;

    pthread_mutex_lock(&key_mutex);

    if (key == 'X' || key == 'x')
      isEmergencyLanding = true;

    pthread_mutex_unlock(&key_mutex);
  }
}

void takeoff(ros::Publisher takeoff_pub, ros::Rate loop_rate) {
  static int startTakeOff = ros::Time::now().toSec();

  if (ros::Time::now().toSec() - startTakeOff >= 5) {
    std::cout << "Has finished taking off" << std::endl;
    isTakingOff = false;
  }
  
  takeoff_pub.publish(std_msgs::Empty());
}

void increaseAltitude(ros::Publisher publisher, ros::Rate loop_rate) {
  publisher.publish(drone_vector(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0));
}

void land(ros::Publisher land_pub) {
  if (altitude < 20) {
    std::cout << "Has finished landing" << std::endl;
    isLanding = true;
    exit(0);
  }
  
  land_pub.publish(std_msgs::Empty());
}

Point circle_center;
int circle_radius = 0;

double calCircleDist() 
{
  int ring_radius = 50; //cm - from ring 1 and 2
  double focus_l = 503.24;

  return ((ring_radius * focus_l) / circle_radius);
}

bool find_circle(Mat& img, Mat gray) 
{
  GaussianBlur(gray, gray, Size(9, 9), 2, 2);

  std::vector<Vec3f> circles;
  HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray.rows / 4, 30, 300);
  //HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray.rows / 4, 50, 250, 250 );
  
  for (std::size_t i = 0; i < circles.size(); i++)
  {
    circle_center = Point(cvRound(circles[i][0]), cvRound(circles[i][1]));    
    circle_radius = cvRound(circles[i][2]);

    circle(img, circle_center, 3, Scalar(0, 255, 0), -1, 8, 0); // draw the circle center
    circle(img, circle_center, circle_radius, Scalar(0, 0, 255), 3, 8, 0); // draw the circle outline
  }

  return (circles.size() > 0);
}

int image_width = 0;
int image_height = 0;

void path(ros::Publisher publisher, ros::Rate loop_rate) {
	float dx = (image_width / 2) - circle_center.x;
	float dy = (image_height / 2) - circle_center.y;

	float abs_dx = abs(dx);
	float abs_dy = abs(dy);

	float temp_vx = 0.0;
	float temp_vy = 0.0;
	float temp_vz = 0.0;

	float deviant = 30.0;

	if (abs_dx <= deviant && abs_dy <= deviant)
	{
		printf(".... Moving forward .....\n");
		temp_vx = 0.25;
		printf("... Done moving forward...\n");
	}
	else
	{
		if (dx < 0) // center too far right
			temp_vy = -0.01;
		else if (dx > 0) // center too far left
			temp_vy = 0.01;

		if (dy < 0) // center too low
			temp_vz = -0.025;
		else if (dy > 0) // center too high
			temp_vz = 0.025;

		if (calCircleDist() > 150 && (temp_vy != 0.0 || temp_vz != 0.0))
			temp_vx = 0.025;
	}

	if (temp_vx != 0.0 || temp_vy != 0.0 || temp_vz != 0.0)
		publisher.publish(drone_vector(temp_vx, temp_vy, temp_vz, 0.0, 0.0, 0.0, 0.0));

	if (temp_vx == 0.25)
		ros::Duration(3).sleep();
	
	ros::Duration(0.5).sleep();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::Mat image = cv_bridge::toCvShare(msg, "rgb8")->image;

		if (image.empty())
		{
			std::cerr << "Error: Unable to query image from capture device.\n" << std::endl;
			return;
		}

		cv::Mat gray(image.size(), CV_MAKETYPE(image.depth(), 1));  
		cv::cvtColor(image, gray, CV_RGB2GRAY);

		image_width = image.cols;
		image_height = image.rows;

	  	if (altitude >= 1500)
	  	{
	    	if (find_circle(image, gray))
	    		printf("Distance: %f\n", calCircleDist());
	    	else if (calCircleDist() < 110)
	    		fly_pub.publish(drone_vector(-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

 			imshow("Image", image);
	  	}
	  	else if (!foundQR)
	  	{
		  	std::vector<qr_code> qr_codes;
		  	
		    if (qr_code::find_qr_code(reinterpret_cast<uint8_t*>(gray.data), image.cols, image.rows, qr_codes))
		    {
		    	printf("Found a QR code\n");

		    	for (std::size_t i = 0; i < qr_codes.size(); i++)
		    	{
		    		qr_code& qr = qr_codes.at(i);

		    		char qr_index[16];
		    		sprintf(qr_index, "P.%02d", ringIndex);

		    		//if (!qr.get_data().compare(qr_index))
		    		//{

		    			MOVE_DIRECTION direction = qr.get_direction();

			    		printf("Found QR code: %s -> %d.\n", 
			    			qr_codes.at(i).get_data().c_str(), 
			    			direction);

		    			switch (direction)
		    			{
						case DIRECTION_LEFT:
							foundQR = false;
							fly_pub.publish(drone_vector(0.0, 0.05, 0.0, 0.0, 0.0, -0.05, 0.0));
							break;

						case DIRECTION_RIGHT:
							foundQR = false;
							fly_pub.publish(drone_vector(0.0, -0.05, 0.0, 0.0, 0.0, 0.05, 0.0));
							break;

						case DIRECTION_NONE:
						default:
	        				fly_pub.publish(drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
							foundQR = true;
							break;
						}


		    		//}
		    	}

		    	printf("\n");
	    		imshow ("QR Image", image);
		    }
	   }

		//imshow ("Image", cv_bridge::toCvShare(msg, "rgb8")->image);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv)
{
  namedWindow("Image");
  moveWindow("Image", 20,20);
  namedWindow("QR Image");
  moveWindow("QR Image", 20,500);

  ros::init(argc, argv, "drone_main");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);  
  nav_sub = nh.subscribe("ardrone/navdata", 10, navdata_callback);
  takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1);
  fly_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  flatTrim_pub = nh.advertise<std_msgs::Empty>("ardrone/flatTrim",1);

  while(wait_for_navdata) {
    ros::spinOnce();
  }

  flatTrim_pub.publish(std_msgs::Empty());
  fly_pub.publish(drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  int pthread_result = pthread_create(&key_thread, NULL, wait_for_input_key, NULL);
  int pthread_mutex_result = pthread_mutex_init(&key_mutex, NULL);

  // Begin main loop
  while (ros::ok()) {
    if (batteryLevel <= 20) {
      ROS_INFO("Battery level is too low, %f", batteryLevel);
      land(land_pub);
      exit(0);
    }

    pthread_mutex_lock(&key_mutex);

    if (isTakingOff)
    {
      takeoff(takeoff_pub, loop_rate);
    }
    else if (isLanding || isEmergencyLanding)
    {
      static bool reset = false;

      if (!reset)
      {
        fly_pub.publish(drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        reset = true;
      }

      land(land_pub);
    }
    else
    {
      if (altitude > 900) 
      {
	      static bool reset = false;

	      if (!reset)
	      {
	        fly_pub.publish(drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	        reset = true;
	      }

		if (foundQR && altitude < 1500)
        	increaseAltitude(fly_pub, loop_rate);
        else if (altitude >= 1500)
      		path(fly_pub, loop_rate);
       
        //updatePosition();
        //moveDrone(fly_pub, loop_rate);
      }
      else
      {
        increaseAltitude(fly_pub, loop_rate);
      }
    }
  
    pthread_mutex_unlock(&key_mutex);

    loop_rate.sleep();
    ros::spinOnce();
  }

  void* join_status = 0;
  pthread_join(key_thread, &join_status);
  pthread_mutex_destroy(&key_mutex);
  
  return 0;
}