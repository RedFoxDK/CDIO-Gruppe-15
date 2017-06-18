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
#include "circle_finder.hpp"

using namespace cv;
using namespace cv_bridge;

enum DRONE_STATE : unsigned char
{
	STATE_UNKNOWN = 0,
	STATE_WAIT_FOR_NAVDATA,

	STATE_TAKING_OFF,
	STATE_WAIT_FOR_TAKEOFF,

	STATE_LANDING,
	STATE_WAIT_FOR_LANDING,

	STATE_FIND_QR,

	STATE_PREPARE_FIND_CIRCLE,
	STATE_FIND_CIRCLE
};

int ring_index = 0;
double ring_distance = 0.0;

DRONE_STATE state = STATE_WAIT_FOR_NAVDATA;

float batteryLevel;
float vx = 0;
float vy = 0;
float vz = 0;
float rotz = 0;
int altitude = 0;

pthread_mutex_t input_mutex;

ros::Publisher fly_publisher;

geometry_msgs::Twist drone_vector(double new_vx, double new_vy, double new_vz, double new_ax, double new_ay, double new_az)
{
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
	return drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
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

		switch (state)
		{
		case STATE_FIND_QR:
			{
				static int last_qr_find = 0;

				std::vector<qr_code> qr_codes;

				if (qr_code::find_qr_codes(reinterpret_cast<uint8_t*>(gray.data), image.cols, image.rows, qr_codes))
				{
					last_qr_find = ros::Time::now().toSec();

					for (std::size_t i = 0; i < qr_codes.size(); i++)
					{
						qr_code& qr = qr_codes.at(i);

						char qr_index[16];
						sprintf(qr_index, "P.%02d", ring_index);

						if (qr.get_data().at(0) != 'W')
						//if (!qr.get_data().compare(qr_index))
						{
							MOVE_DIRECTION direction = qr.get_direction();

							printf("Found QR code: %s -> %d.\n", qr_codes.at(i).get_data().c_str(), direction);

							switch (direction)
							{
							case DIRECTION_LEFT:
								fly_publisher.publish(drone_vector(0.0, 0.05, 0.0, 0.0, 0.0, -0.05));
								break;

							case DIRECTION_RIGHT:
								fly_publisher.publish(drone_vector(0.0, -0.05, 0.0, 0.0, 0.0, 0.05));
								break;

							case DIRECTION_NONE:
							default:
								fly_publisher.publish(drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
								state = STATE_PREPARE_FIND_CIRCLE;
								break;
							}
						}
					}

					printf("\n");
				}
				else if (ros::Time::now().toSec() - last_qr_find >= 5)
					fly_publisher.publish(drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, -0.1));

				imshow ("QR Image", image);
			}

			break;

		case STATE_FIND_CIRCLE:
			{
				float temp_vx = 0.0;
				float temp_vy = 0.0;
				float temp_vz = 0.0;

				std::vector<ring_circle> circles;

				if (ring_circle::find_circles(image, gray, circles))
				{
					ring_circle* closest_ring = 0;

					for (std::size_t i = 0; i < circles.size(); i++)
					{
						ring_circle& ring = circles.at(i);

						if (closest_ring == 0 || ring.get_distance(0) < closest_ring->get_distance(0))
							closest_ring = &ring;
					}

					ring_distance = closest_ring->get_distance(ring_index);

					float dx = (image.cols / 2) - closest_ring->get_x();
					float dy = (image.rows / 2) - closest_ring->get_y();

					float abs_dx = abs(dx);
					float abs_dy = abs(dy);

					float deviant = 50.0;

					if (abs_dx <= deviant && abs_dy <= deviant && ring_distance < 180)
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

						if (ring_distance > 150 && (temp_vy != 0.0 || temp_vz != 0.0))
							temp_vx = 0.025;
					}

					printf("Distance: %f\n", ring_distance);
				}
				else if (ring_distance != 0.0 && ring_distance < 110)
					temp_vx = -0.1;

				if (temp_vx != 0.0 || temp_vy != 0.0 || temp_vz != 0.0)
					fly_publisher.publish(drone_vector(temp_vx, temp_vy, temp_vz, 0.0, 0.0, 0.0));

				imshow("Circle Image", image);

				if (temp_vx == 0.25)
				{
					ros::Duration(ring_distance / 100.0 + 0.5).sleep();
					fly_publisher.publish(reset_vector());
					state = STATE_FIND_QR;
				}
			}

			break;

		default:
			break;
		}

		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void* input_thread_callback(void* arg) 
{
	char key = 0;

	while (state != STATE_LANDING)
	{
		std::cin >> key;

		pthread_mutex_lock(&input_mutex);

		if (key == 'X' || key == 'x')
			state = STATE_LANDING;

		pthread_mutex_unlock(&input_mutex);
	}
}

void navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& msg) 
{
	batteryLevel = msg->batteryPercent;
	vx = msg->vx;
	vy = msg->vy;
	vz = msg->vz;
	rotz = msg->rotZ;
	altitude = msg->altd;

	if (state == STATE_WAIT_FOR_NAVDATA)
		state = STATE_TAKING_OFF;
}

int main(int argc, char **argv)
{
	namedWindow("Circle Image");
	moveWindow("Circle Image", 20,20);

	namedWindow("QR Image");
	moveWindow("QR Image", 20,500);

	ros::init(argc, argv, "drone_main");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);

	ros::Subscriber nav_sub = nh.subscribe("ardrone/navdata", 10, navdata_callback);
	ros::Publisher flat_trim_publisher = nh.advertise<std_msgs::Empty>("ardrone/flatTrim",1);
	ros::Publisher takeoff_publisher = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	ros::Publisher land_publisher = nh.advertise<std_msgs::Empty>("ardrone/land", 1);

	fly_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	while (state == STATE_WAIT_FOR_NAVDATA) {
		ros::spinOnce();
	}

	flat_trim_publisher.publish(std_msgs::Empty());
	fly_publisher.publish(reset_vector());

	pthread_t input_thread;
	int pthread_result = pthread_create(&input_thread, NULL, input_thread_callback, NULL);
	int pthread_mutex_result = pthread_mutex_init(&input_mutex, NULL);

	ros::Rate loop_rate(50);

	int start_take_off = 0;

	while (ros::ok()) 
	{
		if (batteryLevel <= 20) 
		{
			ROS_INFO("Battery level is too low, %f", batteryLevel);
			fly_publisher.publish(reset_vector());
			land_publisher.publish(std_msgs::Empty());
			exit(0);
		}

		pthread_mutex_lock(&input_mutex);

		switch (state)
		{
		case STATE_TAKING_OFF:
			{
				start_take_off = ros::Time::now().toSec();
				takeoff_publisher.publish(std_msgs::Empty());
				state = STATE_WAIT_FOR_TAKEOFF;
			}
			
			break;

		case STATE_WAIT_FOR_TAKEOFF:
			{
				if (ros::Time::now().toSec() - start_take_off >= 5)
					state = STATE_FIND_QR;
			}

			break;

		case STATE_LANDING:
			{
				fly_publisher.publish(reset_vector());
				land_publisher.publish(std_msgs::Empty());
				state = STATE_WAIT_FOR_LANDING;
			}

			break;

		case STATE_WAIT_FOR_LANDING:
			{
				if (altitude < 20)
					exit(0);
			}

			break;

		case STATE_FIND_QR:
			{
				fly_publisher.publish(drone_vector(0.0, 0.0, 0.5 * (altitude < 900 ? 1 : -1), 0.0, 0.0, 0.0));
			}

			break;

		case STATE_PREPARE_FIND_CIRCLE:
			{
				if (altitude < 1500)
					fly_publisher.publish(drone_vector(0.0, 0.0, 0.5, 0.0, 0.0, 0.0));
				else
				{
					fly_publisher.publish(reset_vector());
					state = STATE_FIND_CIRCLE;
				}
			}

			break;

		case STATE_FIND_CIRCLE:
			{
			}

			break;

		default:
			{
				//updatePosition();
			}

			break;
		}

		pthread_mutex_unlock(&input_mutex);

		loop_rate.sleep();
		ros::spinOnce();
	}

	void* join_status = 0;
	pthread_join(input_thread, &join_status);
	pthread_mutex_destroy(&input_mutex);

	return 0;
}