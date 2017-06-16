#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <zbar.h>
#include "std_srvs/Empty.h"
#include <CDIO/send_qr.h>

using namespace cv;
using namespace cv_bridge;

#include "qr_reader.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
  	std::vector<qr_code> qr_codes;
  	
    if (find_qr_code(cv_bridge::toCvShare(msg, "rgb8")->image, qr_codes))
    {
    	for (std::size_t i = 0; i < qr_codes.size(); i++)
    		printf("Found QR code: %s -> %d.\n", 
    			qr_codes.at(i).get_data().c_str(), 
    			qr_codes.at(i).get_direction());

    	printf("\n");
    }

    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qr_reader");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);  
  //qr_pub = nh.advertise<CDIO::send_qr>("CDIO/qr_finder");
  
  ros::spin();
}