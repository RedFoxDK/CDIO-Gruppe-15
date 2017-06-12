#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <math.h>



using namespace cv;
using namespace std;

static string window_name = "view";

Point find_circel(Mat img) {
  Mat gray;
  Point center;

  cvtColor(img, gray, CV_BGR2GRAY);
  // smooth it, otherwise a lot of false circles may be detected
  GaussianBlur( gray, gray, Size(9, 9), 2, 2 );
  vector<Vec3f> circles;
  HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray.rows/4, 30, 300 );
  for( size_t i = 0; i < circles.size(); i++ )
  {
    center = Point(cvRound(circles[i][0]), cvRound(circles[i][1]));    
    int radius = cvRound(circles[i][2]);
    // draw the circle center
    circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
    // draw the circle outline
    circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
  }
  imshow(window_name, img);
  return center;
}

using namespace cv_bridge;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Point center = find_circel(cv_bridge::toCvShare(msg, "bgr8")->image);
    ROS_INFO("X coord: %f, Y coord: %f", center.x, center.y);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "circel_finder");
  ros::NodeHandle nh;
  cv::namedWindow(window_name);
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);  
  ros::spin();
  cv::destroyWindow(window_name);
}
