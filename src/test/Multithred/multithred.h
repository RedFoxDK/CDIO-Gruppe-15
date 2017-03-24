#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <cstdlib>

class multithred
{
	private:
		ros::NodeHandle nh;
		ros::Subscriber navdata_sub;

	public:
		multithred(ros::NodeHandle &nh);
		void Callback1(const ardrone_autonomy::Navdata::ConstPtr& msg);
		bool run();
};
