#ifndef CIRCLE_FINDER_HPP_
#define CIRCLE_FINDER_HPP_

/*
	Author: Valdemar Carøe, Andreas Andersen
*/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

class ring_circle
{
public:
	static bool find_circles(cv::Mat& image, cv::Mat gray, std::vector<ring_circle>& circles);

	double get_distance(int ring_index);

	int get_x();
	int get_y();
	int get_radius();

private:
	ring_circle(int cx, int cy, int radius);

	void draw_circle(cv::Mat& image);

private:
	cv::Point center;
	int radius;
};

#endif
