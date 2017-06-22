#include "circle_finder.hpp"

/*
	Author: Martin L. Djurhuus
*/

bool ring_circle::find_circles(cv::Mat& image, cv::Mat gray, std::vector<ring_circle>& circles)
{
	cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

	std::vector<cv::Vec3f> circle_vectors;
	cv::HoughCircles(gray, circle_vectors, CV_HOUGH_GRADIENT, 2, gray.rows / 4, 30, 300);

	for (std::size_t i = 0; i < circle_vectors.size(); i++)
	{
		ring_circle ring(cvRound(circle_vectors[i][0]), cvRound(circle_vectors[i][1]), cvRound(circle_vectors[i][2]));

		ring.draw_circle(image);

		circles.push_back(ring);
	}

	return (circle_vectors.size() > 0);
}

double ring_circle::get_distance(int ring_index)
{
	static int ring_diameters[] = { 100, 100, 90, 90, 80, 80 };

	double ring_radius = ring_diameters[ring_index] / 2.0;
	double focus_length = 503.24;

 	return ((ring_radius * focus_length) / this->radius);
}

int ring_circle::get_x()
{
	return this->center.x;
}

int ring_circle::get_y()
{
	return this->center.y;
}

int ring_circle::get_radius()
{
	return this->radius;
}

ring_circle::ring_circle(int cx, int cy, int radius)
	: center(cx, cy), radius(radius)
{

}

void ring_circle::draw_circle(cv::Mat& image)
{
	if (this->radius > 0)
	{
		/* Draw the circle's center */
		cv::circle(image, this->center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);

		/* Draw the circle's outline */
		cv::circle(image, this->center, this->radius, cv::Scalar(0, 0, 255), 3, 8, 0);
	}
}
