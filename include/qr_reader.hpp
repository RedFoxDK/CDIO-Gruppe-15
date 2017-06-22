#ifndef QR_READER_HPP_
#define QR_READER_HPP_

/*
	Author: Valdemar Carøe, Andreas Andersen
*/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

enum MOVE_DIRECTION : unsigned char 
{
	DIRECTION_UNKNOWN = 0,
	DIRECTION_NONE,
	DIRECTION_LEFT,
	DIRECTION_RIGHT
};

class qr_code
{
public:
	static bool find_qr_codes(cv::Mat& image, uint8_t* raw, int width, int height, std::vector<qr_code>& qr_codes);

	cv::Point get_center();
	MOVE_DIRECTION get_direction();

	std::string get_data();

private:
	qr_code(std::string const& data);

	void add_point(int x, int y);
	void print_points();

	void draw_square(cv::Mat& image);

	float distance(cv::Point& p1, cv::Point& p2);

private:
	std::string data;
	std::vector<cv::Point> points;
};


#endif
