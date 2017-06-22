#include "qr_reader.hpp"

#include <zbar.h>

#include <iostream>

/*
	Author: Valdemar Carøe, Andreas Andersen
*/

bool qr_code::find_qr_codes(cv::Mat& image, uint8_t* raw, int width, int height, std::vector<qr_code>& qr_codes)
{
  zbar::ImageScanner scanner;  
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

  zbar::Image qr_image(width, height, "Y800", raw, width * height);
   
  int symbol_count = scanner.scan(qr_image);  
  
  for (zbar::Image::SymbolIterator symbol = qr_image.symbol_begin(); symbol != qr_image.symbol_end(); ++symbol) 
  { 
    qr_code code(symbol->get_data());

    for (int i = 0, size = symbol->get_location_size(); i < size; i++)
      code.add_point(symbol->get_location_x(i), symbol->get_location_y(i));

    code.draw_square(image);
    
    qr_codes.push_back(code);
  }  

  return (symbol_count > 0);
}

cv::Point qr_code::get_center()
{
  int cx = ((this->points[0].x + this->points[1].x + this->points[2].x + this->points[3].x) / 4);
  int cy = ((this->points[0].y + this->points[1].y + this->points[2].y + this->points[3].y) / 4);
  
  return cv::Point(cx, cy);
}

MOVE_DIRECTION qr_code::get_direction()
{
  if (this->points.size() == 4)
  {
    float l1 = this->distance(this->points[0], this->points[1]);
    float l2 = this->distance(this->points[2], this->points[3]);

    float delta = l1 - l2;

    if (delta >= 1)
      return DIRECTION_RIGHT;
    else if (delta <= -1)
      return DIRECTION_LEFT;

    return DIRECTION_NONE;
  }

  return DIRECTION_UNKNOWN;
}

std::string qr_code::get_data()
{
  return this->data;
}

qr_code::qr_code(std::string const& data)
  : data(data)
{

}

void qr_code::add_point(int x, int y)
{
  this->points.push_back(cv::Point(x, y));
}

void qr_code::print_points()
{
  printf("(%d, %d), (%d, %d), (%d, %d), (%d, %d)\n", 
    this->points[0].x, this->points[0].y, 
    this->points[1].x, this->points[1].y, 
    this->points[2].x, this->points[2].y, 
    this->points[3].x, this->points[3].y);
}

void qr_code::draw_square(cv::Mat& image)
{
  if (this->points.size() == 4)
  {
    for (int i = 0; i < 4; i++)
      line(image, this->points[i], this->points[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);
  }
}

float qr_code::distance(cv::Point& p1, cv::Point& p2)
{
  return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}
