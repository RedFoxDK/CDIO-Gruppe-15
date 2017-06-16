#include "qr_reader.hpp"

#include <zbar.h>

#include <iostream>

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
  }

  return DIRECTION_UNKNOWN;
}

std::string qr_code::get_data()
{
  return this->data;
}

float qr_code::distance(cv::Point& p1, cv::Point& p2)
{
  return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

/* global functions */
bool find_qr_code(cv::Mat image, std::vector<qr_code>& qr_codes)
{
  if (image.empty())
  {
    std::cerr << "Error: Unable to query image from capture device.\n" << std::endl;
    return false;
  }

  cv::Mat gray(image.size(), CV_MAKETYPE(image.depth(), 1));  
  cv::cvtColor(image, gray, CV_RGB2GRAY);
  
  zbar::ImageScanner scanner;  
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

  int width = image.cols;  
  int height = image.rows; 

  uint8_t* raw = reinterpret_cast<uint8_t*>(gray.data); 

  zbar::Image qr_image(width, height, "Y800", raw, width * height);
   
  int symbol_count = scanner.scan(qr_image);  
  
  if (symbol_count > 0)
  {
    for(zbar::Image::SymbolIterator symbol = qr_image.symbol_begin(); symbol != qr_image.symbol_end(); ++symbol) 
    { 
      qr_code code(symbol->get_data());

      for (int i = 0, size = symbol->get_location_size(); i < size; i++)
        code.add_point(symbol->get_location_x(i), symbol->get_location_y(i));

      code.draw_square(image);
      qr_codes.push_back(code);
    }  

    imshow ("Image", image);
    return true;
  }

  return false;
}