#include <cv.h>
#include "opencv2/opencv.hpp"
#include <highgui.h>
#include <math.h>

using namespace cv;

int main(int argc, char** argv)
{
    Mat img, gray;
   // if( argc != 2 && !(img=imread(argv[1], 1)).data)
     //   return -1;

    //img = imread(argv[1], 1);

    VideoCapture cap(0);
    
    namedWindow( "circles", 1 );

    for(;;)
    {
	    cap >> img;
	
	    cvtColor(img, gray, CV_BGR2GRAY);
	    // smooth it, otherwise a lot of false circles may be detected
	    GaussianBlur( gray, gray, Size(9, 9), 2, 2 );
	    vector<Vec3f> circles;
	    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray.rows/4, 50, 300 );
	    for( size_t i = 0; i < circles.size(); i++ )
	    {
		 Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		 int radius = cvRound(circles[i][2]);
		 // draw the circle center
		 circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
		 // draw the circle outline
		 circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
	    }
	    


    
    	imshow( "circles", img );
	if(waitKey(30) >= 0) break;


    }
    return 0;
}
