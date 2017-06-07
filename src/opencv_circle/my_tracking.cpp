#include "opencv2/opencv.hpp"

using namespace cv;

void on_H_thresh_trackbar(int, void *) {};
void on_S_thresh_trackbar(int, void *) {};
void on_V_thresh_trackbar(int, void *) {};

int H_low = 165, S_low = 127, V_low = 42; 
int H_top = 260, S_top = 260, V_top = 260;
int H_max = 260, S_max = 260, V_max = 260;

int main( int argc, char** argv ) {
   Mat frame, HSVimage, HSVdone;
   vector<Mat> channels;

//   namedWindow("RBG",1);
   namedWindow("HSV",1);
   namedWindow("H",1);
   namedWindow("S",1);
   namedWindow("V",1);

   /*VideoCapture cap(0); // open the default camera
   if(!cap.isOpened()) {  // check if we succeeded
   	return -1;
   }*/
   for (;;) {
   	//cap >> frame;
   	frame = imread(argv[1], 1);
	cvtColor(frame, HSVimage, CV_BGR2HSV);

	split(HSVimage, channels);
	
	createTrackbar("H Low", "H", &H_low, H_top, on_H_thresh_trackbar);
	createTrackbar("H Max", "H", &H_max, H_top, on_H_thresh_trackbar);
	createTrackbar("S Low", "S", &S_low, S_top, on_S_thresh_trackbar);
	createTrackbar("S Max", "S", &S_max, S_top, on_S_thresh_trackbar);
	createTrackbar("V Low", "V", &V_low, V_top, on_V_thresh_trackbar);
	createTrackbar("V Max", "V", &V_max, V_top, on_V_thresh_trackbar);

	inRange(channels[0], Scalar(H_low), Scalar(H_max), channels[0]);
	inRange(channels[1], Scalar(S_low), Scalar(S_max), channels[1]);
	inRange(channels[2], Scalar(V_low), Scalar(V_max), channels[2]);

	inRange(HSVimage, Scalar(H_low, S_low, V_low, 0), Scalar(H_max, S_max, V_max, 0), HSVimage);

   	//imshow("RBG", HSVdone);
	imshow("HSV", HSVimage);
	//imshow("H", channels[0]);
	//imshow("S", channels[1]);
	//imshow("V", channels[2]);

	if(waitKey(30) >= 0) break;
   }
   return 0;
}
