#include "opencv2/opencv.hpp"

using namespace cv;

void on_H_thresh_trackbar(int, void *) {};
void on_S_thresh_trackbar(int, void *) {};
void on_V_thresh_trackbar(int, void *) {};

int H_low = 58, S_low = 0, V_low = 13; 
int H_top = 260, S_top = 260, V_top = 260;
int H_max = 84, S_max = 40, V_max = 86;

int main( int argc, char** argv ) {
   Mat frame, HSVimage, HSVdone;
   vector<Mat> channels;

   namedWindow("RGB",1);
   namedWindow("First",1);
   namedWindow("R",1);
   namedWindow("G",1);
   namedWindow("B",1);

   /*VideoCapture cap(0); // open the default camera
   if(!cap.isOpened()) {  // check if we succeeded
   	return -1;
   }*/
   for (;;) {
   	//cap >> frame;
   	frame = imread(argv[1], 1);
	//cvtColor(frame, HSVimage, CV_BGR2HSV);
	cvtColor(frame, HSVdone, CV_BGR2RGB);

	split(HSVdone, channels);
	
	createTrackbar("R Low", "R", &H_low, H_top, on_H_thresh_trackbar);
	createTrackbar("R Max", "R", &H_max, H_top, on_H_thresh_trackbar);
	createTrackbar("G Low", "G", &S_low, S_top, on_S_thresh_trackbar);
	createTrackbar("G Max", "G", &S_max, S_top, on_S_thresh_trackbar);
	createTrackbar("B Low", "B", &V_low, V_top, on_V_thresh_trackbar);
	createTrackbar("B Max", "B", &V_max, V_top, on_V_thresh_trackbar);

	inRange(channels[0], Scalar(H_low), Scalar(H_max), channels[0]);
	inRange(channels[1], Scalar(S_low), Scalar(S_max), channels[1]);
	inRange(channels[2], Scalar(V_low), Scalar(V_max), channels[2]);

	inRange(HSVdone, Scalar(H_low, S_low, V_low, 0), Scalar(H_max, S_max, V_max, 0), HSVdone);

   	imshow("RGB", HSVdone);
	imshow("First", frame);
	imshow("R", channels[0]);
	imshow("G", channels[1]);
	imshow("B", channels[2]);

	if(waitKey(30) >= 0) break;
   }
   return 0;
}
