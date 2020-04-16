#ifndef CV_CALIBRATIOM_H
#define CV_CALIBRATIOM_H
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
using namespace cv;
using namespace std;


int success = 0;//Calibration 성공 횟수
int keyForSnap = 0;//적절하게 Frame이 잡혔는지 저장

bool calibImage(VideoCapture&, Mat& intrinsicMat, Mat& distCoefMat);

#endif