#ifndef CV_CALIBRATIOM_H
#define CV_CALIBRATIOM_H
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
using namespace cv;
using namespace std;


int success = 0;//Calibration ���� Ƚ��
int keyForSnap = 0;//�����ϰ� Frame�� �������� ����

bool calibImage(VideoCapture&, Mat& intrinsicMat, Mat& distCoefMat);

#endif