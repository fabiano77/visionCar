<<<<<<< HEAD
#ifndef CV_CALIBRATIOM_H
#define CV_CALIBRATIOM_H
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
using namespace cv;
using namespace std;


int success = 0;//Calibration ���� Ƚ��
int keyForSnap = 0;//�����ϰ� Frame�� �������� ����

bool calibImage(VideoCapture&, Mat& intrinsicMat, Mat& distCoefMat);
//undistort�Լ��� ����� intrinsicMat,distance CoefficientMat ���� �Լ�
//true�� ����
//false�� ���� ���� undistort�Լ� ���
void imgBlur(Mat& src, Mat& dst, int processingCode);
#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
Mat regionOfInterest(Mat& src, Point* points);//roi ����
#endif

=======
#ifndef CV_CALIBRATIOM_H
#define CV_CALIBRATIOM_H
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
using namespace cv;
using namespace std;


int success = 0;//Calibration ���� Ƚ��
int keyForSnap = 0;//�����ϰ� Frame�� �������� ����

bool calibImage(VideoCapture&, Mat& intrinsicMat, Mat& distCoefMat);

>>>>>>> 5c586bc8a1d45903781e2ed57b19a8963640fd05
#endif