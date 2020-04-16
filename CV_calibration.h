#ifndef CV_CALIBRATIOM_H
#define CV_CALIBRATIOM_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace std;


static int success = 0;//Calibration ���� Ƚ��
static int keyForSnap = 0;//�����ϰ� Frame�� �������� ����
bool calibImage(Mat& chessImg, Mat& intrinsicMat, Mat& distCoefMat);
bool calibImage(VideoCapture&, Mat& intrinsicMat, Mat& distCoefMat);
//undistort�Լ��� ����� intrinsicMat,distance CoefficientMat ���� �Լ�
//true�� ����
//false�� ���� ���� undistort�Լ� ���
#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
void regionOfInterest(Mat& src,Mat& dst, Point* points);//roi ����

#endif

#endif