<<<<<<< HEAD
#ifndef CV_CALIBRATIOM_H
#define CV_CALIBRATIOM_H
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
using namespace cv;
using namespace std;


int success = 0;//Calibration 성공 횟수
int keyForSnap = 0;//적절하게 Frame이 잡혔는지 저장

bool calibImage(VideoCapture&, Mat& intrinsicMat, Mat& distCoefMat);
//undistort함수에 사용할 intrinsicMat,distance CoefficientMat 추출 함수
//true면 성공
//false면 실패 이후 undistort함수 사용
void imgBlur(Mat& src, Mat& dst, int processingCode);
#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
Mat regionOfInterest(Mat& src, Point* points);//roi 지정
#endif

=======
#ifndef CV_CALIBRATIOM_H
#define CV_CALIBRATIOM_H
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
using namespace cv;
using namespace std;


int success = 0;//Calibration 성공 횟수
int keyForSnap = 0;//적절하게 Frame이 잡혔는지 저장

bool calibImage(VideoCapture&, Mat& intrinsicMat, Mat& distCoefMat);

>>>>>>> 5c586bc8a1d45903781e2ed57b19a8963640fd05
#endif