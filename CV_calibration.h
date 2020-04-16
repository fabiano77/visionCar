#ifndef CV_CALIBRATIOM_H
#define CV_CALIBRATIOM_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace std;


static int success = 0;//Calibration 성공 횟수
static int keyForSnap = 0;//적절하게 Frame이 잡혔는지 저장
bool calibImage(Mat& chessImg, Mat& intrinsicMat, Mat& distCoefMat);
bool calibImage(VideoCapture&, Mat& intrinsicMat, Mat& distCoefMat);
//undistort함수에 사용할 intrinsicMat,distance CoefficientMat 추출 함수
//true면 성공
//false면 실패 이후 undistort함수 사용
#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
void regionOfInterest(Mat& src,Mat& dst, Point* points);//roi 지정

#endif

#endif