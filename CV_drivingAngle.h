#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
#include "ImageProcessing_Constants.h"
using namespace cv;
using namespace std;

class Steer {
public:
	Steer();
	void inputData(double dydxRight, double dydxLeft, double headingAngle);
	double getSteering();
private:
	int nextIdx(int pos);
	int predIdx(int pos);
	double RightAngle[MAX_SAVINGANGLE];
	double LeftAngle[MAX_SAVINGANGLE];
	double steering[MAX_SAVINGANGLE];
	int currentPos;
	int setLeftFlag = 0;
	int setRightFlag = 0;
	int setStraightLeftFlag = 0;
	int setStraightRightFlag = 0;
	double currentHeading;
};
//for drivingAngle
// 라인 검출기능과 각도 반환을 통해서 조작 가능하다
void drivingAngle(Mat& inputImg, vector<Vec4i> lines, double& steering);
//precondition: at least one lines for calculate drivingAngle
// lines for already detected by extracLines function
// postcondition : none
// return :steering Angle for driving (degree로 표현되며 정면이 0도 오른쪽이 + 왼쪽이 -임
void drivingAngle_SM(Mat& inputImg, vector<Vec4i> lines, double& steering, double& steering_Before, int Mode);
void drivingAngle_MS(Mat& inputImg, vector<Vec4i> lines, double& steering, Steer& test);
void regionOfInterest(Mat& src, Mat& dst, Point* points);//roi 지정
//precondition : point that you want to roi(3 or 4 points recommended
//postcondition : fill white without roi sector
// return : masked img dst 

bool extractLines(Mat& src, vector<Vec4i>& lines);//추출되면 1 안되면 0
//precondition: src must be color image
// return : lines는 call by ref로 반환됨
void filter_colors(Mat& src, Mat& img_filtered,Scalar& lower,Scalar& upper);
//color filtering
//precondition src: must be color image
//postcondition : 노란색과 흰색으로 구분되고 나머지는 검정으로 채워 반환된다.
//threshold 값은 imageProcessing constant에 들어있음

#endif
