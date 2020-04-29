#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
using namespace cv;
using namespace std;
//for drivingAngle
// 라인 검출기능과 각도 반환을 통해서 조작 가능하다
void drivingAngle(Mat& inputImg, vector<Vec4i>& lines, double& steering);
//precondition: at least one lines for calculate drivingAngle
// lines for already detected by extracLines function
// postcondition : none
// return :steering Angle for driving
Mat regionOfInterest(Mat& src, Mat& dst, Point* points);//roi 지정
//precondition : point that you want to roi(3 or 4 points recommended
//postcondition : fill white without roi sector
// return : masked img dst 

bool extractLines(Mat& src, vector<Vec4i>& lines);//추출되면 1 안되면 0
//src는 컬러상태여야함
// lines는 call by ref로 반환됨
void filter_colors(Mat& src, Mat& img_filtered);//color filtering
#endif
