#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
using namespace cv;
using namespace std;
void drivingAngle(Mat& dst, vector<Vec4i> lines, double& stiring);
Mat regionOfInterest(Mat& src, Point* points);//roi 지정

bool extractLines(Mat& src, vector<Vec4i>& lines);//추출되면 1 안되면 0
//src는 컬러상태여야함
// lines는 call by ref로 반환됨
void filter_colors(Mat& src, Mat& img_filtered);//color filtering
#endif
