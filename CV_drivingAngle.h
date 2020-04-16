#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
using namespace cv;
using namespace std;
void drivingAngle(Mat& dst, vector<Vec4i> lines, double& stiring);

#endif
