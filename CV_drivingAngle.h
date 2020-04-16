<<<<<<< HEAD
#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
using namespace cv;
using namespace std;
void drivingAngle(Mat& dst, vector<Vec4i> lines, double& stiring);
Mat regionOfInterest(Mat& src, Point* points);//roi ÁöÁ¤
#endif
=======
#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
using namespace cv;
using namespace std;
void drivingAngle(Mat& dst, vector<Vec4i> lines, double& stiring);

#endif
>>>>>>> 5c586bc8a1d45903781e2ed57b19a8963640fd05
