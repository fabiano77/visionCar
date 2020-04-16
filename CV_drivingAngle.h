#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
using namespace cv;
using namespace std;
void drivingAngle(Mat& dst, vector<Vec4i> lines, double& stiring);
Mat regionOfInterest(Mat& src, Point* points);//roi ����

bool extractLines(Mat& src, vector<Vec4i>& lines);//����Ǹ� 1 �ȵǸ� 0
//src�� �÷����¿�����
// lines�� call by ref�� ��ȯ��
void filter_colors(Mat& src, Mat& img_filtered);//color filtering
#endif
