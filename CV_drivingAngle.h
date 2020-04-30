#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
#include "ImageProcessing_Constants.h"
using namespace cv;
using namespace std;
//for drivingAngle
// ���� �����ɰ� ���� ��ȯ�� ���ؼ� ���� �����ϴ�
void drivingAngle(Mat& inputImg, vector<Vec4i>& lines, double& steering);
//precondition: at least one lines for calculate drivingAngle
// lines for already detected by extracLines function
// postcondition : none
// return :steering Angle for driving (degree�� ǥ���Ǹ� ������ 0�� �������� + ������ -��
Mat regionOfInterest(Mat& src, Mat& dst, Point* points);//roi ����
//precondition : point that you want to roi(3 or 4 points recommended
//postcondition : fill white without roi sector
// return : masked img dst 

bool extractLines(Mat& src, vector<Vec4i>& lines);//����Ǹ� 1 �ȵǸ� 0
//precondition: src must be color image
// return : lines�� call by ref�� ��ȯ��
void filter_colors(Mat& src, Mat& img_filtered);
//color filtering
//precondition src: must be color image
//postcondition : ������� ������� ���еǰ� �������� �������� ä�� ��ȯ�ȴ�.
//threshold ���� imageProcessing constant�� �������
#endif
