#ifndef IMAGEPROCESSING_CONSTANT_H
#define IMAGEPROCESSING_CONSTANT_H
#include <opencv2/opencv.hpp>
using namespace cv;

const int MAX_SAVINGANGLE=5;
Scalar lower_w = Scalar(120, 120, 120); //��� ���� (RGB)
Scalar upper_w = Scalar(255, 255, 255);
Scalar lower_y = Scalar(10, 100, 100); //����� ���� (HSV)
Scalar upper_y = Scalar(40, 255, 255);

const int numCornersH = 9;//���� �ڳʼ�
const int numCornersV = 9;//���� �ڳʼ�
const int numBoards = 5;//���� ���� ����
const int numPoint = 4;//roi�� ����
int steeringFlag=0;//ȸ����Ȳ �Ǵܿ� flag
#endif