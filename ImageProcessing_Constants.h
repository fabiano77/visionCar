#ifndef IMAGEPROCESSING_CONSTANT_H
#define IMAGEPROCESSING_CONSTANT_H
#include<opencv2/opencv.hpp>
using namespace cv;

Scalar lower_white = Scalar(120, 120, 120); //��� ���� (RGB)
Scalar upper_white = Scalar(255, 255, 255);
Scalar lower_yellow = Scalar(10, 100, 100); //����� ���� (HSV)
Scalar upper_yellow = Scalar(40, 255, 255);

const int numCornersH = 9;//���� �ڳʼ�
const int numCornersV = 9;//���� �ڳʼ�
const int numBoards = 5;//���� ���� ����
int numSquares = numCornersH * numCornersV;
const int numPoint = 4;//roi�� ����

#endif