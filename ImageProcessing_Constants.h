#ifndef IMAGEPROCESSING_CONSTANT_H
#define IMAGEPROCESSING_CONSTANT_H
#include <opencv2/opencv.hpp>
using namespace cv;

const int MAX_SAVINGANGLE=5;
Scalar lower_w = Scalar(120, 120, 120); //흰색 차선 (RGB)
Scalar upper_w = Scalar(255, 255, 255);
Scalar lower_y = Scalar(10, 100, 100); //노란색 차선 (HSV)
Scalar upper_y = Scalar(40, 255, 255);

const int numCornersH = 9;//수평 코너수
const int numCornersV = 9;//수직 코너수
const int numBoards = 5;//찍을 보드 갯수
const int numPoint = 4;//roi점 갯수
int steeringFlag=0;//회전상황 판단용 flag
#endif