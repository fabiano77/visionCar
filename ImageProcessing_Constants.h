#ifndef IMAGEPROCESSING_CONSTANT_H
#define IMAGEPROCESSING_CONSTANT_H
#include <opencv2/opencv.hpp>
using namespace cv;

const int MAX_SAVINGANGLE=5;

const int numCornersH = 9;//수평 코너수
const int numCornersV = 9;//수직 코너수
const int numBoards = 5;//찍을 보드 갯수
const int numPoint = 4;//roi점 갯수

const int MAX_GOSTRAIGHTFLAG = 4;//직진변환시도 flag의 최댓값
const int turnAngleThreshold = 20;
const int MAX_TRYTURNFLAG = 10;//회전시도 turnflag의 최댓값
const int MAX_TURNFLAG = 10;//회전유지 turn flag의 최댓값 
#endif