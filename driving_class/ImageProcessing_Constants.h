#ifndef IMAGEPROCESSING_CONSTANT_H
#define IMAGEPROCESSING_CONSTANT_H
#include <opencv2/opencv.hpp>
using namespace cv;

const int MAX_SAVINGANGLE=5;

const int numCornersH = 9;//���� �ڳʼ�
const int numCornersV = 9;//���� �ڳʼ�
const int numBoards = 5;//���� ���� ����
const int numPoint = 4;//roi�� ����

const int MAX_GOSTRAIGHTFLAG = 4;//������ȯ�õ� flag�� �ִ�
const int turnAngleThreshold = 20;
const int MAX_TRYTURNFLAG = 10;//ȸ���õ� turnflag�� �ִ�
const int MAX_TURNFLAG = 10;//ȸ������ turn flag�� �ִ� 
#endif