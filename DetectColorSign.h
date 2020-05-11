#ifndef DETECT_COLOR_SIGN_H
#define DETECT_COLOR_SIGN_H

#include"opencv2/opencv.hpp"
using namespace cv;
using namespace std;

class DetectColorSign
{
public:
	DetectColorSign();
	DetectColorSign(bool onPrint);

	bool isRedStop(Mat& frame, int percent);
	//PreCondition :: percent�� �������� ���ۼ�Ʈ �����ؾ� �����Ұ��� �Է�
	//PostCondition :: none
	//Return :: red�� percent���� ���� ����Ǹ� true

private:
	bool print;
	Scalar lower_red1;
	Scalar upper_red1;
	Scalar lower_red2;
	Scalar upper_red2;

	Mat frame_hsv;
	Mat frame_red;
	Mat frame_red1;
	Mat frame_red2;
};

#endif	//DETECT_COLOR_SIGN_H