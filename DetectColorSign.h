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
	bool detectTunnel(Mat& frame, double percent);
	//return :: ��ο������� percent�� ������ true��ȯ.

	bool priorityStop(Mat& frame, double percent);
	//PreCondition :: percent�� �������� ���ۼ�Ʈ �����ؾ� �����Ұ��� �Է�
	//PostCondition :: none
	//Return :: red�� percent���� ���� ����ǰ� �簢���� ����Ǹ� true


	bool isRedStop(Mat& frame, double percent);
	//PreCondition :: percent�� �������� ���ۼ�Ʈ �����ؾ� �����Ұ��� �Է�
	//PostCondition :: none
	//Return :: red�� percent���� ���� ����Ǹ� true

	bool isYellow(Mat& frame, double percent);
	//PreCondition :: percent�� ������� ���ۼ�Ʈ �����ؾ� �����Ұ��� �Է�
	//PostCondition :: none
	//Return :: yellow�� percent���� ���� ����Ǹ� true

	int isGreenTurnSignal(Mat& frame, double percent);
	//PreCondition :: none
	//PostCondition :: none
	//Return :: �ʷϺ� ������� ������ 0
	//			��ȸ�� ��ȣ�̸� 1
	//			��ȸ�� ��ȣ�̸� 2 ����ȯ�Ѵ�

private:
	bool print;
	Scalar lower_red1;
	Scalar upper_red1;
	Scalar lower_red2;
	Scalar upper_red2;

	Scalar lower_yellow;
	Scalar upper_yellow;

	Scalar lower_green;
	Scalar upper_green;

	Mat frame_hsv;
	Mat frame_red1;
	Mat frame_red2;
	Mat store_red;
	double m_redRatio;
};

#endif	//DETECT_COLOR_SIGN_H