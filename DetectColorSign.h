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
	//return :: 어두운정도가 percent를 넘으면 true반환.

	bool priorityStop(Mat& frame, double percent);
	//PreCondition :: percent에 붉은색이 몇퍼센트 존재해야 검출할건지 입력
	//PostCondition :: none
	//Return :: red가 percent보다 많이 검출되고 사각형이 검출되면 true


	bool isRedStop(Mat& frame, double percent);
	//PreCondition :: percent에 붉은색이 몇퍼센트 존재해야 검출할건지 입력
	//PostCondition :: none
	//Return :: red가 percent보다 많이 검출되면 true

	bool isYellow(Mat& frame, double percent);
	//PreCondition :: percent에 노란색이 몇퍼센트 존재해야 검출할건지 입력
	//PostCondition :: none
	//Return :: yellow가 percent보다 많이 검출되면 true

	int isGreenTurnSignal(Mat& frame, double percent);
	//PreCondition :: none
	//PostCondition :: none
	//Return :: 초록불 검출되지 않으면 0
	//			좌회전 신호이면 1
	//			우회전 신호이면 2 를반환한다

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