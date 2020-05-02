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
	//onPrint는 판단 과정을 출력할 것인가.

	bool isRedStop(Mat& frame, int percent);
	//PreCondition :: percent에 붉은색이 몇퍼센트 존재해야 검출할건지 입력
	//PostCondition :: none
	//Return :: red가 percent보다 많이 검출되면 true

private:
	bool print;
	Scalar lower_red1;
	Scalar upper_red1;
	Scalar lower_red2;
	Scalar upper_red2;
};

#endif	//DETECT_COLOR_SIGN_H