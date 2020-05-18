#ifndef CV_DRIVINGANGLE_H
#define CV_DRIVINGANGLE_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
#include "ImageProcessing_Constants.h"
using namespace cv;
using namespace std;

class CheckStart {
public:
	CheckStart();
	bool isWhite(Mat& frame, double percent);
	//PreCondition   :: 흰색이 몇퍼센트 이상 존재해야 할 지에 대해 percent로 입력
	//PostCondition  :: none
	//Return         :: White가 percent보다 많이 검출되면 true
	bool isBlack(Mat& frame, double percent);
	//PreCondition   :: 검은색이 몇퍼센트 이상 존재해야 할 지에 대해 percent로 입력
	//PostCondition  :: none
	//Return         :: Black이 percent보다 많이 검출되면 true
	bool isStart(Mat& frame, double percent);
	//PreCondition   :: isWhite(frame, percent)가 동작
	//PostCondition  :: DCmoter.go 동작함. Go!라는 문자가 화면에 출력됨
	//Return         :: isWhite가 true에서 false로 된 후 일정 frame 이상 유지되면 true
	bool isTunnel(Mat& frame, double percent);
	//PreCondition   :: isBlack(frame, percent)가 동작
	//PostCondition  :: 터널에서 차선을 검출할 함수 동작함. Tunnel!라는 문자가 화면에 출력됨
	//Return         :: isBlack이 true로 된 후 일정 frame 이상 유지되면 true
	void GetFlag_start();
	void GetFlag_tunnel();
private:
	int flag_start;
	int flag_tunnel;
	Scalar lower_white;
	Scalar upper_white;
	Scalar lower_black;
	Scalar upper_black;
	Mat frame_hsv;
};

//for drivingAngle
// 라인 검출기능과 각도 반환을 통해서 조작 가능하다
void drivingAngle_SM(Mat& inputImg, vector<Vec4i> lines, double& steering, double& steering_Before, int& flag);

void regionOfInterest(Mat& src, Mat& dst, Point* points);//roi 지정
//precondition : point that you want to roi(3 or 4 points recommended
//postcondition : fill white without roi sector
// return : masked img dst 

bool extractLines(Mat& src, vector<Vec4i>& lines);//추출되면 1 안되면 0
//precondition: src must be color image
// return : lines는 call by ref로 반환됨
void filter_colors(Mat& src, Mat& img_filtered);
//color filtering
//precondition src: must be color image
//postcondition : 노란색과 흰색으로 구분되고 나머지는 검정으로 채워 반환된다.
//threshold 값은 imageProcessing constant에 들어있음
void imgProcessing(Mat& src, Mat& dst, int processingCode);
#endif
