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

	bool isStop(Mat& frame, double percent);
	//PreCondition   :: 멈춰있는 상태. isWhite(frame, percent)가 동작.
	//PostCondition  :: DCmoter.go 동작.
	//Return         :: isWhite가 true에서 false로 된 후 일정 frame 이상 유지되면 false
	bool isTunnel(Mat& frame, double percent);
	//PreCondition   :: DCmoter.go 동작. isBlack(frame, percent)가 동작.
	//PostCondition  :: 터널에서 차선을 검출할 함수 동작.
	//Return         :: isBlack이 true로 된 후 몇 초가 지나면 true
	int GetFlag_start();
	int GetFlag_tunnel();
private:
	bool isWhite(Mat& frame, double percent);
	bool isBlack(Mat& frame, double percent);
	int flag_start;
	int flag_tunnel;
	int check_start;
	Scalar lower_white;
	Scalar upper_white;
	Scalar lower_black;
	Scalar upper_black;
};

class RoundAbout { // 회전 교차로
public:
	RoundAbout();

	bool isStop(const double Distance);
	//PreCondition   :: 회전 교차로 정지선에서 멈춰있어야 함. 초음파 센서 활성화
	//PostCondition  :: DCmoter.go 동작.
	//Return         :: 앞 차와의 Distance가 감지된 후, 일정 거리 이상 멀어지면 false
	bool isDelay(const double Distance);
	//PreCondition   :: 회전 교차로에서 DCmoter.go 동작.
	//PostCondition  :: true이면 DCmoter.stop 동작. false이면 DCmoter.go 동작.
	//Return         :: 앞 차와의 Distance가 가까워지면 true, 멀어진 후 몇 초가 지나면 false
private:
	int flag1_start;
	int check1_start;
	double lower1_distance;
	double uper1_distance;

	int flag2_start;
	int check2_start;
	double lower2_distance;
	double uper2_distance;
};

/*
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
*/

#endif
