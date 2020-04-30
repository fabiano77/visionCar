#ifndef DRIVING_DH_H
#define DRIVING_DH_H

#include<iostream>
#include"opencv2/opencv.hpp"
using namespace cv;
using namespace std;

class Driving_DH
{
public:
	Driving_DH();
	Driving_DH(bool printFlag, double cLevel, double sLevel);					//cam
	Driving_DH(const char* filename);	//img
	Driving_DH(string& filename);	//img
	void driving(Mat& frame, double& steerVal, double& speedVal, double basicSpd, double level);	//level은 감,가속

private:
	void lineExtend(Vec4i& line, int mode);
	void imgProcess(Mat& frame, double& steerVal);
	// imgProcess를 호출하면 frame의 차선을 인식하여
	// steerVal로 주행각을 반환한다
	// 가중치가 1.0일때( 50을 기준으로 하여 0 ~ 100 ).

	Size frame_size;
	Mat frame_ROI_Line;
	Point RoiCenter;
	bool print;
	bool driveFlag;
	double speed;
	double steering;
	double cornerLevel;
	double straightLevel;
};

#endif	//DRIVING_DH_H