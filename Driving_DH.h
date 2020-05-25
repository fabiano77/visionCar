#ifndef DRIVING_DH_H
#define DRIVING_DH_H

#include"opencv2/opencv.hpp"
using namespace cv;
using namespace std;

class Driving_DH
{
public:
	Driving_DH();
	Driving_DH(bool printFlag, double sLevel);			//cam
	Driving_DH(string& filename);		//img
	Driving_DH(const char* filename);	//img
	void driving(Mat& frame, double& steerVal, int& resultLineCnt, bool rotaryFlag);	//level은 감,가속
	void mappingSetSection(double section0_, double section1_, double section2_, double section3_, double section4_, double section5_, double section6_);
	void mappingSetValue(double value0_, double value1_, double value2_, double value3_, double value4_, double value5_, double value6_);

private:
	void basicSetting();
	// 생성자마다 기본세팅

	void lineExtend(Vec4i& line, int mode, double& lineSlope);
	// 직선을 입력하면 화면전체크기의 직선으로 반환

	void imgProcess(Mat& frame, double& steerVal, int& resultLineCnt, bool rotaryFlag);
	// imgProcess를 호출하면 frame의 차선을 인식하여
	// steerVal로 주행각을 반환한다
	// 가중치가 1.0일때( 50을 기준으로 하여 0 ~ 100 ).

	double mapping(int linePoint);
	// (0 ~ frame_size.height) 의 값을 넣으면
	// (0 ~ 50)으로 정해진 setting에따라 맵핑

	Size frame_size;
	Mat frame_ROI_Line;
	Point RoiCenter;

	Mat frame_ROI;
	Mat frame_hsv;
	Mat yellowThreshold;
	Mat whiteThreshold;
	Mat frame_yellow;
	Mat frame_white;
	Mat frame_edge;

	Point lowest;
	Vec4i lowestLine;
	Vec4i rightLine;
	Vec4i leftLine;
	//double rightSlope;
	//double leftSlope;

	Vec4i resultLine;

	bool print;
	bool printResult;

	double speed;
	double steering;

	double straightLevel;

	double section[7];
	double value[7];
};

#endif	//DRIVING_DH_H