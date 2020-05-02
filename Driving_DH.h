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
	void driving(Mat& frame, double& steerVal, double& speedVal, double basicSpd, double level);	//level�� ��,����
	void mappingSetSection(double section0_, double section1_, double section2_, double section3_, double section4_, double section5_);
	void mappingSetValue(double value0_, double value1_, double value2_, double value3_, double value4_, double value5_);

private:
	void basicSetting();
	// �����ڸ��� �⺻����

	void lineExtend(Vec4i& line, int mode);
	// ������ �Է��ϸ� ȭ����üũ���� �������� ��ȯ

	void imgProcess(Mat& frame, double& steerVal);
	// imgProcess�� ȣ���ϸ� frame�� ������ �ν��Ͽ�
	// steerVal�� ���ఢ�� ��ȯ�Ѵ�
	// ����ġ�� 1.0�϶�( 50�� �������� �Ͽ� 0 ~ 100 ).

	double mapping(int linePoint);
	// (0 ~ frame_size.height) �� ���� ������
	// (0 ~ 50)���� ������ setting������ ����

	Size frame_size;
	Mat frame_ROI_Line;
	Point RoiCenter;

	bool print;
	bool printResult;

	double speed;
	double steering;

	double straightLevel;

	double section[6];
	double value[6];
};

#endif	//DRIVING_DH_H