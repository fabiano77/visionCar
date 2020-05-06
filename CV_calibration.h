#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <assert.h>
#include "ImageProcessing_Constants.h"

using namespace std;
using namespace cv;

class Calibration {
public:
	Calibration();
	//���� �� ���� ���� �Լ�
	//postcondition: insertImage�� �̹��� �� ��ŭ �Է����־�� returnCalibInfo��� ����
	//�̹������� ImageProcessing_constant�� �ԷµǾ�����
	Calibration(Mat& inputIchessImage, int numImage);
	//���� ���ϴ� ����ŭ chessImage�� �Է��Ͽ� ���尡��
	//precondition : �̹��� ���ڵ� ���� �־��־����
	//postcondition : �ٷ� insertImage��밡��
	void insertImage(Mat& inputChessImage);
	//procondition:none
	//postcondition: �Է��� �Ϸ�Ǿ��ٰ� �� ��� returncalibration���డ��
	bool returnCalibInfo(Mat& intrinsicMat, Mat& distanceCoeffsMat);
	//precondition : Mat&,int�� �Ķ���ͷ� ���� ������ ����ϰų� insertImage�Լ��� �ִ븸ŭ �θ��� ��밡��
	//postcondition : nothing
	//return : intrinsic, distancecoefficient �� ��ȯ calibration������ true,���н� false ����

private:
	int flag = 0;
	int numImage;
	int idx = 0;//���Խ� index, numImage���� �ִ� 1�۾ƾ� �Ѵ�.
	bool calibImage();
	Mat* primaryImage = nullptr;
	Mat* intrinsic=nullptr;
	Mat* distcoeffs=nullptr;
	int width = 0;
	int height = 0;
	Size imgSize;
};
bool calibImage(VideoCapture& inVideo, Mat& intrinsicMat, Mat& distCoefMat);
#endif