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
	//생성 후 삽입 예정 함수
	//postcondition: insertImage를 이미지 수 만큼 입력해주어야 returnCalibInfo사용 가능
	//이미지수는 ImageProcessing_constant에 입력되어있음
	Calibration(Mat& inputIchessImage, int numImage);
	//내가 원하는 수만큼 chessImage를 입력하여 저장가능
	//precondition : 이미지 숫자도 같이 넣어주어야함
	//postcondition : 바로 insertImage사용가능
	void insertImage(Mat& inputChessImage);
	//procondition:none
	//postcondition: 입력이 완료되었다고 뜬 경우 returncalibration진행가능
	bool returnCalibInfo(Mat& intrinsicMat, Mat& distanceCoeffsMat);
	//precondition : Mat&,int를 파라미터로 갖는 생성자 사용하거나 insertImage함수를 최대만큼 부르고 사용가능
	//postcondition : nothing
	//return : intrinsic, distancecoefficient 값 반환 calibration성공시 true,실패시 false 리턴

private:
	int flag = 0;
	int numImage;
	int idx = 0;//삽입시 index, numImage보다 최대 1작아야 한다.
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