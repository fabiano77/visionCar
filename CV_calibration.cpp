#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <assert.h>
#include "ImageProcessing_Constants.h"
#include "CV_Calibration.h"
using namespace std;
using namespace cv;
Calibration::Calibration() {
	imgSize = Size(0, 0);
	primaryImage = new Mat[numBoards];
	numImage = numBoards;
	intrinsic = new Mat(3, 3, CV_32FC1);
}

Calibration::Calibration(Mat& inputChessImage, int numImage) {
	this->numImage = numImage;
	primaryImage = new Mat[this->numImage];
	width = primaryImage[0].size().width;
	height = primaryImage[0].size().height;
	imgSize = Size(width, height);
}

void Calibration::insertImage(Mat& inputChessImage) {
	inputChessImage.copyTo(primaryImage[idx]);
	assert(idx + 1 > numImage);
	if (idx == 0) {
		width = primaryImage[0].size().width;
		height = primaryImage[0].size().height;
		imgSize = Size(width, height);
	}
	idx++;
	if (idx == numImage) { cout << "������ ��� �Է� �Ϸ�Ǿ����ϴ�" << endl; }
}

bool Calibration::calibImage() {
	//from 2d to 3d convert
	vector<vector<Point3f>> objPoints;
	vector<vector<Point2f>> imgPoints;
	vector<Point3f> obj;
	vector<Point2f> corner;//ã�� �ڳ��� ������ġ

	//from video to Mat
	Mat* grayImg = new Mat[numImage];//�̹��� ���ڸ�ŭ ��ȯ�� ���̹Ƿ� ����
	//for convert to gray scale

	Mat distCoeffs;//�Ÿ� ���
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	flag = 0;

	for (int j = 0; j < numSquares; j++) {
		obj.push_back(Point3f(j / numCornersH, j % numCornersH, 0.0f));
	}
	for (int i = 0; i < numImage; i++) {
		cvtColor(primaryImage[i], grayImg[i], COLOR_BGR2GRAY);
		bool found = findChessboardCorners(grayImg[i], boardSize, corner, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
		if (found) {//cornerã����

			//�׷��� �˾ƺ��� �ɶ� ���
			//drawChessboardCorners(grayImg[i], boardSize, corner, found);

			cout << "found Corners" << endl;
			imgPoints.push_back(corner);
			objPoints.push_back(obj);
		}
		else {
			flag++;
			cout << flag << "��° �̹����� calibration ���� ã�� ���߽��ϴ�" << endl;
		}
		(*intrinsic).ptr<float>(0)[0] = 1;
		(*intrinsic).ptr<float>(1)[1] = 1;
		calibrateCamera(objPoints, imgPoints, imgSize, *intrinsic, distCoeffs, rvecs, tvecs);
	}
	if (flag == numImage) { return false; }//��� calibration���� ���� ���Ѱ�� fail
	//Calib�� ����
	return true;
}

bool Calibration::returnCalibInfo(Mat& intrinsicMat, Mat& distanceCoeffsMat) {
	calibImage();
	if (flag == numImage) { return false; }
	else {
		intrinsicMat = *intrinsic;
		distanceCoeffsMat = *distcoeffs;
		return true;
	}
}
