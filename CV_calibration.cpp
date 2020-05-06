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
	if (idx == numImage) { cout << "사진이 모두 입력 완료되었습니다" << endl; }
}

bool Calibration::calibImage() {
	//from 2d to 3d convert
	vector<vector<Point3f>> objPoints;
	vector<vector<Point2f>> imgPoints;
	vector<Point3f> obj;
	vector<Point2f> corner;//찾은 코너의 저장위치

	//from video to Mat
	Mat* grayImg = new Mat[numImage];//이미지 숫자만큼 변환할 것이므로 생성
	//for convert to gray scale

	Mat distCoeffs;//거리 계수
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	flag = 0;

	for (int j = 0; j < numSquares; j++) {
		obj.push_back(Point3f(j / numCornersH, j % numCornersH, 0.0f));
	}
	for (int i = 0; i < numImage; i++) {
		cvtColor(primaryImage[i], grayImg[i], COLOR_BGR2GRAY);
		bool found = findChessboardCorners(grayImg[i], boardSize, corner, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
		if (found) {//corner찾으면

			//그려서 알아봐야 될때 사용
			//drawChessboardCorners(grayImg[i], boardSize, corner, found);

			cout << "found Corners" << endl;
			imgPoints.push_back(corner);
			objPoints.push_back(obj);
		}
		else {
			flag++;
			cout << flag << "번째 이미지는 calibration 점을 찾지 못했습니다" << endl;
		}
		(*intrinsic).ptr<float>(0)[0] = 1;
		(*intrinsic).ptr<float>(1)[1] = 1;
		calibrateCamera(objPoints, imgPoints, imgSize, *intrinsic, distCoeffs, rvecs, tvecs);
	}
	if (flag == numImage) { return false; }//모두 calibration점을 잡지 못한경우 fail
	//Calib값 추출
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


bool calibImage(VideoCapture& inVideo, Mat& intrinsicMat, Mat& distCoefMat) { // 흑백 입력 필수
	if (!(inVideo.isOpened())) {
		cout << "영상 입력되지 않았습니다." << endl;
		return false;
	}
	//수정가능한 값
	const int numCornersH = 9;//수평 코너수
	const int numCornersV = 9;//수직 코너수
	const int numBoards = 5;//찍을 보드 갯수

	int numSquares = numCornersH * numCornersV;
	Size boardSize(numCornersH, numCornersV);

	//from 2d to 3d convert
	vector<vector<Point3f>> objPoints;
	vector<vector<Point2f>> imgPoints;
	vector<Point3f> obj;
	vector<Point2f> corner;//찾은 코너의 저장위치

	Mat img;//from video to Mat
	Mat grayImg;//convert to gray scale
	Mat intrinsic(3, 3, CV_32FC1);
	Mat distCoeffs;//거리 계수
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	for (int j = 0; j < numSquares; j++) {
		obj.push_back(Point3f(j / numCornersH, j % numCornersH, 0.0f));
	}
	while (success < numBoards) {
		inVideo >> img;
		cvtColor(img, grayImg, COLOR_BGR2GRAY);
		imshow("frame", grayImg);
		bool found = findChessboardCorners(grayImg, boardSize, corner, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found) {//corner찾으면
			cornerSubPix(grayImg, corner, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(grayImg, boardSize, corner, found);
			keyForSnap++;
			if (keyForSnap == 5) {
				cout << "found" << endl;
				imgPoints.push_back(corner);
				objPoints.push_back(obj);
				success++;
				cout << "numBoards:" << numBoards << " success: " << success << endl;
				waitKey(1000);//1초 정도 완료되면 자리 재정비 시간을 가짐
				keyForSnap = 0;
			}
		}
		if (success >= numBoards) {
			//calib진
			break;
		}
		waitKey(10);
	}
	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;
	calibrateCamera(objPoints, imgPoints, img.size(), intrinsic, distCoeffs, rvecs, tvecs);
	//Calib값 추출
	intrinsicMat = intrinsic;
	distCoefMat = distCoeffs;
	return true;
}