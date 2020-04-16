#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "CV_calibration.h"
using namespace std;
using namespace cv;
bool calibImage(Mat& chessImg, Mat& intrinsicMat, Mat& distCoefMat) { // ��� �Է� �ʼ�
	
	//���������� ��
	const int numCornersH = 9;//���� �ڳʼ�
	const int numCornersV = 9;//���� �ڳʼ�
	const int numBoards = 1;//���� ���� ����

	int numSquares = numCornersH * numCornersV;
	Size boardSize(numCornersH, numCornersV);

	//from 2d to 3d convert
	vector<vector<Point3f>> objPoints;
	vector<vector<Point2f>> imgPoints;
	vector<Point3f> obj;
	vector<Point2f> corner;//ã�� �ڳ��� ������ġ

	Mat img=chessImg;//from video to Mat
	Mat grayImg;//convert to gray scale
	Mat intrinsic(3, 3, CV_32FC1);
	Mat distCoeffs;//�Ÿ� ���
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	for (int j = 0; j < numSquares; j++) {
		obj.push_back(Point3f(j / numCornersH, j % numCornersH, 0.0f));
	}
	cvtColor(img, grayImg, COLOR_BGR2GRAY);
	bool found = findChessboardCorners(grayImg, boardSize, corner, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
	if (found) {//cornerã����
		drawChessboardCorners(grayImg, boardSize, corner, found);
		cout << "found Corners" << endl;
		imgPoints.push_back(corner);
		objPoints.push_back(obj);
	}
	else return false;
	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;
	calibrateCamera(objPoints, imgPoints, img.size(), intrinsic, distCoeffs, rvecs, tvecs);
	//Calib�� ����
	intrinsicMat = intrinsic;
	distCoefMat = distCoeffs;
	return true;
}
bool calibImage(VideoCapture& inVideo, Mat& intrinsicMat, Mat& distCoefMat) { // ��� �Է� �ʼ�
	if (!(inVideo.isOpened())) {
		cout << "���� �Էµ��� �ʾҽ��ϴ�." << endl;
		return false;
	}
	//���������� ��
	const int numCornersH = 9;//���� �ڳʼ�
	const int numCornersV = 9;//���� �ڳʼ�
	const int numBoards = 5;//���� ���� ����

	int numSquares = numCornersH * numCornersV;
	Size boardSize(numCornersH, numCornersV);

	//from 2d to 3d convert
	vector<vector<Point3f>> objPoints;
	vector<vector<Point2f>> imgPoints;
	vector<Point3f> obj;
	vector<Point2f> corner;//ã�� �ڳ��� ������ġ

	Mat img;//from video to Mat
	Mat grayImg;//convert to gray scale
	Mat intrinsic(3, 3, CV_32FC1);
	Mat distCoeffs;//�Ÿ� ���
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	for (int j = 0; j < numSquares; j++) {
		obj.push_back(Point3f(j / numCornersH, j % numCornersH, 0.0f));
	}
	while (success < numBoards) {
		inVideo >> img;
		cvtColor(img, grayImg, COLOR_BGR2GRAY);
		imshow("frame", grayImg);
		bool found = findChessboardCorners(grayImg, boardSize, corner, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
		if (found) {//cornerã����
			drawChessboardCorners(grayImg, boardSize, corner, found);
			keyForSnap++;
			if (keyForSnap == 5) {
				cout << "found" << endl;
				imgPoints.push_back(corner);
				objPoints.push_back(obj);
				success++;
				cout << "numBoards:" << numBoards << " success: " << success << endl;
				waitKey(1000);//1�� ���� �Ϸ�Ǹ� �ڸ� ������ �ð��� ����
				keyForSnap = 0;
			}
		}
		if (success >= numBoards) {
			//calib��
			break;
		}
		waitKey(10);
	}
	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;
	calibrateCamera(objPoints, imgPoints, img.size(), intrinsic, distCoeffs, rvecs, tvecs);
	//Calib�� ����
	intrinsicMat = intrinsic;
	distCoefMat = distCoeffs;
	return true;
}

Mat regionOfInterest(Mat& src, Point* points) {// points�� �������� ����-> �������� ������ ���

	Mat maskImg = Mat::zeros(src.size(), CV_8UC1);

	Scalar ignore_mask_color = Scalar(255, 255, 255);
	Scalar red = Scalar(0, 0, 255);
	const Point* ppt[1] = { points };//���� ������ :n vertices
	int npt[] = { 4 };

	fillPoly(maskImg, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);
	Mat maskedImg;
	bitwise_and(src, maskImg, maskedImg);
	return maskedImg;
}