#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include"CV_calibration.h"
using namespace std;
using namespace cv;

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
		cvtColor(img, grayImg, CV_BGR2GRAY);
		imshow("frame", grayImg);
		bool found = findChessboardCorners(grayImg, boardSize, corner, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found) {//cornerã����
			cornerSubPix(grayImg, corner, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
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