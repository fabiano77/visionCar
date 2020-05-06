#ifndef CALIBRAION_H
#define CALIBRAION_H

#include <iostream>
#include <sstream>
#include <string>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

void DoCalib(Mat& distCoeffs, Mat& intrinsic, int& numBoards);

void DoCalib(Mat& distCoeffs, Mat& intrinsic, int& numBoards) {

	int numCornerHor = 9; // ���� ���� ����
	int numCornerVer = 9; // ���� ���� ����
	int numSquare = numCornerHor * numCornerVer; // �簢���� ����

	Size board_sz = Size(numCornerHor, numCornerVer);

	vector<vector<Point3f>> object_point; // ���� 3D ������ �ǹ��ϴ� ��ä
	vector<vector<Point2f>> image_point; // ü�� ���� XY��鿡 ��ġ��Ų ���¸� �ǹ��ϴ� ��ä

	vector<Point2f> corners;
	int successes = 0; // �ϳ��� �̹����� ���� Calib�� ������ �� ���Ǵ� ī����
					   // ü�� ���� �̹��� �� ��ŭ ����Ǿ�� �Ѵ�.

	Mat image; // ü�� ���� �̹����� ������ Mat ��ä
	Mat gray_image;

	vector<Point3f> obj;
	for (int i = 0; i < numSquare; i++) {
		obj.push_back(Point3f(i / numCornerHor, i % numCornerHor, 0.0f));
		//ü�� �ǿ� ���� XY�࿡ ������ ���·� �����Ѵ�.
		//��ǥ�� ���� ������ �ý����� ���Ͽ� obj ���Ϳ� ü�� ���� ������ ��ǥ�� ��� �����Ѵ�.
	   // (0~numCornerVer, 0~numCornerHor, 0.0f) ����
	}

	ostringstream osstream; // imread�� ���� sstream

	while (successes < numBoards) { // ��� ü�� ���� ������ ó�� �� �� ���� loop�� �����Ѵ�.

		osstream.str("");
		osstream << "capture" << successes + 1 << ".jpg"; // ������ ���� ó��

		image = imread(osstream.str(), IMREAD_COLOR); // ù ��° �������� image��ä�� read��Ų��.

		if (image.empty()) { // image ��ä�� ���� ���� ����
			cout << "IMAGE IS EMPTY!" << endl;
			return;
		}

		cvtColor(image, gray_image, COLOR_BGR2GRAY);

		bool found = findChessboardCorners(image, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
		//ù ��° ���ڴ� 3-ä�� color �̹����� InputOutputArray�� ���;��Ѵ�.
		//�� ��° ���ڴ� ü�� ���� �𼭸��� Size�̴�. ĭ�� ���� �ƴ� �𼭸��� ���� ����.
		//�� ��° ���ڴ� ����� �𼭸��� ��ǥ�� �Էµȴ�.

		if (found) {
			//cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER), 30, 0.1);
			drawChessboardCorners(image, board_sz, corners, found);
		}
		//imshow("COLOR", image);
		//imshow("GRAY", gray_image);
		//waitKey(10);

		//if (key == 27) // esc �Է� �� ����
		//	return 0;

		//if (key == ' ' && found != 0) { // space bar �Է� �� ��ǥ ���� ��ä�� �����

		image_point.push_back(corners);
		object_point.push_back(obj);

		cout << successes + 1 << "th snap stored!" << endl; // Consoleâ�� ���

		successes++; // ���� ������ ���� loop ����

		osstream.clear(); // ������ ���� osstream �ʱ�ȭ

		if (successes >= numBoards) // ��� ������ ���� ����� �Ϸ�Ǹ� loop Ż��
			break;
		/*}*/

	}

	vector<Mat> rvecs;
	vector<Mat> tvecs;

	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(0)[0] = 1;

	calibrateCamera(object_point, image_point, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
	// ���� �� ���ڸ� �̿��Ͽ� intrinsic, distCoeffs�� ���Ѵ�.
	// �� ���ڴ� ī�޶��� Ư�����̴�.
}

#endif //CALIBRAION_H