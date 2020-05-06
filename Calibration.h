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

	int numCornerHor = 9; // 수평 점의 개수
	int numCornerVer = 9; // 수직 점의 개수
	int numSquare = numCornerHor * numCornerVer; // 사각형의 개수

	Size board_sz = Size(numCornerHor, numCornerVer);

	vector<vector<Point3f>> object_point; // 실제 3D 차원을 의미하는 객채
	vector<vector<Point2f>> image_point; // 체스 판을 XY평면에 위치시킨 상태를 의미하는 객채

	vector<Point2f> corners;
	int successes = 0; // 하나의 이미지에 대해 Calib을 수행할 때 사용되는 카운터
					   // 체스 판의 이미지 수 만큼 수행되어야 한다.

	Mat image; // 체스 판의 이미지를 저장할 Mat 객채
	Mat gray_image;

	vector<Point3f> obj;
	for (int i = 0; i < numSquare; i++) {
		obj.push_back(Point3f(i / numCornerHor, i % numCornerHor, 0.0f));
		//체스 판에 대해 XY축에 고정된 상태로 가정한다.
		//좌표에 대한 랜덤한 시스템을 위하여 obj 벡터에 체스 판의 가능한 좌표를 모두 저장한다.
	   // (0~numCornerVer, 0~numCornerHor, 0.0f) 범위
	}

	ostringstream osstream; // imread를 위한 sstream

	while (successes < numBoards) { // 모든 체스 판의 사진을 처리 할 때 까지 loop를 실행한다.

		osstream.str("");
		osstream << "capture" << successes + 1 << ".jpg"; // 사진의 제목 처리

		image = imread(osstream.str(), IMREAD_COLOR); // 첫 번째 사진부터 image객채에 read시킨다.

		if (image.empty()) { // image 객채에 대한 오류 검출
			cout << "IMAGE IS EMPTY!" << endl;
			return;
		}

		cvtColor(image, gray_image, COLOR_BGR2GRAY);

		bool found = findChessboardCorners(image, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
		//첫 번째 인자는 3-채널 color 이미지인 InputOutputArray가 들어와야한다.
		//두 번째 인자는 체스 판의 모서리의 Size이다. 칸의 수가 아닌 모서리의 수로 센다.
		//세 번째 인자는 검출된 모서리의 좌표가 입력된다.

		if (found) {
			//cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER), 30, 0.1);
			drawChessboardCorners(image, board_sz, corners, found);
		}
		//imshow("COLOR", image);
		//imshow("GRAY", gray_image);
		//waitKey(10);

		//if (key == 27) // esc 입력 시 종료
		//	return 0;

		//if (key == ' ' && found != 0) { // space bar 입력 시 좌표 값이 객채로 저장됨

		image_point.push_back(corners);
		object_point.push_back(obj);

		cout << successes + 1 << "th snap stored!" << endl; // Console창에 출력

		successes++; // 다음 사진에 대해 loop 실행

		osstream.clear(); // 제목을 담을 osstream 초기화

		if (successes >= numBoards) // 모든 사진에 대해 계산이 완료되면 loop 탈출
			break;
		/*}*/

	}

	vector<Mat> rvecs;
	vector<Mat> tvecs;

	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(0)[0] = 1;

	calibrateCamera(object_point, image_point, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
	// 앞의 세 인자를 이용하여 intrinsic, distCoeffs를 구한다.
	// 두 인자는 카메라의 특성값이다.
}

#endif //CALIBRAION_H