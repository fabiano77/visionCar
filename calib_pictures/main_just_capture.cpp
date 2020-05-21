#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "Calibration.h"

using namespace std;
using namespace cv;


int main(void) {


	Mat image;
	Mat intrinsic = Mat(3, 3, CV_32FC1);
	Mat distCoeffs;
	int numBoards = 20;
	DoCalib(distCoeffs, intrinsic, numBoards);

	Mat imageUndistorted; // Calibration 이후의 이미지를 담을 객채

	VideoCapture cap(0);

	while (1) {
		cap >> image;

		if (image.empty()) {
			cout << "IMAGE IS EMPTY" << endl;
			return -1;
		}

		undistort(image, imageUndistorted, intrinsic, distCoeffs); // image에 대해 Calibration 수행

		imshow("Origin", image);
		imshow("AFTERCALIB", imageUndistorted);

		waitKey(33);
	}
	return 0;
}
