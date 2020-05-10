
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include "CustomPicar.h"
#include "CV_drivingAngle.h"
#include "Calibration.h"

//cpp를 추가해보는 것은 어떠한가

using namespace std;
using namespace auto_car;
using namespace cv;

int main()
{
	//board, servoMotor configuration-----------------------------
	PCA9685 pca{};
	pca.set_pwm_freq(60.0);
	Servo steering(pca, Steering);
	Servo cam_tilt(pca, Tilt);
	Servo cam_pan(pca, Pan);
	Wheel DCmotor(pca, LeftWheel, RightWheel);
	allServoReset(pca);			// 3 Servo motor center reset

	//OpenCV setting----------------------------------------------
	Mat frame;					//standard Mat obj
	VideoCapture videocap(0);	//camera obj
	if (!videocap.isOpened()) {
		cerr << "video capture fail!" << endl;
		return -1;
	}
	cout << "Camera test is complete" << endl << endl;

	//mode selection---------------------------------------------
	cout << "[visionCar] program start" << endl << endl;
	cout << "mode 1 : test mode" << endl;
	cout << "mode 2 : manual mode" << endl;
	cout << "mode 3 : calb & angle mode" << endl;
	cout << "mode 4 : daehee's code" << endl;
	cout << "mode 5 : who's code?" << endl;
	cout << "mode 6 : Minsoo's code" << endl;
	cout << "mode 7 : who's code?" << endl << endl;
	cout << "select mode : ";
	int mode;
	cin >> mode;

	//start mode------------------------------------------------
	if (mode == 1)//test mode
	{

	}
	//end test mode


	else if (mode == 2)//manual mode
	{

	}
	//end manual mode


	else if (mode == 3) {

	}
	//end calb mode


	else if (mode == 4)	//daehee's code
	{

	}
	//end daehee's code


	else if (mode == 5) // SangMin's code
	{

	}
	//end SangMin's code

	else if (mode == 6)
	{
		Mat intrinsic = Mat(3, 3, CV_32FC1);
		Mat disCoeffs;
		int numBoards = 20;
		DoCalib(disCoeffs, intrinsic, numBoards);
		Mat distortFrame;
		Mat srcImg;
		videocap >> srcImg;
		int width = srcImg.size().width;
		int height = srcImg.size().height;
		Mat grayImg;
		Mat blurImg, edgeImg;
		Mat dstImg(srcImg.size(), CV_8UC3);
		Mat filteredImg;
		Steer steer1;
		double steeringValue;	//초기 각도(50이 중심)
		double speedVal_straight(40.0);	//초기 속도(0~100)
		double speedVal_turn(30.0);
		bool gostop = true;
		Point pointROI[4] = { Point(width * 3 / 7,height * 3 / 5),Point(width * 4 / 7,height * 3 / 5),Point(width,height * 6 / 7),Point(0,height * 6 / 7) };
		Scalar WHITE_BGR(255, 255, 255);
		while (videocap.isOpened()&&gostop==true) {
			videocap >> distortFrame;
			undistort(distortFrame, frame, intrinsic, disCoeffs);
			filter_colors(frame, filteredImg);
			//cvtColor(srcImg, grayImg, COLOR_BGR2GRAY);
			//이미지 처리 blur 및 edge 검출

			imgProcessing(filteredImg, blurImg, 1);
			imgProcessing(blurImg, edgeImg, 2);

			//roi 설정
			//Point pt[4] = { Point(0,height * 1/2),Point(width,height * 1/2),Point(width,height),Point(0,height) };
			
			Mat roiImg;
			
			regionOfInterest(edgeImg, roiImg,pointROI);

			vector<Vec4i> lines;
			imshow("roiImg", roiImg);
			HoughLinesP(roiImg, lines, 1, CV_PI / 180.0, 30, 10, 20);
			srcImg.copyTo(dstImg);


			drivingAngle_MS(dstImg, lines, steeringValue, steer1);
			if (abs(steeringValue)>24) {
				DCmotor.go(speedVal_turn);
			}
			else {
				DCmotor.go(speedVal_straight);
			}
			steeringValue = 50 + steeringValue;
			steering.setRatio(steeringValue);//방향각 조정

			gostop=steer1.gostop();
			if (gostop == false) {
				DCmotor.stop();
				cout << "정지합니다" << endl;
				break;
			}

			//imshow("roiImg", roiImg);			//바퀴 조향
			
		}

	}


	else if (mode == 7)
	{
		//write your code
	}

	else cout << "invalid mode selection" << endl;

	cout << "program finished" << endl;
	allServoReset(pca);	// 3 Servo motor center reset
	return 0;
	//끝
}
