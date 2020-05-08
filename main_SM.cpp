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
	cout << "mode 6 : who's code?" << endl;
	cout << "mode 7 : who's code?" << endl << endl;
	cout << "select mode : ";
	int mode;
	cin >> mode;

	//start mode------------------------------------------------
	if (mode == 1)//test mode
	{
		steering.setRatio(100);			//바퀴 우측
		steering.setRatio(0);			//바퀴 좌측
		steering.resetCenter();			//바퀴 정렬
		cam_tilt.setRatio(100);			//카메라 상향
		cam_tilt.setRatio(0);			//카메라 하향
		cam_tilt.resetCenter();			//카메라 정렬
		cam_pan.setRatio(100);			//카메라 우향
		cam_pan.setRatio(0);			//카메라 좌향
		cam_pan.resetCenter();			//카메라 정렬
		DCmotor.go();					//dc모터 시작
		waitKey(1500);					//wait 1.5sec
		DCmotor.stop();					//dc모터 멈춤
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
		Mat intrinsic = Mat(3, 3, CV_32FC1);
		Mat disCoeffs;
		int numBoards = 5;
		DoCalib(disCoeffs, intrinsic, numBoards);

		Mat undistortImg;
		vector<Vec4i> exLines;

		double speedVal(35.0);	//초기 속도(0~100)
		double steering_After, steering_Before = 0;

		int Mode;
		cout << "select Mode : ";
		cin >> Mode;
		cout << "Mode : " << Mode << endl;
		while (1) {
			TickMeter tm;	//시간 측정 클래스
			tm.start();
			videocap >> frame;
			undistort(frame, undistortImg, intrinsic, disCoeffs);
			imshow("Live", undistortImg);

			bool Check = extractLines(undistortImg, exLines);
			drivingAngle_SM(undistortImg, exLines, steering_After, steering_Before, Mode);
			steering.setRatio(50 + steering_After); //바퀴 조향
			cout << "조향각 : " << 50 + steering_After << endl;
			DCmotor.go(speedVal);			
			waitKey(15);
			tm.stop();		//시간측정 끝
			cout << tm.getTimeMilli() << "[ms]" << '\n';
		}
	}
	//end SangMin's code

	else if (mode == 6)
	{
		Mat intrinsic = Mat(3, 3, CV_32FC1);
		Mat disCoeffs;
		int numBoards = 5;
		DoCalib(disCoeffs, intrinsic, numBoards);

		Mat undistortImg;
		vector<Vec4i> exLines;

		double speedVal(35.0);	//초기 속도(0~100)
		double steering_After, steering_Before = 0;

		int Mode;
		cout << "select Mode : ";
		cin >> Mode;
		cout << "Mode : " << Mode << endl;
		while (1) {
			TickMeter tm;	//시간 측정 클래스
			tm.start();
			videocap >> frame;
			undistort(frame, undistortImg, intrinsic, disCoeffs);

			bool Check = extractLines(undistortImg, exLines);
			drivingAngle_SM(undistortImg, exLines, steering_After, steering_Before, Mode);
			steering.setRatio(50 + steering_After); //바퀴 조향
			DCmotor.go(speedVal);
			tm.stop();		//시간측정 끝
			cout << tm.getTimeMilli() << "[ms]" << '\n';
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
