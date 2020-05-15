#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include "CustomPicar.h"
#include "SM_drivingAngle.h"
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
	cout << "mode 5 : SangMin's code" << endl;
	cout << "mode 6 : time Check" << endl << endl;
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

	else if (mode == 4) {
		CheckStart cs;
		Size videoSize = Size(640, 480);
		Mat map1, map2, disCoeffs;
		Mat cameraMatrix = Mat(3, 3, CV_32FC1);
		int numBoards = 7;
		DoCalib(disCoeffs, cameraMatrix, numBoards);
		initUndistortRectifyMap(cameraMatrix, disCoeffs, Mat(), cameraMatrix, videoSize, CV_32FC1, map1, map2);

		Mat distortedFrame;
		cout << "[calibration complete]" << endl;

		while (1) {
			videocap >> frame;
			remap(frame, distortedFrame, map1, map2, INTER_LINEAR);
			bool check = cs.isStart(distortedFrame, 75);
			if (!check)
				cs.GetFlag();
			imshow("Live", distortedFrame);			
			int key = waitKey(15);	//33
			if (key == 27) break;	//프로그램 종료 ESC키.
		}	
	}

	else if (mode == 5) // SangMin's code
	{
		cam_tilt.setRatio(10);
		Size videoSize = Size(640, 480);
		Mat map1, map2, disCoeffs;
		Mat cameraMatrix = Mat(3, 3, CV_32FC1);
		int numBoards = 7;
		DoCalib(disCoeffs, cameraMatrix, numBoards);
		initUndistortRectifyMap(cameraMatrix, disCoeffs, Mat(), cameraMatrix, videoSize, CV_32FC1, map1, map2);

		Mat distortedFrame;
		vector<Vec4i> exLines;
		cout << "[calibration complete]" << endl;

		double speedVal(35.0);	//초기 속도(0~100)
		double steering_After, steering_Before = 0;

		int Mode;
		cout << "select Mode(1,2) : ";
		cin >> Mode;
		cout << "Mode : " << Mode << endl << endl;
		while (1) {
			videocap >> frame;
			remap(frame, distortedFrame, map1, map2, INTER_LINEAR);
			imshow("Live", distortedFrame);

			bool Check = extractLines(distortedFrame, exLines);
			drivingAngle_SM(distortedFrame, exLines, steering_After, steering_Before, Mode);
			steering.setRatio(50 + steering_After); //바퀴 조향
			//cout << "조향각 : " << 50 + steering_After << endl;
			DCmotor.go(speedVal);			
			//waitKey(15);	
			int key = waitKey(15);	//33
			if (key == 27) break;	//프로그램 종료 ESC키.
		}
	}
	//end SangMin's code

	else if (mode == 6)
	{
		cam_tilt.setRatio(10);
		Size videoSize = Size(640, 480);
		Mat map1, map2;
		Mat cameraMatrix = Mat(3, 3, CV_32FC1);
		Mat disCoeffs;
		int numBoards = 5;
		DoCalib(disCoeffs, cameraMatrix, numBoards);
		initUndistortRectifyMap(cameraMatrix, disCoeffs, Mat(), cameraMatrix, videoSize, CV_32FC1, map1, map2);

		Mat distortedFrame;
		vector<Vec4i> exLines;

		double speedVal(35.0);	//초기 속도(0~100)
		double steering_After, steering_Before = 0;

		int Mode;
		cout << "select Mode(1,2) : ";
		cin >> Mode;
		cout << "Mode : " << Mode << endl;
		while (1) {
			//TickMeter tm;	//시간 측정 클래스
			//tm.start();
			TickMeter tm7;
			tm7.start();

			TickMeter tm1;
			tm1.start();
			videocap >> frame;
			tm1.stop();

			TickMeter tm2;
			tm2.start();
			remap(frame, distortedFrame, map1, map2, INTER_LINEAR);
			tm2.stop();

			TickMeter tm3;
			tm3.start();
			imshow("Live", distortedFrame);
			tm3.stop();

			bool Check = extractLines(distortedFrame, exLines);

			TickMeter tm4;
			tm4.start();
			drivingAngle_SM(distortedFrame, exLines, steering_After, steering_Before, Mode);
			tm4.stop();

			TickMeter tm5;
			tm5.start();
			steering.setRatio(50 + steering_After); //바퀴 조향
			tm5.stop();

			//waitKey(15);
			if (waitKey(1) == 27) break;	//프로그램 종료 ESC(아스키코드 = 27)키.

			tm7.stop();
			//DCmotor.go(speedVal);
			//tm.stop();		//시간측정 끝
			//cout << tm.getTimeMilli() << "[ms]" << '\n';
			cout << "videocap >>    : " << tm1.getTimeMilli() << "[ms]" << '\n';
			cout << "undistort()    : " << tm2.getTimeMilli() << "[ms]" << '\n';
			cout << "imshow() : " << tm3.getTimeMilli() << "[ms]" << '\n';
			cout << "drivingAngle_SM() : " << tm4.getTimeMilli() << "[ms]" << '\n';
			cout << "steering.set() : " << tm5.getTimeMilli() << "[ms]" << '\n';
			cout << "all time   : " << tm7.getTimeMilli() << "[ms]" << '\n' << '\n';
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
