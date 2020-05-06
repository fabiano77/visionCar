#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include "CustomPicar.h"
#include "CV_Calibration.h"
#include "CV_drivingAngle.h"
#include "Driving_DH.h"
#include "DetectColorSign.h"

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
		ManualMode Manual(pca, 40);	//ManualMode class & basic speed rate
		Manual.guide();				//cout the key guide 
		int key(-1);
		while (key != 27)			//if not ESC
		{
			videocap >> frame;
			imshow("Live camera", frame);
			int key = waitKey(delay);	//if you not press, return -1
			Manual.input(key);		//movement by keyboard
		}
	}
	//end manual mode


	else if (mode == 3) {
		Mat intrinsic;
		Mat disCoeff;
		if (calibImage(videocap, intrinsic, disCoeff)) {
			cout << "Calibration Success!" << endl;
		}
		else {
			cout << "Calibration Failed!" << endl;
		}
		Mat undistortImg;
		vector<Vec4i> exLines;
		double stiring;
		while (1) {
			videocap >> frame;
			undistort(frame, undistortImg, intrinsic, disCoeff);
			imshow("Live", undistortImg);

			bool Check = extractLines(undistortImg, exLines);
			drivingAngle(undistortImg, exLines, stiring);
			waitKey(15);
		}
	}
	//end calb mode


	else if (mode == 4)	//daehee's code
	{
		/*
		~code processing calibration~
		*/
		//Mat undistortFrame;

		Driving_DH DH(true, 1.00);	//printFlag, sLevel
									//sLevel : 직선구간 민감도(높을수록 많이 꺾임)
		DH.mappingSetSection(0, 0.10, 0.40, 0.70, 0.77, 1.00);
		DH.mappingSetValue(0.0, 0.00, 10.0, 25.0, 50.0, 50.0);
		//DH.mappingSetValue(10, 10.0, 15.0, 25.0, 50.0, 50.0);
		//코너구간 조향수준 맵핑값 세팅

		DetectColorSign detectColorSign(false);	//색깔 표지판 감지 클래스

		double steerVal(50.0);	//초기 각도(50이 중심)
		double speedVal(40.0);	//초기 속도(0~100)

		while (true)
		{
			videocap >> frame;
			//undistort(frame, undistortFrame, intrinsic, disCoeff);

			if (false) //event 체크
			{

			}
			else if (detectColorSign.isRedStop(frame, 10)) //빨간색 표지판 감지
			{
				while (detectColorSign.isRedStop(frame, 10))
				{
					DCmotor.stop();	//멈춘다.
				}
			}
			else if (false)	//기타 event 체크
			{

			}
			else //정상주행
			{
				//DH.driving(undistortFrame, steerVal, speedVal, 40.0, 2.0);
				DH.driving(frame, steerVal, speedVal, 30.0, 0.0);

				steering.setRatio(steerVal);			//바퀴 조향
				DCmotor.go(speedVal);

				cout << "steer : " << steerVal << ", speed : " << speedVal << endl;

				//imshow("frame", undistortFrame);
				imshow("frame", frame);
				waitKey(10);//33
			}
		}
	}
	//end daehee's code


	else if (mode == 5) // SangMin's code
	{
		Mat intrinsic;
		Mat disCoeff;
		if (calibImage(videocap, intrinsic, disCoeff)) {
			cout << "Calibration Success!" << endl;
		}
		else {
			cout << "Calibration Failed!" << endl;
		}
		Mat undistortImg;
		vector<Vec4i> exLines;

		double speedVal(40.0);	//초기 속도(0~100)
		double steering_After, steering_Before = 0;
		
		int Mode;
		cout << "select Mode : ";
		cin >> Mode;
		cout << "Mode : " << Mode << endl;
		while (1) {
			videocap >> frame;
			undistort(frame, undistortImg, intrinsic, disCoeff);
			imshow("Live", undistortImg);

			bool Check = extractLines(undistortImg, exLines);
			drivingAngle_SM(undistortImg, exLines, steering_After, steering_Before, Mode);
			steering.setRatio(50 + steering_After); //바퀴 조향
			cout << "조향각 : " << 50 + steering_After << endl;			
			DCmotor.go(speedVal);
			waitKey(15);
			
		}
	}
	//end SangMin's code

	else if (mode == 6)
	{
		//write your code
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
