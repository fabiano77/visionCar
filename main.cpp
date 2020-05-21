#include <iostream>
#include <opencv2/opencv.hpp>
#include "CustomPicar.h"
#include "Driving_DH.h"
#include "DetectColorSign.h"
#include "Calibration.h"
using namespace std;
using namespace auto_car;
using namespace cv;

int main()
{
	//Board, Servo, DCmotor configuration-------------------------
	PCA9685 pca{};
	//pca.set_pwm_freq(60.0);
	//Servo steering(pca, Steering);
	//Servo cam_tilt(pca, Tilt);
	//Servo cam_pan(pca, Pan);
	//Wheel DCmotor(pca, LeftWheel, RightWheel);
	//allServoReset(pca);				// 3 Servo motor center reset
	UltraSonic firstSonic(28, 27);	// 초음파센서 객체
	cout << "[Sensor and motor setting complete]" << endl << endl;

	//OpenCV setting----------------------------------------------
	VideoCapture videocap(0);	//camera obj
	if (!videocap.isOpened())
	{
		cerr << endl << "video capture fail!" << endl;
		return -1;
	}
	cout << "[VideoCapture loading complete]" << endl << endl;

	//Calibration setting-----------------------------------------
	Size videoSize = Size(640, 480);
	Mat map1, map2, distCoeffs;
	Mat cameraMatrix = Mat(3, 3, CV_32FC1);
	int numBoards = 5;
	DoCalib(distCoeffs, cameraMatrix, numBoards);
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, videoSize, CV_32FC1, map1, map2);
	Mat distortedFrame;
	Mat frame;
	cout << "[calibration complete]" << endl << endl;

	//mode selection----------------------------------------------
	cout << "[visionCar] program start" << endl << endl;

	cout << "Test 1 : Basic test" << endl;
	cout << "Test 2 : Manual test" << endl << endl;

	cout << "Mode 3 : Signal detection(대희)" << endl;
	cout << "Mode 4 : Driving(대희)" << endl;
	cout << "Mode 5 : Parking(석준)" << endl;
	cout << "Mode 6 : Rotary(상민)" << endl;
	cout << "Mode 7 : Overtaking(민수)" << endl;
	cout << "Mode 8 : Tunnel" << endl << endl;

	cout << "Select mode : ";
	int mode;
	cin >> mode;

	//if (mode == 1)//Test 1 : Basic test------------------------------------------------------
	//{
	//	steering.setRatio(100);			//바퀴 우측
	//	steering.setRatio(0);			//바퀴 좌측
	//	steering.resetCenter();			//바퀴 좌우정렬

	//	cam_tilt.setRatio(100);			//카메라 상향
	//	cam_tilt.setRatio(0);			//카메라 하향
	//	cam_tilt.resetCenter();			//카메라 상하정렬

	//	cam_pan.setRatio(100);			//카메라 우향
	//	cam_pan.setRatio(0);			//카메라 좌향
	//	cam_pan.resetCenter();			//카메라 좌우정렬

	//	DCmotor.go();					//dc모터 전진
	//	waitKey(1500);					//wait 1.5sec

	//	DCmotor.backward();				//dc모터 후진
	//	waitKey(1500);					//wait 1.5sec

	//	DCmotor.stop();					//dc모터 멈춤
	//}
	////End basic test

	//else if (mode == 2)//Test 2 : Manual test------------------------------------------------
	//{
	//	//Self-driving class configuration
	//	Driving_DH DH(true, 1.00);
	//	DH.mappingSetSection(0, 0.10, 0.40, 0.75, 0.79, 1.00);
	//	DH.mappingSetValue(8.0, 8.00, 15.0, 20.0, 50.0, 50.0);
	//	double steerVal(50.0);
	//	double speedVal(40.0);

	//	//ManualMode class & basic speed rate
	//	ManualMode Manual(pca, 40);
	//	Manual.guide();

	//	//메인루프
	//	int key(-1);
	//	while (key != 27)//if not ESC
	//	{
	//		videocap >> distortedFrame;
	//		remap(distortedFrame, frame, map1, map2, INTER_LINEAR);

	//		DH.driving(frame, steerVal, speedVal, 37.0, 0.0);

	//		namedWindow("frame", WINDOW_NORMAL);
	//		imshow("frame", frame);
	//		resizeWindow("frame", 480, 360);
	//		moveWindow("frame", 0, 0);

	//		key = waitKey(33);//if you not press, return -1
	//		if (key != -1) Manual.input(key);//movement by keyboard
	//		rewind(stdin);
	//	}
	//}
	////End manual test

	//else if (mode == 3)//Mode 3 : Signal detection(대희) ------------------------------------
	//{
	//	//color detecting class ganerate
	//	DetectColorSign detectColorSign(true);

	//	while (true)
	//	{
	//		videocap >> distortedFrame;
	//		remap(distortedFrame, frame, map1, map2, INTER_LINEAR);

	//		if (detectColorSign.priorityStop(distortedFrame, 1.5))
	//		{
	//			cout << "A priority stop signal was detected." << '\n';
	//		}
	//		else if (detectColorSign.isRedStop(distortedFrame, 1.5)) //빨간색 표지판 감지
	//		{
	//			cout << "A red stop sign was detected." << '\n';
	//		}
	//		else if (detectColorSign.isYellow(distortedFrame, 1.5)) //노란색 표지판 감지
	//		{
	//			cout << "A yellow sign was detected." << '\n';
	//		}
	//		else if (detectColorSign.isGreenTurnSignal(distortedFrame, 1.0) == 1) //초록색 표지판 감지
	//		{
	//			cout << "<----- signal was detected." << '\n';
	//		}
	//		else if (detectColorSign.isGreenTurnSignal(distortedFrame, 1.5) == 2) //초록색 표지판 감지
	//		{
	//			cout << "-----> signal was detected." << '\n';
	//		}
	//		namedWindow("frame", WINDOW_NORMAL);
	//		imshow("frame", frame);
	//		resizeWindow("frame", 480, 360);
	//		moveWindow("frame", 320, 60+240);
	//		if (waitKey(33) == 27) break;	//프로그램 종료 ESC(아스키코드 = 27)키.
	//	}
	//}
	////End Signal detection mode

	//else if (mode == 4)	//Mode 4 : Driving(대희) --------------------------------------------
	//{
	//	//Self-driving class configuration
	//	Driving_DH DH(true, 1.00);
	//	cout << "corner value select : ";
	//	cin >> mode;
	//	switch (mode)
	//	{
	//	case 1:		//기존
	//		DH.mappingSetSection(0, 0.10, 0.50, 0.77, 0.81, 1.00);
	//		DH.mappingSetValue(6.0, 6.00, 8.00, 10.0, 50.0, 50.0);	//코너구간 조향수준 맵핑값 세팅
	//		break;
	//	case 2:		//중간 0값
	//		DH.mappingSetSection(0, 0.10, 0.30, 0.75, 0.80, 1.00);
	//		DH.mappingSetValue(6.0, 6.00, 0.00, 0.00, 50.0, 50.0);	//코너구간 조향수준 맵핑값 세팅
	//		break;
	//	case 3:		//중간 음수
	//		DH.mappingSetSection(0, 0.10, 0.30, 0.70, 0.80, 1.00);
	//		DH.mappingSetValue(6.0, 6.00, 0.00, -4.0, 50.0, 50.0);	//코너구간 조향수준 맵핑값 세팅
	//		break;
	//	case 4:		//중간 음수
	//		DH.mappingSetSection(0, 0.10, 0.30, 0.75, 0.80, 1.00);
	//		DH.mappingSetValue(6.0, 6.00, 0.00, -4.0, 50.0, 50.0);	//코너구간 조향수준 맵핑값 세팅
	//		break;
	//	case 7:		//코너플래그 활성화
	//		DH.mappingSetSection(0, 0.40, 0.50, 0.75, 0.80, 1.00);
	//		DH.mappingSetValue(5.0, 5.00, 8.00, 10.0, 50.0, 50.0);	//코너구간 조향수준 맵핑값 세팅
	//		break;
	//	default:

	//		break;
	//	}
	//	//DH.mappingSetSection(0, 0.10, 0.40, 0.73, 0.79, 1.00);
	//	//DH.mappingSetValue(8.0, 8.00, 15.0, 22.0, 50.0, 50.0);	//코너구간 조향수준 맵핑값 세팅
	//	double steerVal(50.0);	//초기 각도(50이 중심)
	//	double speedVal(40.0);	//초기 속도(0~100)
	//	bool cornerFlag(false);
	//	bool waitingFlag(true);

	//	//카메라 좌우 보정 50->52 : 살짝 우측으로 보정
	//	cam_pan.setRatio(52);

	//	//메인동작 루프
	//	while (true)
	//	{
	//		videocap >> distortedFrame;
	//		remap(distortedFrame, frame, map1, map2, INTER_LINEAR);

	//		if (false)
	//		{

	//		}
	//		else if (mode == 7 && cornerFlag) //코너 flag on일때, 조향하지 않고 꺾은채 유지.
	//		{
	//			DH.driving(frame, steerVal, speedVal, 37.0, 0.0);
	//			if (steerVal >= 30 && steerVal <= 70) cornerFlag = false;	//코너 flag 해제

	//			DCmotor.go(30);
	//		}
	//		else //정상주행
	//		{
	//			DH.driving(frame, steerVal, speedVal, 38.0, 0.0);

	//			if (cornerFlag = true)
	//			{
	//				if (steerVal >= 44 && steerVal <= 56) cornerFlag = false;
	//				steering.setRatio(steerVal);
	//			}
	//			else	//기본 false
	//			{
	//				if (mode == 7 && (steerVal >= 75 || steerVal <= 25)) cornerFlag = true;
	//				if (steerVal >= 56)
	//				{
	//					steering.setRatio(56);
	//				}
	//				else if (steerVal >= 44)
	//				{
	//					steering.setRatio(44);
	//				}
	//				else
	//				{
	//					steering.setRatio(steerVal);
	//				}
	//			}
	//			DCmotor.go(speedVal);
	//		}

	//		namedWindow("frame", WINDOW_NORMAL);
	//		imshow("frame", frame);
	//		resizeWindow("frame", 480, 360);
	//		moveWindow("frame", 0, 0);

	//		if (waitKey(1) == 27) break;	//프로그램 종료 ESC(아스키코드 = 27)키.
	//	}
	//}
	////End Driving mode


	//else if (mode == 5)//Mode 5 : Parking(석준) ---------------------------------------------
	//{
	//	double Distance;	//거리값

	//	while (true)
	//	{
	//		videocap >> distortedFrame;
	//		remap(distortedFrame, frame, map1, map2, INTER_LINEAR);	//캘리된 영상 frame

	//		Distance = firstSonic.distance();	//초음파 거리측정.

	//		cout << "distance = " << Distance << endl;	//거리출력

	//		steering.setRatio(50);	//바퀴조향
	//		DCmotor.go();			//dc모터 전진
	//		DCmotor.backward();		//dc모터 후진
	//		DCmotor.stop();			//정지

	//		imshow("frame", frame);
	//		if (waitKey(33) == 27) break;	//프로그램 종료 ESC키.
	//	}
	//}
	////End Parking mode


	//else if (mode == 6)//Mode 6 : Rotary(상민) ----------------------------------------------
	//{
	//	double Distance;	//거리값

	//	while (true)
	//	{
	//		videocap >> distortedFrame;
	//		remap(distortedFrame, frame, map1, map2, INTER_LINEAR);	//캘리된 영상 frame

	//		Distance = firstSonic.distance();	//초음파 거리측정.

	//		cout << "distance = " << Distance << endl;	//거리출력

	//		steering.setRatio(50);	//바퀴조향
	//		DCmotor.go();			//dc모터 전진
	//		DCmotor.backward();		//dc모터 후진
	//		DCmotor.stop();			//정지

	//		imshow("frame", frame);
	//		if (waitKey(33) == 27) break;	//프로그램 종료 ESC키.
	//	}
	//}
	////End Rotary mode


	//else if (mode == 7)//Mode 7 : Overtaking(민수) ------------------------------------------
	//{
	//	double Distance;	//거리값

	//	while (true)
	//	{
	//		videocap >> distortedFrame;
	//		remap(distortedFrame, frame, map1, map2, INTER_LINEAR);	//캘리된 영상 frame

	//		Distance = firstSonic.distance();	//초음파 거리측정.

	//		cout << "distance = " << Distance << endl;	//거리출력

	//		steering.setRatio(50);	//바퀴조향
	//		DCmotor.go();			//dc모터 전진
	//		DCmotor.backward();		//dc모터 후진
	//		DCmotor.stop();			//정지

	//		imshow("frame", frame);
	//		if (waitKey(33) == 27) break;	//프로그램 종료 ESC키.
	//	}
	//}
	////End Overtaking mode



	//else if (mode == 8)//Mode 8 : Tunnel ----------------------------------------------------
	//{
	//	double Distance;	//거리값

	//	while (true)
	//	{
	//		videocap >> distortedFrame;
	//		remap(distortedFrame, frame, map1, map2, INTER_LINEAR);	//캘리된 영상 frame

	//		Distance = firstSonic.distance();	//초음파 거리측정.

	//		cout << "distance = " << Distance << endl;	//거리출력

	//		steering.setRatio(50);	//바퀴조향
	//		DCmotor.go();			//dc모터 전진
	//		DCmotor.backward();		//dc모터 후진
	//		DCmotor.stop();			//정지

	//		imshow("frame", frame);
	//		if (waitKey(33) == 27) break;	//프로그램 종료 ESC키.
	//	}
	//}
	////End Tunnel mode



	//cout << "-------------[program finished]-------------" << endl << endl;
	//allServoReset(pca);
	return 0;
}
