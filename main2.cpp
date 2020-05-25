#include <iostream>
#include <opencv2/opencv.hpp>

#include "Calibration.h"
#include "CustomPicar.h"
#include "DetectColorSign.h"
#include "Driving_DH.h"

using namespace std;
using namespace auto_car;
using namespace cv;

int main()
{
	//Board, Servo, DCmotor configuration-------------------------
	PCA9685 pca{};
	pca.set_pwm_freq(60.0);
	Servo steering(pca, Steering);
	Servo cam_tilt(pca, Tilt);
	Servo cam_pan(pca, Pan);
	Wheel DCmotor(pca, LeftWheel, RightWheel);
	allServoReset(pca);				// 3 Servo motor center reset
	UltraSonic firstSonic(28, 27);	// 초음파센서 객체
	UltraSonic secondSonic(30, 29); // 초음파센서 객체 2(민수: 우측)
	cout << "[Sensor and motor setting complete]" << endl
		<< endl;

	//OpenCV setting----------------------------------------------
	VideoCapture videocap(0); //camera obj
	if (!videocap.isOpened())
	{
		cerr << endl
			<< "video capture fail!" << endl;
		return -1;
	}
	cout << "[VideoCapture loading complete]" << endl
		<< endl;

	//Calibration setting-----------------------------------------
	Size videoSize = Size(640, 480);
	Mat map1, map2, distCoeffs;
	Mat cameraMatrix = Mat(3, 3, CV_32FC1);
	int numBoards = 5;
	DoCalib(distCoeffs, cameraMatrix, numBoards);
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, videoSize, CV_32FC1, map1, map2);
	Mat distortedFrame;
	Mat frame;
	cout << "[calibration complete]" << endl
		<< endl;

	//mode selection----------------------------------------------
	cout << "[visionCar] program start" << endl
		<< endl;

	cout << "Test 1 : Basic test" << endl;
	cout << "Test 2 : Manual test" << endl
		<< endl;

	cout << "Mode 3 : Signal detection(대희)" << endl;
	cout << "Mode 4 : Driving(대희)" << endl;
	cout << "Mode 5 : Parking(석준)" << endl;
	cout << "Mode 6 : Rotary(상민)" << endl;
	cout << "Mode 7 : Overtaking(민수)" << endl;
	cout << "Mode 8 : Tunnel" << endl
		<< endl;

	cout << "Select mode : ";
	int mode;
	cin >> mode;

	if (mode == 1) //Test 1 : Basic test------------------------------------------------------
	{
		steering.setRatio(100); //바퀴 우측
		steering.setRatio(0);	//바퀴 좌측
		steering.resetCenter(); //바퀴 좌우정렬

		cam_tilt.setRatio(100); //카메라 상향
		cam_tilt.setRatio(0);	//카메라 하향
		cam_tilt.resetCenter(); //카메라 상하정렬

		cam_pan.setRatio(100); //카메라 우향
		cam_pan.setRatio(0);   //카메라 좌향
		cam_pan.resetCenter(); //카메라 좌우정렬

		DCmotor.go();  //dc모터 전진
		waitKey(1500); //wait 1.5sec

		DCmotor.backward(); //dc모터 후진
		waitKey(1500);		//wait 1.5sec

		DCmotor.stop(); //dc모터 멈춤
	}
	//End basic test

	else if (mode == 2) //Test 2 : Manual test------------------------------------------------
	{
		//ManualMode class & basic speed rate
		ManualMode Manual(pca, 25);
		Manual.guide();

		//메인루프
		int key(-1);
		while (key != 27) //if not ESC
		{
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR);

			namedWindow("frame", WINDOW_NORMAL);
			imshow("frame", frame);
			resizeWindow("frame", 480, 360);
			moveWindow("frame", 320, 80 + 240);

			key = waitKey(33); //if you not press, return -1
			if (key == 27)
				break;
			else if (key != -1)
				Manual.input(key); //movement by keyboard
			rewind(stdin);
		}
	}
	//End manual test

	else if (mode == 3) //Mode 3 : Signal detection(대희) ------------------------------------
	{
		//color detecting class ganerate
		DetectColorSign detectColorSign(true);

		while (true)
		{
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR);

			if (detectColorSign.priorityStop(distortedFrame, 1.5))
			{
				cout << "A priority stop signal was detected." << '\n';
			}
			else if (detectColorSign.isRedStop(distortedFrame, 1.5)) //빨간색 표지판 감지
			{
				cout << "A red stop sign was detected." << '\n';
			}
			else if (detectColorSign.isYellow(distortedFrame, 1.5)) //노란색 표지판 감지
			{
				cout << "A yellow sign was detected." << '\n';
			}
			else if (detectColorSign.isGreenTurnSignal(distortedFrame, 1.0) == 1) //초록색 표지판 감지
			{
				cout << "<----- signal was detected." << '\n';
			}
			else if (detectColorSign.isGreenTurnSignal(distortedFrame, 1.5) == 2) //초록색 표지판 감지
			{
				cout << "-----> signal was detected." << '\n';
			}
			namedWindow("frame", WINDOW_NORMAL);
			imshow("frame", frame);
			resizeWindow("frame", 480, 360);
			moveWindow("frame", 320, 80 + 240);
			if (waitKey(33) == 27)
				break; //프로그램 종료 ESC(아스키코드 = 27)키.
		}
	}
	//End Signal detection mode

	else if (mode == 4) //Mode 4 : Driving(대희) --------------------------------------------
	{
		//ManualMode class & basic speed rate
		ManualMode Manual(pca, 40);

		//Self-driving class configuration
		Driving_DH DH(true, 1.00);
		DH.mappingSetSection(-0.15, 0.00, 0.15, 0.35, 0.45, 0.50, 0.55);
		DH.mappingSetValue(20.0000, 10.0, 8.00, 0.00, -4.0, 0.00, 40.0); //코너구간 조향수준 맵핑값 세팅
		double steerVal(50.0);								   //초기 각도(50이 중심)
		double speedVal(40.0);								   //초기 속도(0~100)

		bool cornerFlag(false);
		bool rotaryFlag(false);
		int detectedLineCnt(-1);

		//메인동작 루프
		while (true)
		{
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR);
			if (false)
			{
			}
			else //정상주행
			{
				DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);

				if (!cornerFlag && (steerVal == 90 || steerVal == 10))
				{
					cornerFlag = true;
					DH.mappingSetSection(-0.15, 0.00, 0.15, 0.35, 0.45, 0.50, 0.55);
					DH.mappingSetValue(20.0000, 10.0, 15.0, 25.0, 30.0, 35.0, 40.0);
					cout << "cornerFlag ON" << '\n';
				}
				//else if (cornerFlag && steerVal >= 43 && steerVal <= 57)
				else if (cornerFlag && detectedLineCnt == 2)
				{
					cornerFlag = false;
					DH.mappingSetSection(-0.15, 0.00, 0.15, 0.35, 0.45, 0.50, 0.55);
					DH.mappingSetValue(20.0000, 10.0, 8.00, 0.00, -4.0, 0.00, 40.0);
					cout << "cornerFlag OFF" << '\n';
				}

				steering.setRatio(steerVal);

				//DCmotor.go(37);
			}

			namedWindow("frame", WINDOW_NORMAL);
			imshow("frame", frame);
			resizeWindow("frame", 480, 360);
			moveWindow("frame", 320, 80 + 240);

			int key = waitKey(10);
			if (key == 27)
				break; //프로그램 종료 ESC(아스키코드 = 27)키.
			else if (key == 'w')
				DCmotor.go(37);
			else if (key == 'x')
				DCmotor.backward(40);
			else if (key == 's')
				DCmotor.stop();
			else if (key == '0')
			{
				Manual.guide();

				//메인루프
				int key(-1);
				while (key != 27) //if not ESC
				{
					videocap >> distortedFrame;
					remap(distortedFrame, frame, map1, map2, INTER_LINEAR);

					DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);

					if (!cornerFlag && (steerVal == 90 || steerVal == 10))
					{
						cornerFlag = true;
						DH.mappingSetSection(-0.15, 0.00, 0.15, 0.35, 0.45, 0.50, 0.55);
						DH.mappingSetValue(20.0000, 10.0, 15.0, 25.0, 30.0, 35.0, 40.0);
						cout << "cornerFlag ON" << '\n';
					}
					//else if (cornerFlag && steerVal >= 43 && steerVal <= 57)
					else if (cornerFlag && detectedLineCnt == 2)
					{
						cornerFlag = false;
						DH.mappingSetSection(-0.15, 0.00, 0.15, 0.35, 0.45, 0.50, 0.55);
						DH.mappingSetValue(20.0000, 10.0, 8.00, 0.00, -4.0, 0.00, 40.0);
						cout << "cornerFlag OFF" << '\n';
					}

					namedWindow("frame", WINDOW_NORMAL);
					imshow("frame", frame);
					resizeWindow("frame", 480, 360);
					moveWindow("frame", 320, 80 + 240);

					key = waitKey(33); //if you not press, return -1
					if (key == 27)
						break;
					else if (key == '0')
						cout << "Auto driving start" << endl;
					break;
					else if (key != -1)
						Manual.input(key); //movement by keyboard
					rewind(stdin);
				}
			}
		}
	}
	//End Driving mode


	allServoReset(pca);
	cout << "-------------[program finished]-------------" << endl
		<< endl;
	return 0;
}
