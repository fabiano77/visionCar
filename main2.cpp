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
		bool cornerFlag(false);
		bool manualFlag(false);
		int detectedLineCnt(-1);
		double steerVal(50.0);			//초기 각도(50이 중심)
		DH.mappingSet(cornerFlag);		//조향수준 맵핑값 세팅

		bool rotaryFlag(false);

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

				if (!cornerFlag && steerVal == 90 || steerVal == 10)	//최대 각 검출되면 cornerFlag ON
				{
					cornerFlag = true;
					DH.mappingSet(cornerFlag);
					cout << "cornerFlag ON" << '\n';
				}
				else if (cornerFlag && detectedLineCnt == 2)			//직선 두개 검출되면 cornerFlag OFF
				{
					cornerFlag = false;
					DH.mappingSet(cornerFlag);
					cout << "cornerFlag OFF" << '\n';
				}
				if (!manualFlag) steering.setRatio(steerVal);

				//DCmotor.go(37);
			}

			namedWindow("frame", WINDOW_NORMAL);
			imshow("frame", frame);
			resizeWindow("frame", 480, 360);
			moveWindow("frame", 320, 80 + 240);

			int key = waitKey(10);
			if (key == 27)
				break; //프로그램 종료 ESC(아스키코드 = 27)키.
			else if (key == '0')
			{
				manualFlag = !manualFlag;
				Manual.guide();
				cout << "Auto driving start" << endl;
			}
			else if (manualFlag && key != -1)
			{
				Manual.input(key); //movement by keyboard
				rewind(stdin);
			}
			else if (key == 'w')
				DCmotor.go(37);
			else if (key == 'x')
				DCmotor.backward(40);
			else if (key == 's')
				DCmotor.stop();

		}
	}
	//End Driving mode



	else if (mode == 5) //Mode 5 : Parking(석준) ---------------------------------------------
	{
		double sideDistance = 0;	  // 측면 거리센서 값
		double backDistance = 0;	  // 후방 거리센서 값
		int caseNum = 0;			  // Switch - Case 문 변수
		bool parkingComplete = false; // 주차 완료를 나타내는 플래그
		bool sensingFlag(false);


		while (!parkingComplete)
		{
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //캘리된 영상 frame

			sideDistance = secondSonic.distance();  //초음파 거리측정.
			waitKey(50);
			backDistance = firstSonic.distance(); //초음파 거리측정.

			cout << "sideDistance : " << sideDistance << endl;
			cout << "backDistance : " << backDistance << endl;


			cout << "caseNum : " << caseNum << endl;

			switch (caseNum)
			{
				//주차 상황 판단전
			case 0:
				cout << "기본 주행 코드" << endl;
				if (sideDistance < 30) // 처음 벽을 만나면 다음 분기로 이동
					caseNum = 1;
				break;

			case 1:
				cout << "벽 처음 감지" << endl;
				if (sideDistance > 45) // 벽을 지나 주차공간을 만나면 다음 분기로 이동
					caseNum = 2;
				break;

			case 2:
				if (!sensingFlag)	//주차공간의 폭을 측정
				{
					sensingFlag = true;
					TickMeter tm;
					tm.start();
				}
				cout << "주차 공간 감지" << endl;
				if (sideDistance < 30) //
				{
					tm.stop();
					cout << "폭 감지 시간 = " << tm.getTimeMilli() << endl;
					DCmotor.stop();
					if (tm.getTimeMilli() > 2500)	//폭 길 경우 -> 수평
					{
						cout << "수평 주차로 판단한다." << endl;
						DCmotor.stop();
						waitKey(500);
						DCmotor.go();
						waitKey(700);
						DCmotor.stop();
						waitKey(500);
						steering.setRatio(90); // 바퀴를 오른쪽으로 돌린 후 후진
						DCmotor.backward(40);
						caseNum = 103;
					}
					else 	//폭 짧음 -> 수직
					{
						cout << "수직 주차로 판단한다." << endl;
						DCmotor.stop();
						waitKey(500);
						DCmotor.backward();
						waitKey(550);
						DCmotor.stop();
						steering.setRatio(20);
						DCmotor.go(40);
						waitKey(1000);
						DCmotor.stop();
						steering.setRatio(80); // 바퀴를 오른쪽으로 돌린 후 후진
						DCmotor.backward(40);
						caseNum = 203;
					}
				}
				break;
				//주차 상황 판단 완료.


				//수평 주차 시작---------------------------------------------
			case 103:
				cout << "수평) 후진 진행 - 1 -" << endl;
				if ((backDistance != 0) && (backDistance < 10) || ((sideDistance != 0) && (sideDistance < 12)))
				{ // 후진 중 어느정도 주차공간에 진입하였으면 다음 분기로 이동
					DCmotor.stop();
					waitKey(500);
					DCmotor.go();
					waitKey(700);
					steering.setRatio(10); // 바퀴를 왼쪽으로 돌린 후 후진
					DCmotor.backward(40);
					caseNum = 104;
				}
				break;
			case 104:
				cout << "수평) 후진 진행 - 2 -" << endl;
				if (((sideDistance != 0) && (sideDistance < 8)) || ((backDistance != 0) && (backDistance < 8)))
				{
					DCmotor.stop(); // 3초 정도 대기, sleep 함수 이용 or clock 함수로 시간 측정하여 이용
					waitKey(3000);
					caseNum = 105;
				}
				break;
			case 105:
				cout << "수평) 주차 완료 및 차량 복귀" << endl;
				DCmotor.go(); // 바퀴 조향은 그대로 탈출
				if (1)
				{ // 주차 분기 탈출 구문으로 차선이 검출되면 주차 분기를 탈출한다.
					waitKey(2000);
					cout << "Detect line and keep going" << endl;
					caseNum = 106;
				}
				break;
				//End 수평 주차---------------------------------------------



				//수직 주차 시작---------------------------------------------
			case 203:
				cout << "수직) 후진 진행 - 1 -" << endl;
				if (sideDistance < 10)
				{ // 후진 중 어느정도 주차공간에 진입하였으면 다음 분기로 이동
					DCmotor.stop();
					steering.setRatio(50); // 바퀴를 왼쪽으로 돌린 후 후진
					DCmotor.backward();
					caseNum = 205;
				}
				break;
			case 205:
				cout << "수직) 후진 진행 - 2 -" << endl;
				if (backDistance < 5)
				{
					DCmotor.stop(); // 3초 정도 대기, sleep 함수 이용 or clock 함수로 시간 측정하여 이용
					waitKey(3000);
					caseNum = 206;
				}
				break;
			case 206:
				DCmotor.go(); // 바퀴 조향은 그대로 탈출
				if (1)
				{ // 주차 분기 탈출 구문으로 차선이 검출되면 주차 분기를 탈출한다.
					waitKey(3000);
					cout << "Detect line and keep going" << endl;
					caseNum = 207;
				}
				break;
				//End 수직주차 case---------------------------------------------


			default:
				parkingComplete = true;
				if (parkingComplete)
					cout << "Parking branck is done" << endl;
				DCmotor.stop();
				break;
			}
			//End Parking mode



			allServoReset(pca);
			cout << "-------------[program finished]-------------" << endl
				<< endl;
			return 0;
		}
