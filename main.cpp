#include <iostream>
#include <opencv2/opencv.hpp>

#include "Calibration.h"
#include "CustomPicar.h"
#include "DetectColorSign.h"
#include "Driving_DH.h"
#include "driving_class/SM_drivingAngle.h"

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
		//Self-driving class configuration
		Driving_DH DH(true, 1.00);
		DH.mappingSetSection(0, 0.15, 0.20, 0.30, 0.42, 0.43);
		DH.mappingSetValue(7.0, 7.00, 0.00, -4.0, 0.00, 40.0); //코너구간 조향수준 맵핑값 세팅
		double steerVal(50.0);								   //초기 각도(50이 중심)
		double speedVal(40.0);								   //초기 속도(0~100)

		bool cornerFlag(false);
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
				DH.driving(frame, steerVal, speedVal, speedVal, 0.0, rotaryFlag);

				if (!cornerFlag && (steerVal == 90 || steerVal == 10))
				{
					cornerFlag = true;
					DH.mappingSetSection(0, 0.15, 0.20, 0.30, 0.42, 0.43);
					DH.mappingSetValue(15., 15.0, 10.0, 15.0, 30.0, 30.0);
					cout << "cornerFlag ON" << '\n';
				}
				else if (cornerFlag && steerVal >= 43 && steerVal <= 57)
				{
					cornerFlag = false;
					DH.mappingSetSection(0, 0.15, 0.20, 0.30, 0.42, 0.43);
					DH.mappingSetValue(7.0, 7.00, 0.00, -4.0, 0.00, 40.0);
					cout << "cornerFlag OFF" << '\n';
				}

				steering.setRatio(steerVal);

				//DCmotor.go(speedVal);
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
				//ManualMode class & basic speed rate
				ManualMode Manual(pca, 40);
				Manual.guide();

				//메인루프
				int key(-1);
				while (key != 27) //if not ESC
				{
					videocap >> distortedFrame;
					remap(distortedFrame, frame, map1, map2, INTER_LINEAR);

					DH.driving(frame, steerVal, speedVal, 37.0, 0.0);

					namedWindow("frame", WINDOW_NORMAL);
					imshow("frame", frame);
					resizeWindow("frame", 480, 360);
					moveWindow("frame", 320, 80 + 240);

					key = waitKey(33); //if you not press, return -1
					if (key == 27)
						break;
					else if (key == '0')
						break;
					else if (key != -1)
						Manual.input(key); //movement by keyboard
					rewind(stdin);
				}
			}
		}
	}
	//End Driving mode

	else if (mode == 5) //Mode 5 : Parking(석준) ---------------------------------------------
	{
		double sideDistance = 0;	  // 측면 거리센서 값
		double backDistance = 0;	  // 후방 거리센서 값
		int caseNum = 0;			  // Switch - Case 문 변수
		bool parkingComplete = false; // 주차 완료를 나타내는 플래그

		cout << "Select Mode : " << endl;
		cout << "0 : Parallel Parking" << endl;
		cout << "1 : Vertical Parking" << endl;
		cin >> mode;
		if (mode == 0)
		{ // 평행주차 모드
			while (true)
			{
				videocap >> distortedFrame;
				remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //캘리된 영상 frame

				sideDistance = firstSonic.distance();  //초음파 거리측정.
				backDistance = secondSonic.distance(); //초음파 거리측정.

				while (1)
				{
					switch (caseNum)
					{
					case 0:
						cout << "기본 주행 코드" << endl;
						if ((sideDistance != 0) && (sideDistance < 5)) // 처음 벽을 만나면 다음 분기로 이동
							caseNum++;
						break;
					case 1:
						cout << "벽을 처음 만난 후" << endl;
						if (sideDistance > 10) // 벽을 지나 주차공간을 만나면 다음 분기로 이동
							caseNum++;
						break;
					case 2:
						cout << "주차 공간을 만난 후" << endl;
						if ((sideDistance != 0) && (sideDistance < 5))
						{ // 주차공간을 지나 다시 벽을 만나면 다음 분기로 이동
							DCmotor.stop();
							steering.setRatio(80); // 바퀴를 오른쪽으로 돌린 후 후진
							DCmotor.backward(40);
							caseNum++;
						}
						break;
					case 3:
						cout << "후진 진행 - 1 -" << endl;
						if ((backDistance != 0) && (backDistance < 5))
						{ // 후진 중 어느정도 주차공간에 진입하였으면 다음 분기로 이동
							DCmotor.stop();
							steering.setRatio(20); // 바퀴를 왼쪽으로 돌린 후 후진
							DCmotor.backward(40);
							caseNum++;
						}
						break;
					case 4:
						cout << "후진 진행 - 2 -" << endl;
						if ((sideDistance != 0) && (sideDistance < 2))
						{
							DCmotor.stop(); // 3초 정도 대기, sleep 함수 이용 or clock 함수로 시간 측정하여 이용
							waitKey(3000);
							caseNum++;
						}
						break;
					case 5:
						cout << "주차 완료 및 차량 복귀" << endl;
						DCmotor.go(); // 바퀴 조향은 그대로 탈출
						if (1)
						{ // 주차 분기 탈출 구문으로 차선이 검출되면 주차 분기를 탈출한다.
							waitKey(1500);
							cout << "Detect line and keep going" << endl;
							caseNum++;
						}
					default:
						parkingComplete = true;
						if (parkingComplete)
							cout << "Parking branck is done" << endl;
						DCmotor.stop();
						break;
					}
				}
			}
		}

		else
		{ // 수직주차 모드

			while (true)
			{
				videocap >> distortedFrame;
				remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //캘리된 영상 frame

				sideDistance = firstSonic.distance();  //초음파 거리측정.
				backDistance = secondSonic.distance(); //초음파 거리측정.

				while (1)
				{
					switch (caseNum)
					{
					case 0:
						cout << "기본 주행 코드" << endl;
						if ((sideDistance != 0) && (sideDistance < 5)) // 처음 벽을 만나면 다음 분기로 이동
							caseNum++;
						break;
					case 1:
						cout << "벽을 처음 만난 후" << endl;
						if (sideDistance > 10) // 벽을 지나 주차공간을 만나면 다음 분기로 이동
							caseNum++;
						break;
					case 2:
						cout << "주차 공간을 만난 후" << endl;
						if ((sideDistance != 0) && (sideDistance < 5))
						{ // 주차공간을 지나 다시 벽을 만나면 다음 분기로 이동
							DCmotor.stop();
							steering.setRatio(90); // 바퀴를 오른쪽으로 돌린 후 후진
							DCmotor.backward(40);
							caseNum++;
						}
						break;
					case 3:
						cout << "후진 진행 - 1 -" << endl;
						if ((backDistance != 0) && (backDistance < 3))
						{ // 후진 중 어느정도 주차공간에 진입하였으면 다음 분기로 이동
							DCmotor.stop();
							steering.setRatio(50); // 바퀴를 왼쪽으로 돌린 후 후진
							DCmotor.go();
							waitKey(1000); // 조금 앞으로 차를 뺀다.
							DCmotor.stop();
							steering.setRatio(25);
							DCmotor.backward();
							caseNum++;
						}
						break;
					case 4:
						cout << "후진 진행 - 2 -" << endl;
						if ((sideDistance != 0) && (sideDistance < 2))
						{
							DCmotor.stop(); // 3초 정도 대기, sleep 함수 이용 or clock 함수로 시간 측정하여 이용
							steering.setRatio(50);
							DCmotor.backward();
							caseNum++;
						}
						break;
					case 5:
						cout << "후진 진행 - 2 -" << endl;
						if ((backDistance != 0) && (backDistance < 1.5))
						{
							DCmotor.stop(); // 3초 정도 대기, sleep 함수 이용 or clock 함수로 시간 측정하여 이용
							waitKey(3000);
							caseNum++;
						}
						break;
					case 6:
						cout << "주차 완료 및 차량 복귀" << endl;
						DCmotor.go(); // 바퀴 조향은 그대로 탈출
						if (1)
						{ // 주차 분기 탈출 구문으로 차선이 검출되면 주차 분기를 탈출한다.
							waitKey(1500);
							cout << "Detect line and keep going" << endl;
							caseNum++;
						}
					default:
						parkingComplete = true;
						if (parkingComplete)
							cout << "Parking branck is done" << endl;
						DCmotor.stop();
						break;
					}
				}
			}
		}
	}
	//End Parking mode

		else if (mode == 6)//Mode 6 : Rotary(상민) ----------------------------------------------
	{
	//Self-driving class configuration
	Driving_DH DH(true, 1.00);
	DH.mappingSetSection(0, 0.15, 0.20, 0.30, 0.42, 0.43);
	DH.mappingSetValue(7.0, 7.00, 0.00, -4.0, 0.00, 40.0);	//코너구간 조향수준 맵핑값 세팅
	double steerVal(50.0);	//초기 각도(50이 중심)
	double speedVal(40.0);	//초기 속도(0~100)
	double speedVal_rotary(20.0);
	double Distance;	//거리값

	bool cornerFlag(false);
	bool rotaryFlag(true);
	int rotaryDelayFlag = 0;
	RoundAbout Rotary;
	//메인동작 루프
	while (true)
	{
		videocap >> distortedFrame;
		remap(distortedFrame, frame, map1, map2, INTER_LINEAR);

		Distance = firstSonic.distance();	//초음파 거리측정.

		cout << "distance = " << Distance << endl;	//거리출력

		if (!Rotary.isStop(Distance)) // 회전 교차로 진입 (흰색 차선에서 멈춰있다고 가정)
		{
			if (Rotary.isDelay(Distance)) { // 앞의 차량과 가까워졌을 시 정지
				DCmotor.stop();
			}
			else if (Rotary.isDelay(Distance)) { // 
				DH.driving(frame, steerVal, speedVal, speedVal, 0.0, rotaryFlag);

				if (!cornerFlag && (steerVal == 90 || steerVal == 10))
				{
					cornerFlag = true;
					DH.mappingSetSection(0, 0.15, 0.20, 0.30, 0.42, 0.43);
					DH.mappingSetValue(15., 15.0, 10.0, 15.0, 40.0, 40.0);
					cout << "cornerFlag ON" << '\n';
				}
				else if (cornerFlag && steerVal >= 43 && steerVal <= 57)
				{
					cornerFlag = false;
					DH.mappingSetSection(0, 0.15, 0.20, 0.30, 0.42, 0.43);
					DH.mappingSetValue(7.0, 7.00, 0.00, -4.0, 0.00, 40.0);
					cout << "cornerFlag OFF" << '\n';
				}

				steering.setRatio(steerVal);

				DCmotor.go(speedVal_rotary);
			}
		}
		
		namedWindow("frame", WINDOW_NORMAL);
		imshow("frame", frame);
		resizeWindow("frame", 480, 360);
		moveWindow("frame", 320, 80 + 240);

		int key = waitKey(10);
		if (key == 27) break;	//프로그램 종료 ESC(아스키코드 = 27)키.
		/*else if (key == 'w') DCmotor.go(37);
		else if (key == 'x') DCmotor.backward(40);
		else if (key == 's') DCmotor.stop();
		else if (key == '0')
		{
			//ManualMode class & basic speed rate
			ManualMode Manual(pca, 40);
			Manual.guide();

			//메인루프
			int key(-1);
			while (key != 27)//if not ESC
			{
				videocap >> distortedFrame;
				remap(distortedFrame, frame, map1, map2, INTER_LINEAR);

				DH.driving(frame, steerVal, speedVal, 37.0, 0.0);

				namedWindow("frame", WINDOW_NORMAL);
				imshow("frame", frame);
				resizeWindow("frame", 480, 360);
				moveWindow("frame", 320, 80 + 240);

				key = waitKey(33);//if you not press, return -1
				if (key == 27) break;
				else if (key == '0') break;
				else if (key != -1) Manual.input(key);//movement by keyboard
				rewind(stdin);
			}
		}*/
	}



	/*double Distance;	//거리값

	while (true)
	{
		videocap >> distortedFrame;
		remap(distortedFrame, frame, map1, map2, INTER_LINEAR);	//캘리된 영상 frame

		Distance = firstSonic.distance();	//초음파 거리측정.

		cout << "distance = " << Distance << endl;	//거리출력

		steering.setRatio(50);	//바퀴조향
		DCmotor.go();			//dc모터 전진 argument로 속도전달가능
		DCmotor.backward();		//dc모터 후진 argument로 속도전달가능
		DCmotor.stop();			//정지

		namedWindow("frame", WINDOW_NORMAL);
		imshow("frame", frame);
		resizeWindow("frame", 480, 360);
		moveWindow("frame", 0, 0);

		if (waitKey(33) == 27) break;	//프로그램 종료 ESC키.
	}*/
	}
	//End Rotary mode

	else if (mode == 7) //Mode 7 : Overtaking(민수) ------------------------------------------
	{
		Driving_DH DH(true, 1.00);
		DH.mappingSetSection(0, 0.15, 0.20, 0.30, 0.42, 0.43);
		DH.mappingSetValue(7.0, 7.00, 0.00, -4.0, 0.00, 40.0); //코너구간 조향수준 맵핑값 세팅
		double steerVal(50.0);								   //초기 각도(50이 중심)
		double speedVal(40.0);								   //초기 속도(0~100)
		bool rotaryFlag(false);
		double Distance_first; //거리값
		double Distance_second;
		const double MAX_ULTRASONIC = 30; //30CM 최대
		const double MIN_ULTRASONIC = 5;  //4CM 최소
		bool shortDistanceFlag = false;	  //너무 가까운지에 대한 판단
		bool overtakingFlag = false;	  //추월상황 판단
		int returnFlag = 0;
		const int MAX_returnFlag = 5; // 아무생각 없이 직진하지 말라는 방지 flag
		//초음파 센서 하나인 경우
		while (true)
		{
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //캘리된 영상 frame

			Distance_first = firstSonic.distance();	  //초음파 거리측정 1번센서.
			Distance_second = secondSonic.distance(); //초음파 거리측정 2번센서.
			if (overtakingFlag == false)			  //추월상황이 아닐때,
			{
				rotaryFlag = false;
				if (Distance_first > MAX_ULTRASONIC) //거리가 멀때
				{
					DH.driving(frame, steerVal, speedVal, speedVal, 0.0, rotaryFlag);
					shortDistanceFlag = false;
					DCmotor.go();
				}
				else if (Distance_first < MIN_ULTRASONIC) //거리가 가까울 때
				{
					cout << "거리가 가까워서 후진" << endl;
					DCmotor.backward();
					shortDistanceFlag = true;
				}
				else if (shortDistanceFlag == false) //추월상황 탐지 거리일 때(처음 진입의 경우)
				{
					cout << "추월 시나리오 진입" << endl;
					overtakingFlag = true;
				}
				else if (shortDistanceFlag == true) //추월상황 탐지 거리일 때(너무 가까이에 왔었던 경우)
				{
					overtakingFlag = false;
				}
			}
			else //추월상황일 때
			{
				if (Distance_first < MAX_ULTRASONIC && Distance_first > MIN_ULTRASONIC && Distance_second > MAX_ULTRASONIC) //추월상황 시작시, 직진의 물체탐지
				{
					cout << "좌회전 추월" << endl;
					steerVal = 10; //먼저 좌회전
					returnFlag = MAX_returnFlag;
				}
				else if (Distance_first > MAX_ULTRASONIC && Distance_second < MAX_ULTRASONIC) //추월 상황중 차량을 지나쳐갈 때 and 차량을 지나치고 복귀중 재탐색시
				{
					rotaryFlag = true;
					DH.driving(frame, steerVal, speedVal, speedVal, 0.0, rotaryFlag);
				}
				else if (Distance_first > MAX_ULTRASONIC && Distance_second > MAX_ULTRASONIC) //추월 상황 종료후 복귀 신호
				{
					rotaryFlag = false;
					steerVal = 90;
					//예비 상황 혹시 차량을 지나쳐가는 루프에 들어오지 못하는 경우 방지
					if (returnFlag < MAX_returnFlag)
					{
						returnFlag++;
					}
					if (returnFlag >= MAX_returnFlag)
					{
						rotaryFlag = true;
						DH.driving(frame, steerVal, speedVal, speedVal, 0.0, rotaryFlag);
					}
				}

				//차량 복귀 신호가 문제일 수 도 있음.
			}

			cout << "distance = " << Distance_first << endl; //거리출력
			//0.3초당 1frame 처리
			// steering.setRatio(50);	//바퀴조향
			// DCmotor.go();			//dc모터 전진 argument로 속도전달가능
			// DCmotor.backward();		//dc모터 후진 argument로 속도전달가능
			// DCmotor.stop();			//정지

			if (shortDistanceFlag == true) //사물과 거리가 너무 가까울 때
			{
				DCmotor.backward();
			}
			else if (shortDistanceFlag == false &&)

				steering.setRatio(steerVal);
			imshow("frame", frame);
			if (waitKey(33) == 27)
				break; //프로그램 종료 ESC키.
		}
	}
	//End Overtaking mode

	else if (mode == 8) //Mode 8 : Tunnel ----------------------------------------------------
	{
		double Distance; //거리값

		while (true)
		{
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //캘리된 영상 frame

			Distance = firstSonic.distance(); //초음파 거리측정.

			cout << "distance = " << Distance << endl; //거리출력

			steering.setRatio(50); //바퀴조향
			DCmotor.go();		   //dc모터 전진 argument로 속도전달가능
			DCmotor.backward();	   //dc모터 후진 argument로 속도전달가능
			DCmotor.stop();		   //정지

			imshow("frame", frame);
			if (waitKey(33) == 27)
				break; //프로그램 종료 ESC키.
		}
	}
	//End Tunnel mode

	allServoReset(pca);
	cout << "-------------[program finished]-------------" << endl
		 << endl;
	return 0;
}
