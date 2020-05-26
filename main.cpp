#include <iostream>
#include <opencv2/opencv.hpp>

#include "Calibration.h"
#include "CustomPicar.h"
#include "DetectColorSign.h"
#include "Driving_DH.h"
#include "SM_drivingAngle.h"

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
	UltraSonic secondSonic(26, 25); // 초음파센서 객체 2(민수: 우측) 
	PicarLED whiteLed(24);
	PicarLED rightLed(23);
	PicarLED leftLed(22);
	whiteLed.on();
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
	whiteLed.off();
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
	whiteLed.on();
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
	cout << "Mode 8 : Tunnel(대희)" << endl
		<< endl;

	cout << "Select mode : ";
	int mode;
	cin >> mode;
	whiteLed.off();

	if (mode == 1) //Test 1 : Basic test------------------------------------------------------
	{
		//steering.setRatio(100); //바퀴 우측
		//steering.setRatio(0);	//바퀴 좌측
		//steering.resetCenter(); //바퀴 좌우정렬

		//cam_tilt.setRatio(100); //카메라 상향
		//cam_tilt.setRatio(0);	//카메라 하향
		//cam_tilt.resetCenter(); //카메라 상하정렬

		//cam_pan.setRatio(100); //카메라 우향
		//cam_pan.setRatio(0);   //카메라 좌향
		//cam_pan.resetCenter(); //카메라 좌우정렬

		//DCmotor.go();  //dc모터 전진
		//waitKey(1500); //wait 1.5sec

		//DCmotor.backward(); //dc모터 후진
		//waitKey(1500);		//wait 1.5sec

		//DCmotor.stop(); //dc모터 멈춤
		while (true)
		{
			cout << "1번센서 거리 = " << firstSonic.distance() << endl;
			cout << "2번센서 거리 = " << secondSonic.distance() << endl;
			waitKey(33);
		}
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
		int flicker(4);

		while (true)
		{
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR);
			if (!flicker--)
				flicker = 4;

			whiteLed.off();
			rightLed.off();
			leftLed.off();

			if (detectColorSign.priorityStop(frame, 1.5))
			{
				if ((flicker % 2) == 1)
				{
					whiteLed.on();
				}
				else
				{
					leftLed.on();
					rightLed.on();
				}
				cout << "A priority stop signal was detected." << '\n';
			}
			else if (detectColorSign.isRedStop(frame, 1.5)) //빨간색 표지판 감지
			{
				if ((flicker % 3) == 1)
				{
					whiteLed.on();
					leftLed.on();
					rightLed.on();
				}
				cout << "A red stop sign was detected." << '\n';
			}
			else if (detectColorSign.isYellow(frame, 1.5)) //노란색 표지판 감지
			{
				if ((flicker % 3) == 1)
				{
					leftLed.on();
					rightLed.on();
				}
				cout << "A yellow sign was detected." << '\n';
			}
			else if (detectColorSign.isGreenTurnSignal(frame, 1.0) == 1) //초록색 표지판 감지
			{
				if ((flicker % 3) == 1) leftLed.on();
				cout << "<----- signal was detected." << '\n';
			}
			else if (detectColorSign.isGreenTurnSignal(frame, 1.5) == 2) //초록색 표지판 감지
			{
				if ((flicker % 3) == 1) rightLed.on();
				cout << "-----> signal was detected." << '\n';
			}
			else
			{
				whiteLed.off();
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
		int flicker(4);

		//color detecting class ganerate
		DetectColorSign detectColorSign(true);
		DetectColorSign startCheck(true);
		bool waitingFlag(true);

		//메인동작 루프
		while (true)
		{
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR);
			if (waitingFlag)
			{
				if (detectColorSign.waitingCheck(frame, 20))
					flicker = 2;
				else if (startCheck.waitingCheck(frame, 15))
					flicker = 4;
				else
					waitingFlag = false;
			}
			else //정상주행
			{
				DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);

				if (!cornerFlag && (steerVal == 90 || steerVal == 10))	//최대 각 검출되면 cornerFlag ON
				{
					cornerFlag = true;
					DH.mappingSet(cornerFlag);
					cout << "cornerFlag ON" << '\n';
				}
				//else if (cornerFlag && detectedLineCnt == 2)				//직선 두개 검출되면 cornerFlag OFF
				else if (cornerFlag && (steerVal >= 40 && steerVal <= 60))	//각도가 좁아지면 cornerFlag OFF
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

			// LED 관리코드
			rightLed.off();
			leftLed.off();
			if (steerVal > 60)
			{
				rightLed.on();
				whiteLed.off();
			}
			else if (steerVal < 40)
			{
				leftLed.on();
				whiteLed.off();
			}
			else
			{
				if (!flicker)
					flicker = 4;
				if (2 < flicker--)
					whiteLed.on();
				else
					whiteLed.off();
			}

			//키입력 관리코드 ( 0 = 수동모드, w = 전진, x = 후진, s = 멈춤, ESC 탈출 )
			int key = waitKey(33);
			if (key == 27)
				break;
			else if (key == '0')
			{
				manualFlag = !manualFlag;
				Manual.guide();
				cout << "Auto driving start" << endl;
			}
			else if (manualFlag && key != -1)
			{
				Manual.input(key);
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
		TickMeter tm;

		DCmotor.go(40);

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
					tm.start();
				}
				cout << "주차 공간 감지" << endl;
				if (sideDistance < 30) //
				{
					tm.stop();
					cout << "폭 감지 시간 = " << tm.getTimeMilli() << endl;
					DCmotor.stop();
					if (tm.getTimeMilli() > 1200)	//폭 길 경우 -> 수평
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
				if ((backDistance < 18) || (sideDistance < 12))
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
				if ((sideDistance < 10) || (backDistance < 10))
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
				if (backDistance < 6)
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
			waitKey(150);
		}
	}
	//End Parking mode



	else if (mode == 6)//Mode 6 : Rotary(상민) ----------------------------------------------
	{
		//Self-driving class configuration
		Driving_DH DH(true, 1.00);
		bool cornerFlag(false);
		int detectedLineCnt(-1);
		DH.mappingSet(cornerFlag);	//조향수준 맵핑값 세팅
		double steerVal(50.0);	//초기 각도(50이 중심)
		double speedVal(40.0);	//초기 속도(0~100)
		double speedVal_rotary(20.0);
		double Distance;	//거리값

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

					DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);

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
		int choosemodeNum = 0;
		cout << "MS'mode 1: Original , 2: 분기점 추가, 3:시간 기반 4: add 대희형 5:time기반" << endl;
		cin >> choosemodeNum;
		Driving_DH DH(true, 1.00);
		bool cornerFlag(false);
		int detectedLineCnt(-1);
		DH.mappingSet(cornerFlag); //조향수준 맵핑값 세팅
		double steerVal(50.0);								   //초기 각도(50이 중심)
		double speedVal(40.0);								   //초기 속도(0~100)
		bool rotaryFlag(false);
		double Distance_first; //거리값
		double Distance_second;
		const double MAX_ULTRASONIC = 35; //30CM 최대
		const double MIN_ULTRASONIC = 5;  //4CM 최소

		//초음파 센서 하나인 경우
		if (choosemodeNum == 1) {
			bool shortDistanceFlag = false;	  //너무 가까운지에 대한 판단
			bool overtakingFlag = false;	  //추월상황 판단
			int returnFlag = 0;
			const int MAX_returnFlag = 5; // 아무생각 없이 직진하지 말라는 방지 flag
			bool endFlag = false;//상황 리턴시 혼돈방지 flag
			bool startFlag = true;//시작할 수 있는경우 true로 함
			int afterOvertakingFlag = 0;
			while (true)
			{

				videocap >> distortedFrame;
				remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //캘리된 영상 frame

				Distance_first = firstSonic.distance();	  //초음파 거리측정 1번센서.
				Distance_second = secondSonic.distance(); //초음파 거리측정 2번센서.
				if (overtakingFlag == false)			  //추월상황이 아닐때,
				{
					startFlag = true;
					endFlag = false;
					rotaryFlag = false;
					if (Distance_first > MAX_ULTRASONIC) //거리가 멀때
					{
						DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);
						shortDistanceFlag = false;
						DCmotor.go();
						cout << "추월상황 미탐지" << endl;
					}
					else if (Distance_first < MIN_ULTRASONIC) //거리가 가까울 때
					{
						cout << "거리가 가까워서 후진" << endl;
						DCmotor.backward();
						shortDistanceFlag = true;
					}
					else if (Distance_first < MAX_ULTRASONIC && shortDistanceFlag == false) //추월상황 탐지 거리일 때(처음 진입의 경우)
					{
						cout << "추월 시나리오 진입" << endl;
						overtakingFlag = true;
					}
					else if (Distance_first < MAX_ULTRASONIC && shortDistanceFlag == true) //추월상황 탐지 거리일 때(너무 가까이에 왔었던 경우)
					{
						overtakingFlag = false;
					}
				}
				else //추월상황일 때
				{
					if (Distance_first < MAX_ULTRASONIC && Distance_first > MIN_ULTRASONIC && Distance_second > MAX_ULTRASONIC) //추월상황 시작시, 직진의 물체탐지
					{
						cout << "좌회전 추월" << endl;
						steerVal = 20; //먼저 좌회전
						returnFlag = MAX_returnFlag;
					}
					else if (Distance_first > MAX_ULTRASONIC && Distance_second < MAX_ULTRASONIC && endFlag == false && startFlag == true) //추월 상황중 차량을 지나쳐갈 때
					{
						cout << "추월중" << endl;
						rotaryFlag = true;
						DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);
						if (afterOvertakingFlag >= MAX_returnFlag) { //추월상태 일정이상 유지수 변경 가능
							startFlag = false;
							steerVal = 80;
						}
						else if (afterOvertakingFlag <= MAX_returnFlag && afterOvertakingFlag > 0) {//추월상태 유지
							afterOvertakingFlag++;
						}
						else {//오류난 경우
							afterOvertakingFlag = 0;
						}
					}
					else if (Distance_first > MAX_ULTRASONIC && Distance_second < MAX_ULTRASONIC && endFlag == true && startFlag == false) // 차량을 지나치고 복귀중 재탐색시
					{
						cout << "복귀 시도중" << endl;
						rotaryFlag = false;
						steerVal = 50;
					}
					else if (Distance_first > MAX_ULTRASONIC && Distance_second > MAX_ULTRASONIC && startFlag == false) //추월 상황 종료후 복귀 신호
					{
						cout << " 복귀 중 " << endl;
						endFlag = true;
						rotaryFlag = false;
						steerVal = 80;
						//예비 상황 혹시 차량을 지나쳐가는 루프에 들어오지 못하는 경우 방지
						if (returnFlag < MAX_returnFlag)
						{
							returnFlag++;
						}
						else if (returnFlag >= MAX_returnFlag)//일정 이상시 복귀
						{
							overtakingFlag = false; //복귀 신호로 전환
							DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);
							returnFlag = 0;
							endFlag = false;
						}
					}

					//차량 복귀 신호가 문제일 수 도 있음.
				}

				steering.setRatio(steerVal);
				if (waitKey(33) == 27) {
					break; //프로그램 종료 ESC키.
				}
			}
		}

		else if (choosemodeNum == 2) {
			bool shortDistanceFlag = false;	  //너무 가까운지에 대한 판단
			bool overtakingFlag = false;	  //추월상황 판단
			int startOvertakingFlag = 0; //추월 시나리오 전환 flag MAX_flag넘으면 시작
			int endOvertakingFlag = 0;
			const int MAX_Flag = 10; // 아무생각 없이 직진하지 말라는 방지 flag
			bool endFlag = false;//상황 리턴시 혼돈방지 flag
			bool startFlag = true;//시작할 수 있는경우 true로 함
			bool frontOvertakingFlag = false;
			bool rearOvertakingFlag = false;
			while (true)
			{
				videocap >> distortedFrame;
				remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //캘리된 영상 frame

				Distance_first = firstSonic.distance();	  //초음파 거리측정 1번센서.
				Distance_second = secondSonic.distance(); //초음파 거리측정 2번센서.
				if (overtakingFlag == false)			  //추월상황이 아닐때,
				{
					startFlag = true;
					endFlag = false;
					rotaryFlag = false;
					if (Distance_first > MAX_ULTRASONIC) //거리가 멀때
					{
						DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);
						shortDistanceFlag = false;
						DCmotor.go();
						cout << "추월상황 미탐지" << endl;
					}
					else if (Distance_first < MIN_ULTRASONIC) //거리가 가까울 때
					{
						cout << "거리가 가까워서 후진" << endl;
						DCmotor.backward();
						shortDistanceFlag = true;
					}
					else if (Distance_first < MAX_ULTRASONIC && shortDistanceFlag == false || startOvertakingFlag >= MAX_Flag) //추월상황 탐지 거리일 때(처음 진입의 경우)
					{
						cout << "추월 시나리오 시작" << endl;
						if (startOvertakingFlag < MAX_Flag) {
							startOvertakingFlag++;
							steerVal = 10;
						}
						else {
							overtakingFlag = true;
							startOvertakingFlag = 0;
							frontOvertakingFlag = true;
						}
					}
					else if (Distance_first < MAX_ULTRASONIC && shortDistanceFlag == true) //추월상황 탐지 거리일 때(너무 가까이에 왔었던 경우)
					{
						overtakingFlag = false;
					}
				}
				else //추월상황일 때
				{


					if (frontOvertakingFlag == true && rearOvertakingFlag == false && Distance_first < MAX_ULTRASONIC) //추월 시작 부분
					{
						cout << "추월 시작부분" << endl;
						rotaryFlag = true;
						steerVal = 80 - 10;
					}
					else if (frontOvertakingFlag == true && rearOvertakingFlag == false && Distance_first > MAX_ULTRASONIC) //추월 시작이후 각도 조정 부분
					{
						cout << "추월 시작 이후 각도 조정부분" << endl;
						DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);
					}
					else if (Distance_second < MAX_ULTRASONIC)//추월중인 부분
					{
						cout << "추월 중 부분" << endl;
						DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);
						frontOvertakingFlag = false;
					}
					else if (Distance_second > MAX_ULTRASONIC && Distance_first > MAX_ULTRASONIC && frontOvertakingFlag == false) //추월을 지난 부분
					{
						cout << "추월 종료부분" << endl;
						rearOvertakingFlag = true;//추월 종료 시도 부분
					}
					else if (rearOvertakingFlag == true && frontOvertakingFlag == false)//추월 끝나고 복귀 시도 코드
					{
						cout << "추월 종료후 복귀 시도 부분" << endl;
						if (endOvertakingFlag >= MAX_Flag) {
							rotaryFlag = false;
							overtakingFlag = true;
							cout << "추월 종료" << endl;
						}
						else {
							endOvertakingFlag++;
							steerVal = 80;
						}
					}
				}
				steering.setRatio(steerVal);
				if (waitKey(33) == 27) {
					break; //프로그램 종료 ESC키.
				}
			}

		}

		else if (choosemodeNum == 3) {

			int switchCase = 0;//0은 기본주행
			int holdFlag = 0;//상태유지 flag
			while (true)
			{
				DCmotor.go();
				videocap >> distortedFrame;
				remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //캘리된 영상 frame

				Distance_first = firstSonic.distance();	  //초음파 거리측정 1번센서.
				Distance_second = secondSonic.distance(); //초음파 거리측정 2번센서.

				bool overtakingFlag = true;	  //추월상황 판단
				const int MAX_holdFlag = 10;

				switch (switchCase) {
				case 0:
					cout << "직진중" << endl;
					DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);
					if (Distance_first < MAX_ULTRASONIC) {
						switchCase = 1;//회전부분으로 이동
					}
					break;

				case 1: //좌회전 중
					cout << "추월 시작 및 좌회전 중" << endl;
					steerVal = 10;
					if (holdFlag >= MAX_holdFlag) {
						holdFlag = 0;
						switchCase = 2;
					}
					else {
						holdFlag++;
					}
					break;
				case 2: //각도 다시 변환
					cout << "각도 조정중" << endl;
					steerVal = 90;//다시 직진으로 만들자
					if (Distance_second < MAX_ULTRASONIC)
					{
						if (holdFlag >= MAX_holdFlag) {
							holdFlag = 0;
							switchCase = 3;
						}
						else {
							holdFlag++;
						}
					}
					break;
				case 3:
					cout << "추월중 직진중" << endl;
					steerVal = 50;
					if (Distance_second > MAX_ULTRASONIC) { switchCase = 4; }

					if (holdFlag >= 3 * MAX_holdFlag) {
						holdFlag = 0;
						switchCase = 4;
					}
					else {
						holdFlag++;
					}
					break;
				case 4:
					steerVal = 90;
					cout << "추월 후 복귀중" << endl;
					if (Distance_second < MAX_ULTRASONIC) //오른쪽 탐지되면 원래대로 전환
					{
						switchCase = 0;
					}
					else if (holdFlag >= MAX_holdFlag) {
						holdFlag = 0;
						switchCase = 0;
					}
					else {
						holdFlag++;
					}
					break;
				}

				steering.setRatio(steerVal);
				if (waitKey(33) == 27) {
					break; //프로그램 종료 ESC키.
				}
			}

		}
		else if (choosemodeNum == 4) {

			int switchCase = 0;//0은 기본주행
			int holdFlag = 0;//상태유지 flag
			while (true)
			{
				DCmotor.go();
				videocap >> distortedFrame;
				remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //캘리된 영상 frame

				Distance_first = firstSonic.distance();	  //초음파 거리측정 1번센서.
				Distance_second = secondSonic.distance(); //초음파 거리측정 2번센서.

				bool overtakingFlag = true;	  //추월상황 판단
				const int MAX_holdFlag = 10;

				switch (switchCase) {
				case 0:
					cout << "직진중" << endl;
					DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);
					if (Distance_first < MAX_ULTRASONIC) {
						switchCase = 1;//회전부분으로 이동
					}
					break;

				case 1: //좌회전 중
					cout << "추월 시작 및 좌회전 중" << endl;
					steerVal = 10;
					if (holdFlag >= MAX_holdFlag) {
						holdFlag = 0;
						switchCase = 2;
					}
					else {
						holdFlag++;
					}
					break;
				case 2: //각도 다시 변환
					cout << "각도 조정중 및 직진상황" << endl;
					DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);
					if (Distance_second < MAX_ULTRASONIC)
					{
						if (holdFlag >= MAX_holdFlag) {
							holdFlag = 0;
							switchCase = 3;
						}
						else {
							holdFlag++;
						}
					}
					else if (Distance_second > MAX_ULTRASONIC) { switchCase = 4; }
					break;
				case 3:
					cout << "추월중 직진중" << endl;
					steerVal = 50;


					if (holdFlag >= 3 * MAX_holdFlag) {
						holdFlag = 0;
						switchCase = 4;
					}
					else {
						holdFlag++;
					}
					break;
				case 4:
					steerVal = 90;
					cout << "추월 후 복귀중" << endl;
					if (Distance_second < MAX_ULTRASONIC) //오른쪽 탐지되면 원래대로 전환
					{
						switchCase = 0;
					}
					else if (holdFlag >= MAX_holdFlag) {
						holdFlag = 0;
						switchCase = 0;
					}
					else {
						holdFlag++;
					}
					break;
				}

				steering.setRatio(steerVal);
				if (waitKey(33) == 27) {
					break; //프로그램 종료 ESC키.
				}
			}

		}
		else if (choosemodeNum == 5) {

		int switchCase = 0;//0은 기본주행
		while (true)
		{
			DCmotor.go();
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //캘리된 영상 frame

			Distance_first = firstSonic.distance();	  //초음파 거리측정 1번센서.
			Distance_second = secondSonic.distance(); //초음파 거리측정 2번센서.

			bool overtakingFlag = true;	  //추월상황 판단
			const int MAX_holdFlag = 10;

			switch (switchCase) {
			case 0:
				cout << "직진중" << endl;
				DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);
				if (Distance_first < MAX_ULTRASONIC) {
					switchCase = 1;//회전부분으로 이동
				}
				break;

			case 1: //좌회전 중
				cout << "추월 시작 및 좌회전 중" << endl;
				steerVal = 10;
				waitKey(1500);
				switchCase = 2;
				
				break;
			case 2:
				steerVal = 90;
				waitKey(1200);
			case 3: //각도 다시 변환
				cout << "각도 조정중 및 직진상황" << endl;
				DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);
				while (Distance_second < MAX_ULTRASONIC) {
					waitKey(200);
					if (Distance_second > MAX_ULTRASONIC) {
						switchCase = 4;
					}
				}
				break;
			case 4:
				steerVal = 80;
				cout << "추월 후 복귀중" << endl;
				waitKey(1500);
				switchCase = 5;
				break;
			}
			steering.setRatio(steerVal);
			if (waitKey(33) == 27) {
				break; //프로그램 종료 ESC키.
			}
		}

		}
		//0.3초당 1frame 처리
		// steering.setRatio(50);	//바퀴조향
		// DCmotor.go();			//dc모터 전진 argument로 속도전달가능
		// DCmotor.backward();		//dc모터 후진 argument로 속도전달가능
		// DCmotor.stop();			//정지
	}
	//End Overtaking mode

	else if (mode == 8) //Mode 8 : Tunnel(대희) ----------------------------------------------------
	{
		double leftDistance; //좌측 거리값
		double rightDistance; //우측 거리값

		CheckStart cs;
		bool check_tunnel;
		while (true)
		{
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //캘리된 영상 frame

			check_tunnel = cs.isTunnel(frame, 60);

			if (check_tunnel) // 터널 입장
			{
				whiteLed.on();	//전조등 킨다.
				//leftDistance = firstSonic.distance();	//좌측 거리측정.
				//rightDistance = secondSonic.distance(); //우측 거리측정.
				//double longDistance = (leftDistance > rightDistance) ? leftDistance : rightDistance;
				//double shortDistance = (leftDistance > rightDistance) ? rightDistance : leftDistance;
				//double angle = (longDistance / shortDistance) - 1;	//대략 0~0.5사이
				//angle *= 100;	//대략0~50사이
				//if (angle > 15) angle = 15;	//최대 15으로 제한.
				//if (leftDistance > rightDistance)
				//	angle = 50 + angle;
				//else
				//	angle = 50 - angle;
				//steering.setRatio(angle);
				//DCmotor.go(30);
			}
			else	//기본주행
			{
				cs.GetFlag_tunnel();
				whiteLed.off();
				//steering.setRatio(50);
				//DCmotor.go(40);
			}

			imshow("frame", frame);
			if (waitKey(33) == 27)
				break; //프로그램 종료 ESC키.
		}
	}
	//End Tunnel mode

	else if (mode == 9) //Mode 9 : Tunnel(대희) ----------------------------------------------------
	{


	}
	//End Tunnel mode

	whiteLed.off();
	rightLed.off();
	leftLed.off();
	allServoReset(pca);
	cout << "-------------[program finished]-------------" << endl
		<< endl;
	return 0;
}
