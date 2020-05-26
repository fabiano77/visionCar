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
	UltraSonic firstSonic(28, 27);	// �����ļ��� ��ü
	UltraSonic secondSonic(30, 29); // �����ļ��� ��ü 2(�μ�: ����)
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

	cout << "Mode 3 : Signal detection(����)" << endl;
	cout << "Mode 4 : Driving(����)" << endl;
	cout << "Mode 5 : Parking(����)" << endl;
	cout << "Mode 6 : Rotary(���)" << endl;
	cout << "Mode 7 : Overtaking(�μ�)" << endl;
	cout << "Mode 8 : Tunnel" << endl
		<< endl;

	cout << "Select mode : ";
	int mode;
	cin >> mode;

	if (mode == 1) //Test 1 : Basic test------------------------------------------------------
	{
		steering.setRatio(100); //���� ����
		steering.setRatio(0);	//���� ����
		steering.resetCenter(); //���� �¿�����

		cam_tilt.setRatio(100); //ī�޶� ����
		cam_tilt.setRatio(0);	//ī�޶� ����
		cam_tilt.resetCenter(); //ī�޶� ��������

		cam_pan.setRatio(100); //ī�޶� ����
		cam_pan.setRatio(0);   //ī�޶� ����
		cam_pan.resetCenter(); //ī�޶� �¿�����

		DCmotor.go();  //dc���� ����
		waitKey(1500); //wait 1.5sec

		DCmotor.backward(); //dc���� ����
		waitKey(1500);		//wait 1.5sec

		DCmotor.stop(); //dc���� ����
	}
	//End basic test

	else if (mode == 2) //Test 2 : Manual test------------------------------------------------
	{
		//ManualMode class & basic speed rate
		ManualMode Manual(pca, 25);
		Manual.guide();

		//���η���
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

	else if (mode == 3) //Mode 3 : Signal detection(����) ------------------------------------
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
			else if (detectColorSign.isRedStop(distortedFrame, 1.5)) //������ ǥ���� ����
			{
				cout << "A red stop sign was detected." << '\n';
			}
			else if (detectColorSign.isYellow(distortedFrame, 1.5)) //����� ǥ���� ����
			{
				cout << "A yellow sign was detected." << '\n';
			}
			else if (detectColorSign.isGreenTurnSignal(distortedFrame, 1.0) == 1) //�ʷϻ� ǥ���� ����
			{
				cout << "<----- signal was detected." << '\n';
			}
			else if (detectColorSign.isGreenTurnSignal(distortedFrame, 1.5) == 2) //�ʷϻ� ǥ���� ����
			{
				cout << "-----> signal was detected." << '\n';
			}
			namedWindow("frame", WINDOW_NORMAL);
			imshow("frame", frame);
			resizeWindow("frame", 480, 360);
			moveWindow("frame", 320, 80 + 240);
			if (waitKey(33) == 27)
				break; //���α׷� ���� ESC(�ƽ�Ű�ڵ� = 27)Ű.
		}
	}
	//End Signal detection mode

	else if (mode == 4) //Mode 4 : Driving(����) --------------------------------------------
	{
		//ManualMode class & basic speed rate
		ManualMode Manual(pca, 40);

		//Self-driving class configuration
		Driving_DH DH(true, 1.00);
		bool cornerFlag(false);
		bool manualFlag(false);
		int detectedLineCnt(-1);
		double steerVal(50.0);			//�ʱ� ����(50�� �߽�)
		DH.mappingSet(cornerFlag);		//������� ���ΰ� ����

		bool rotaryFlag(false);

		//���ε��� ����
		while (true)
		{
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR);
			if (false)
			{
			}
			else //��������
			{
				DH.driving(frame, steerVal, detectedLineCnt, rotaryFlag);

				if (!cornerFlag && steerVal == 90 || steerVal == 10)	//�ִ� �� ����Ǹ� cornerFlag ON
				{
					cornerFlag = true;
					DH.mappingSet(cornerFlag);
					cout << "cornerFlag ON" << '\n';
				}
				else if (cornerFlag && detectedLineCnt == 2)			//���� �ΰ� ����Ǹ� cornerFlag OFF
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
				break; //���α׷� ���� ESC(�ƽ�Ű�ڵ� = 27)Ű.
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



	else if (mode == 5) //Mode 5 : Parking(����) ---------------------------------------------
	{
		double sideDistance = 0;	  // ���� �Ÿ����� ��
		double backDistance = 0;	  // �Ĺ� �Ÿ����� ��
		int caseNum = 0;			  // Switch - Case �� ����
		bool parkingComplete = false; // ���� �ϷḦ ��Ÿ���� �÷���
		bool sensingFlag(false);


		while (!parkingComplete)
		{
			videocap >> distortedFrame;
			remap(distortedFrame, frame, map1, map2, INTER_LINEAR); //Ķ���� ���� frame

			sideDistance = secondSonic.distance();  //������ �Ÿ�����.
			waitKey(50);
			backDistance = firstSonic.distance(); //������ �Ÿ�����.

			cout << "sideDistance : " << sideDistance << endl;
			cout << "backDistance : " << backDistance << endl;


			cout << "caseNum : " << caseNum << endl;

			switch (caseNum)
			{
				//���� ��Ȳ �Ǵ���
			case 0:
				cout << "�⺻ ���� �ڵ�" << endl;
				if (sideDistance < 30) // ó�� ���� ������ ���� �б�� �̵�
					caseNum = 1;
				break;

			case 1:
				cout << "�� ó�� ����" << endl;
				if (sideDistance > 45) // ���� ���� ���������� ������ ���� �б�� �̵�
					caseNum = 2;
				break;

			case 2:
				if (!sensingFlag)	//���������� ���� ����
				{
					sensingFlag = true;
					TickMeter tm;
					tm.start();
				}
				cout << "���� ���� ����" << endl;
				if (sideDistance < 30) //
				{
					tm.stop();
					cout << "�� ���� �ð� = " << tm.getTimeMilli() << endl;
					DCmotor.stop();
					if (tm.getTimeMilli() > 2500)	//�� �� ��� -> ����
					{
						cout << "���� ������ �Ǵ��Ѵ�." << endl;
						DCmotor.stop();
						waitKey(500);
						DCmotor.go();
						waitKey(700);
						DCmotor.stop();
						waitKey(500);
						steering.setRatio(90); // ������ ���������� ���� �� ����
						DCmotor.backward(40);
						caseNum = 103;
					}
					else 	//�� ª�� -> ����
					{
						cout << "���� ������ �Ǵ��Ѵ�." << endl;
						DCmotor.stop();
						waitKey(500);
						DCmotor.backward();
						waitKey(550);
						DCmotor.stop();
						steering.setRatio(20);
						DCmotor.go(40);
						waitKey(1000);
						DCmotor.stop();
						steering.setRatio(80); // ������ ���������� ���� �� ����
						DCmotor.backward(40);
						caseNum = 203;
					}
				}
				break;
				//���� ��Ȳ �Ǵ� �Ϸ�.


				//���� ���� ����---------------------------------------------
			case 103:
				cout << "����) ���� ���� - 1 -" << endl;
				if ((backDistance != 0) && (backDistance < 10) || ((sideDistance != 0) && (sideDistance < 12)))
				{ // ���� �� ������� ���������� �����Ͽ����� ���� �б�� �̵�
					DCmotor.stop();
					waitKey(500);
					DCmotor.go();
					waitKey(700);
					steering.setRatio(10); // ������ �������� ���� �� ����
					DCmotor.backward(40);
					caseNum = 104;
				}
				break;
			case 104:
				cout << "����) ���� ���� - 2 -" << endl;
				if (((sideDistance != 0) && (sideDistance < 8)) || ((backDistance != 0) && (backDistance < 8)))
				{
					DCmotor.stop(); // 3�� ���� ���, sleep �Լ� �̿� or clock �Լ��� �ð� �����Ͽ� �̿�
					waitKey(3000);
					caseNum = 105;
				}
				break;
			case 105:
				cout << "����) ���� �Ϸ� �� ���� ����" << endl;
				DCmotor.go(); // ���� ������ �״�� Ż��
				if (1)
				{ // ���� �б� Ż�� �������� ������ ����Ǹ� ���� �б⸦ Ż���Ѵ�.
					waitKey(2000);
					cout << "Detect line and keep going" << endl;
					caseNum = 106;
				}
				break;
				//End ���� ����---------------------------------------------



				//���� ���� ����---------------------------------------------
			case 203:
				cout << "����) ���� ���� - 1 -" << endl;
				if (sideDistance < 10)
				{ // ���� �� ������� ���������� �����Ͽ����� ���� �б�� �̵�
					DCmotor.stop();
					steering.setRatio(50); // ������ �������� ���� �� ����
					DCmotor.backward();
					caseNum = 205;
				}
				break;
			case 205:
				cout << "����) ���� ���� - 2 -" << endl;
				if (backDistance < 5)
				{
					DCmotor.stop(); // 3�� ���� ���, sleep �Լ� �̿� or clock �Լ��� �ð� �����Ͽ� �̿�
					waitKey(3000);
					caseNum = 206;
				}
				break;
			case 206:
				DCmotor.go(); // ���� ������ �״�� Ż��
				if (1)
				{ // ���� �б� Ż�� �������� ������ ����Ǹ� ���� �б⸦ Ż���Ѵ�.
					waitKey(3000);
					cout << "Detect line and keep going" << endl;
					caseNum = 207;
				}
				break;
				//End �������� case---------------------------------------------


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
