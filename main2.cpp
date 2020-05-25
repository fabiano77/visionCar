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
		//Self-driving class configuration
		Driving_DH DH(true, 1.00);
		DH.mappingSetSection(0, 0.15, 0.20, 0.30, 0.42, 0.43);
		DH.mappingSetValue(7.0, 7.00, 0.00, -4.0, 0.00, 40.0); //�ڳʱ��� ������� ���ΰ� ����
		double steerVal(50.0);								   //�ʱ� ����(50�� �߽�)
		double speedVal(40.0);								   //�ʱ� �ӵ�(0~100)

		bool cornerFlag(false);
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
				break; //���α׷� ���� ESC(�ƽ�Ű�ڵ� = 27)Ű.
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

				//���η���
				int key(-1);
				while (key != 27) //if not ESC
				{
					videocap >> distortedFrame;
					remap(distortedFrame, frame, map1, map2, INTER_LINEAR);

					DH.driving(frame, steerVal, speedVal, 37.0, 0.0, 0, rotaryFlag);

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


	allServoReset(pca);
	cout << "-------------[program finished]-------------" << endl
		<< endl;
	return 0;
}
