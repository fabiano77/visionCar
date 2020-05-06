#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include "CustomPicar.h"
#include "Driving_DH.h"
#include "DetectColorSign.h"
#include "Calibration.h"

//cpp�� �߰��غ��� ���� ��Ѱ�

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
		steering.setRatio(100);			//���� ����
		steering.setRatio(0);			//���� ����
		steering.resetCenter();			//���� ����
		cam_tilt.setRatio(100);			//ī�޶� ����
		cam_tilt.setRatio(0);			//ī�޶� ����
		cam_tilt.resetCenter();			//ī�޶� ����
		cam_pan.setRatio(100);			//ī�޶� ����
		cam_pan.setRatio(0);			//ī�޶� ����
		cam_pan.resetCenter();			//ī�޶� ����
		DCmotor.go();					//dc���� ����
		waitKey(1500);					//wait 1.5sec
		DCmotor.stop();					//dc���� ����
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
			int key = waitKey(33);	//if you not press, return -1
			Manual.input(key);		//movement by keyboard
		}
	}
	//end manual mode


	else if (mode == 3) {
		//Mat intrinsic;
		//Mat disCoeff;
		//if (calibImage(videocap, intrinsic, disCoeff)) {
		//	cout << "Calibration Success!" << endl;
		//}
		//else {
		//	cout << "Calibration Failed!" << endl;
		//}
		//Mat undistortImg;
		//vector<Vec4i> exLines;
		//double steeringAngle;
		//while (1) {
		//	videocap >> frame;
		//	undistort(frame, undistortImg, intrinsic, disCoeff);
		//	imshow("Live", undistortImg);

		//	bool Check = extractLines(undistortImg, exLines);
		//	drivingAngle(undistortImg, exLines, steeringAngle);
		//	waitKey(15);
		//}
	}
	//end calb mode


	else if (mode == 4)	//daehee's code
	{
		/*
		~code processing calibration~
		*/

		Mat intrinsic = Mat(3, 3, CV_32FC1);
		Mat disCoeffs;
		int numBoards = 5;	// 5���� ������ ���
		DoCalib(disCoeffs, intrinsic, numBoards);
		cout << "complete 'DoCalib()' function" << endl;

		Mat distortFrame;

		Driving_DH DH(true, 1.00);	//printFlag, sLevel
									//sLevel : �������� �ΰ���(�������� ���� ����)
		DH.mappingSetSection(0, 0.10, 0.40, 0.70, 0.77, 1.00);
		//DH.mappingSetValue(0.0, 0.00, 10.0, 25.0, 50.0, 50.0);
		DH.mappingSetValue(10, 10.0, 15.0, 25.0, 50.0, 50.0);
		//�ڳʱ��� ������� ���ΰ� ����

		DetectColorSign detectColorSign(false);	//���� ǥ���� ���� Ŭ����

		double steerVal(50.0);	//�ʱ� ����(50�� �߽�)
		double speedVal(40.0);	//�ʱ� �ӵ�(0~100)

		while (true)
		{
			videocap >> distortFrame;
			undistort(distortFrame, frame, intrinsic, disCoeffs);

			if (false) //event üũ
			{

			}
			//else if (detectColorSign.isRedStop(frame, 10)) //������ ǥ���� ����
			//{
			//	while (detectColorSign.isRedStop(frame, 10))
			//	{
			//		DCmotor.stop();	//�����.
			//		imshow("frame", frame);
			//	}
			//}
			else if (false)	//��Ÿ event üũ
			{

			}
			else //��������
			{
				DH.driving(frame, steerVal, speedVal, 35.0, 0.0);

				steering.setRatio(steerVal);			//���� ����
				DCmotor.go(speedVal);
			
				waitKey(1);//33
			}
			imshow("frame", frame);
		}
	}
	//end daehee's code


	else if (mode == 5) // SangMin's code
	{

	}
	//end SangMin's code

	else if (mode == 6)
	{

	}


	else if (mode == 7)
	{
		//write your code
	}

	else cout << "invalid mode selection" << endl;

	cout << "program finished" << endl;
	allServoReset(pca);	// 3 Servo motor center reset
	return 0;
	//��
}
