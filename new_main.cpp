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
	cout << "[visionCar] program start" << endl;

	cout << "mode 4 : daehee's code" << endl << endl;
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

	}
	//end manual mode


	else if (mode == 3) 
	{

	}



	else if (mode == 4)	//daehee's code
	{
		//calibration start
		Mat intrinsic = Mat(3, 3, CV_32FC1);
		Mat disCoeffs;
		int numBoards = 5;	
		DoCalib(disCoeffs, intrinsic, numBoards);
		cout << "complete 'DoCalib()' function" << endl;
		//calib done

		Mat distortFrame;

		DetectColorSign detectColorSign(false);	//���� ǥ���� ���� Ŭ����
		Driving_DH DH(true, 1.00);	//printFlag, sLevel
									//sLevel : �������� �ΰ���(�������� ���� ����)
		DH.mappingSetSection(0, 0.10, 0.40, 0.73, 0.79, 1.00);
		DH.mappingSetValue(8.0, 8.00, 15.0, 22.0, 50.0, 50.0);
		//DH.mappingSetValue(0.0, 0.00, 10.0, 25.0, 50.0, 50.0);
		//�ڳʱ��� ������� ���ΰ� ����

		double steerVal(50.0);	//�ʱ� ����(50�� �߽�)
		double speedVal(40.0);	//�ʱ� �ӵ�(0~100)

		cam_pan.setRatio(52);	//ī�޶� �¿� ����

		while (true)
		{
			TickMeter tm;	//�ð� ���� Ŭ����
			tm.start();		//�ð� ���� ����

			videocap >> distortFrame;

			if (false) //event üũ
			{

			}
			//else if (detectColorSign.isRedStop(distortFrame, 10)) //������ ǥ���� ����
			//{
			//	while (detectColorSign.isRedStop(distortFrame, 10))
			//	{
			//		DCmotor.stop();	//�����.
			//		imshow("frame", frame);
			//		waitKey(5);
			//	}
			//}
			else if (false)	//��Ÿ event üũ
			{

			}
			else //��������
			{
				undistort(distortFrame, frame, intrinsic, disCoeffs);
				DH.driving(frame, steerVal, speedVal, 37.0, 0.0);

				steering.setRatio(steerVal);
				DCmotor.go(speedVal);

			}

			imshow("frame", frame);
			waitKey(5);//33

			tm.stop();		//�ð����� ��
			cout << tm.getTimeMilli() << "[ms]" << '\n';
		}
	}
	//end daehee's code


	else if (mode == 5) 
	{

	}


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
