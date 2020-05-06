#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include "CustomPicar.h"
#include "CV_drivingAngle.h"
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

	}
	//end manual mode


	else if (mode == 3) {

	}
	//end calb mode


	else if (mode == 4)	//daehee's code
	{
		
	}
	//end daehee's code


	else if (mode == 5) // SangMin's code
	{
		Mat intrinsic = Mat(3, 3, CV_32FC1);
		Mat disCoeffs;
		int numBoards = 20;
		DoCalib(disCoeffs, intrinsic, numBoards);

		Mat undistortImg;
		vector<Vec4i> exLines;

		double speedVal(20.0);	//�ʱ� �ӵ�(0~100)
		double steering_After, steering_Before = 0;

		int Mode;
		cout << "select Mode : ";
		cin >> Mode;
		cout << "Mode : " << Mode << endl;
		while (1) {
			videocap >> frame;
			undistort(frame, undistortImg, intrinsic, disCoeffs);
			imshow("Live", undistortImg);

			bool Check = extractLines(undistortImg, exLines);
			drivingAngle_SM(undistortImg, exLines, steering_After, steering_Before, Mode);
			steering.setRatio(50 + steering_After); //���� ����
			cout << "���Ⱒ : " << 50 + steering_After << endl;
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
	//��
}