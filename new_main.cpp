#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include "CustomPicar.h"
#include "Driving_DH.h"
#include "DetectColorSign.h"
#include "Calibration.h"

//cpp를 추가해보는 것은 어떠한가

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

	cout << "mode 4 : daehee's code" << endl;
	cout << "mode 5 : daehee's code_no camera" << endl << endl;
	cout << "select mode : ";
	int mode;
	cin >> mode;

	//start mode------------------------------------------------
	if (mode == 1)//test mode
	{
		steering.setRatio(100);			//바퀴 우측
		steering.setRatio(0);			//바퀴 좌측
		steering.resetCenter();			//바퀴 정렬
		cam_tilt.setRatio(100);			//카메라 상향
		cam_tilt.setRatio(0);			//카메라 하향
		cam_tilt.resetCenter();			//카메라 정렬
		cam_pan.setRatio(100);			//카메라 우향
		cam_pan.setRatio(0);			//카메라 좌향
		cam_pan.resetCenter();			//카메라 정렬
		DCmotor.go();					//dc모터 시작
		waitKey(1500);					//wait 1.5sec
		DCmotor.stop();					//dc모터 멈춤
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

		DetectColorSign detectColorSign(false);	//색깔 표지판 감지 클래스
		Driving_DH DH(true, 1.00);	//printFlag, sLevel
									//sLevel : 직선구간 민감도(높을수록 많이 꺾임)
		DH.mappingSetSection(0, 0.10, 0.40, 0.73, 0.79, 1.00);
		DH.mappingSetValue(8.0, 8.00, 15.0, 22.0, 50.0, 50.0);
		//DH.mappingSetValue(0.0, 0.00, 10.0, 25.0, 50.0, 50.0);
		//코너구간 조향수준 맵핑값 세팅

		double steerVal(50.0);	//초기 각도(50이 중심)
		double speedVal(40.0);	//초기 속도(0~100)

		cam_pan.setRatio(52);	//카메라 좌우 조절

		while (true)
		{
			TickMeter tm;	//시간 측정 클래스
			tm.start();		//시간 측정 시작

			videocap >> distortFrame;

			if (false) //event 체크
			{

			}
			//else if (detectColorSign.isRedStop(distortFrame, 10)) //빨간색 표지판 감지
			//{
			//	while (detectColorSign.isRedStop(distortFrame, 10))
			//	{
			//		DCmotor.stop();	//멈춘다.
			//		imshow("frame", frame);
			//		waitKey(5);
			//	}
			//}
			else if (false)	//기타 event 체크
			{

			}
			else //정상주행
			{
				undistort(distortFrame, frame, intrinsic, disCoeffs);
				DH.driving(frame, steerVal, speedVal, 37.0, 0.0);

				steering.setRatio(steerVal);
				DCmotor.go(speedVal);

			}

			imshow("frame", frame);
			waitKey(5);//33

			tm.stop();		//시간측정 끝
			cout << tm.getTimeMilli() << "[ms]" << '\n';
		}
	}
	//end daehee's code


	else if (mode == 5) 
	{//calibration start
		Mat intrinsic = Mat(3, 3, CV_32FC1);
		Mat disCoeffs;
		int numBoards = 5;	
		DoCalib(disCoeffs, intrinsic, numBoards);
		cout << "complete 'DoCalib()' function" << endl;
		//calib done

		Mat distortFrame;

		DetectColorSign detectColorSign(false);	//색깔 표지판 감지 클래스
		Driving_DH DH(true, 1.00);	//printFlag, sLevel
									//sLevel : 직선구간 민감도(높을수록 많이 꺾임)
		DH.mappingSetSection(0, 0.10, 0.40, 0.73, 0.79, 1.00);
		DH.mappingSetValue(8.0, 8.00, 15.0, 22.0, 50.0, 50.0);
		//DH.mappingSetValue(0.0, 0.00, 10.0, 25.0, 50.0, 50.0);
		//코너구간 조향수준 맵핑값 세팅

		double steerVal(50.0);	//초기 각도(50이 중심)
		double speedVal(40.0);	//초기 속도(0~100)

		cam_pan.setRatio(52);	//카메라 좌우 조절

		while (true)
		{
			TickMeter tm;	//시간 측정 클래스
			tm.start();		//시간 측정 시작

			videocap >> distortFrame;

			if (false) //event 체크
			{

			}
			//else if (detectColorSign.isRedStop(distortFrame, 10)) //빨간색 표지판 감지
			//{
			//	while (detectColorSign.isRedStop(distortFrame, 10))
			//	{
			//		DCmotor.stop();	//멈춘다.
			//		imshow("frame", frame);
			//		waitKey(5);
			//	}
			//}
			else if (false)	//기타 event 체크
			{

			}
			else //정상주행
			{
				undistort(distortFrame, frame, intrinsic, disCoeffs);
				DH.driving(frame, steerVal, speedVal, 37.0, 0.0);

				steering.setRatio(steerVal);
				DCmotor.go(speedVal);

			}
			tm.stop();		//시간측정 끝
			cout << tm.getTimeMilli() << "[ms]" << '\n';
		}

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
	//끝
}
