#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include "CustomPicar.h"
#include "SJ_drivingAngle.h"
#include "Calibration.h"

//cpp를 추가해보는 것은 어떠한가

using namespace std;
using namespace auto_car;
using namespace cv;

void filter_image(Mat image_color, Mat& image_filtered);

int main()
{
	//board, servoMotor configuration-----------------------------
	PCA9685 pca{};
	pca.set_pwm_freq(60.0);
	Servo Objsteering(pca, Steering);
	Servo cam_tilt(pca, Tilt);
	Servo cam_pan(pca, Pan);
	Wheel DCmotor(pca, LeftWheel, RightWheel);
	allServoReset(pca);			// 3 Servo motor center reset

	//OpenCV setting----------------------------------------------
	Mat frame;					//standard Mat obj
	VideoCapture cap(0);	//camera obj
	if (!cap.isOpened()) {
		cerr << "video capture fail!" << endl;
		return -1;
	}
	cout << "Camera test is complete" << endl << endl;

	//mode selection---------------------------------------------
	cout << "[visionCar] program start" << endl << endl;
	cout << "mode 1 : test mode" << endl;
	cout << "mode 2 : manual mode" << endl;
	cout << "mode 3 : calb & angle mode" << endl;
	cout << "mode 4 : SeokJun's code" << endl;
	cout << "mode 5 : who's code?" << endl;
	cout << "mode 6 : who's code?" << endl;
	cout << "mode 7 : who's code?" << endl << endl;
	cout << "select mode : ";
	int mode;
	cin >> mode;

	//start mode------------------------------------------------
	if (mode == 1)//test mode
	{
		//steering.setRatio(100);			//바퀴 우측
		//steering.setRatio(0);			//바퀴 좌측
		//steering.resetCenter();			//바퀴 정렬
		//cam_tilt.setRatio(100);			//카메라 상향
		//cam_tilt.setRatio(0);			//카메라 하향
		//cam_tilt.resetCenter();			//카메라 정렬
		//cam_pan.setRatio(100);			//카메라 우향
		//cam_pan.setRatio(0);			//카메라 좌향
		//cam_pan.resetCenter();			//카메라 정렬
		//DCmotor.go();					//dc모터 시작
		//waitKey(1500);					//wait 1.5sec
		//DCmotor.stop();					//dc모터 멈춤
	}
	//end test mode


	else if (mode == 2)//manual mode
	{
		//ManualMode Manual(pca, 40);	//ManualMode class & basic speed rate
		//Manual.guide();				//cout the key guide 
		//int key(-1);
		//while (key != 27)			//if not ESC
		//{
		//	videocap >> frame;
		//	imshow("Live camera", frame);
		//	int key = waitKey(33);	//if you not press, return -1
		//	Manual.input(key);		//movement by keyboard
		//}
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
		int numBoards = 20;
		DoCalib(disCoeffs, intrinsic, numBoards);

		cout << "Calibration Completed" << endl;

		Mat distortFrame;
		Point* linePt = new Point[4];
		char key;
		Mat image;
		Mat image_edge;
		Mat image_gauss;
		Mat image_gray;
		Mat image_line;
		Mat image_color;
		Mat image_ROI;
		Mat image_white;
		double steering = 0;
		double Dosteering = steering + 50;

		cap >> image;

		Mat mask = Mat::zeros(image.rows, image.cols, CV_8UC3);
		int height = image.rows; // 행의 갯수 = 높이
		int width = image.cols; // 열의 갯수 = 높이
		int numpoint[] = { 4 }; // fillPoly 함수의 매개변수에 들어갈 꼭지점의 갯수를 의미하는 배열.


		double steerVal(50.0);	//초기 각도(50이 중심)
		double speedVal(40.0);	//초기 속도(0~100)

		DCmotor.go(speedVal);

		while (true)
		{
			cap >> image;

			//Point pt[4] = { Point(width * 3 / 7,height * 3 / 5),Point(width * 4 / 7,height * 3 / 5),Point(width,height*0.9),Point(0,height*0.9) };
			Point pt[4] = { Point(0,height * 2 / 5),Point(width,height * 2 / 5),Point(width,height * 6 / 7),Point(0,height * 6 / 7) };
			Point ptArray[1][4] = { pt[0], pt[1], pt[2], pt[3] };

			const Point* ppt[1] = { ptArray[0] };

			fillPoly(mask, ppt, numpoint, 1, Scalar(255, 255, 255), 8);
			bitwise_and(mask, image, image_ROI);
			filter_image(image_ROI, image_edge);

			vector<Vec4i> lines;
			if (extractLines(image_edge, lines)) {
				//cout << "Extract Success!" << endl;
			}
			else {
				cout << "Extract Failed!" << endl;
			}

			linePt = drivingAngle(image_edge, lines, steering);

			line(image, linePt[0], linePt[1], Scalar(0, 0, 255), LINE_4);
			line(image, linePt[2], linePt[3], Scalar(0, 0, 255), LINE_4);

			Dosteering = steering;

			Objsteering.setRatio(Dosteering);

			cout << "DoSteering : " << Dosteering << endl;
			// 바퀴 조향 출력 0 ~ 100
			namedWindow("sss", WINDOW_AUTOSIZE);
			imshow("sss", image);
			waitKey(33);
		}
		delete[] linePt;
	}
	//end daehee's code


	else if (mode == 5) // SangMin's code
	{
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

		//double speedVal(40.0);	//초기 속도(0~100)
		//double steering_After, steering_Before = 0;

		//int Mode;
		//cout << "select Mode : ";
		//cin >> Mode;
		//cout << "Mode : " << Mode << endl;
		//while (1) {
		//	videocap >> frame;
		//	undistort(frame, undistortImg, intrinsic, disCoeff);
		//	imshow("Live", undistortImg);

		//	bool Check = extractLines(undistortImg, exLines);
		//	drivingAngle_SM(undistortImg, exLines, steering_After, steering_Before, Mode);
		//	steering.setRatio(50 + steering_After); //바퀴 조향
		//	cout << "조향각 : " << 50 + steering_After << endl;
		//	DCmotor.go(speedVal);
		//	waitKey(15);

		//}
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
	//끝
}

void filter_image(Mat image_color, Mat& image_filtered) {

	Mat image_HSV; // 노란색을 검출할 때 InputArray로 사용 할 객채
	Mat image_yellow; // 노란색을 검출하고 OutputArray로 사용
	Mat image_white; // 흰색을 검출하고 OutputArray로 사용
	Mat image_combine; // 흰색, 노란색 객채를 하나의 영상으로 병합
	Mat image_grayscale;
	Mat image_edge; // Canny함수를 이용하여 엣지를 검출함

	//inRange(image_color, lower_w, upper_w, image_white); // 영상에서 흰색 부분을 검출하여 저장
	//cvtColor(image_color, image_HSV, COLOR_BGR2HSV); // 노란색 검출을 위해 HSV 형식으로 변환
	inRange(image_HSV, lower_y, upper_y, image_yellow); // 영상에서 노란색 부분을 검출하여 저장

	//addWeighted(image_white, 1.0, image_yellow, 1.0, 0.0, image_combine); // 검출된 두 영상을 병합

	GaussianBlur(image_yellow, image_grayscale, Size(3, 3), 0, 0); // 가우시안 블러링
	Canny(image_grayscale, image_edge, 50, 150); // 캐니 엣지 검출
	image_edge.copyTo(image_filtered); // 영상을 두 번째 매개 변수에 저장하여 deep copy 실행
}
