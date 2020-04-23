#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include "kbhit.h"
#include "CustomPicar.h"
#include "CV_calibration.h"
#include <opencv2/core/matx.hpp>


using namespace std;
using namespace auto_car;
using namespace cv;

void drivingAngle(Mat& dst, vector<Vec4i> lines, double& stiring);
Mat regionOfInterest(Mat& src, Point* points);//roi 지정

bool extractLines(Mat& src, vector<Vec4i>& lines);//추출되면 1 안되면 0
//src는 컬러상태여야함
// lines는 call by ref로 반환됨
void filter_colors(Mat& src, Mat& img_filtered);//color filtering

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
	if (!videocap.isOpened()){
		cerr << "video capture fail!" << endl;
		return -1;
	}
	double fps = videocap.get(CAP_PROP_FPS);
	cout << "video width :" << videocap.get(CAP_PROP_FRAME_WIDTH) << endl;
	cout << "video height :" << videocap.get(CAP_PROP_FRAME_HEIGHT) << endl;
	cout << "video FPS :" << fps << endl << endl;;
	int delay = cvRound(1000 / fps);
	for (int i = 0; i < 5; i++){
		videocap.read(frame);
		imshow("Test Cam out", frame);
		waitKey(33);
	}
	cout << "Camera test is complete" << endl << endl;
	//mode selection---------------------------------------------
	cout << "[visionCar] program start" << endl << endl;
	cout << "mode 1 : test mode" << endl;
	cout << "mode 2 : manual mode" << endl;
	cout << "mode 3 : calb & angle mode" << endl << endl;
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
	}//end test mode



	else if (mode == 2)//manual mode
	{
		ManualMode Manual(pca, 40);	//ManualMode class & basic speed rate
		Manual.guide();				//cout the key guide 
		int key(-1);
		while (key != 27)			//if not ESC
		{
			videocap >> frame;
			imshow("Live camera", frame);
			int key = waitKey(delay);	//if you not press, return -1
			Manual.input(key);		//movement by keyboard
		}
	}//end manual mode



	else if (mode == 3) {
		Mat intrinsic;
		Mat disCoeff;
		if (calibImage(videocap, intrinsic, disCoeff)) {
			cout << "Calibration Success!" << endl;
		}
		else {
			cout << "Calibration Failed!" << endl;
		}
		Mat undistortImg;
		vector<Vec4i> exLines;
		double stiring;
		while (1) {
			videocap >> frame;
			undistort(frame, undistortImg, intrinsic, disCoeff);
			imshow("Live", undistortImg);

			bool Check = extractLines(undistortImg, exLines);
			drivingAngle(undistortImg, exLines, stiring);
			waitKey(15);
		}
	}



	else if (true)
	{
		//write your code
	}



	else cout << "invalid mode selection" << endl;
	cout << "program finished" << endl;
	allServoReset(pca);	// 3 Servo motor center reset
	return 0;
	//끝
}
