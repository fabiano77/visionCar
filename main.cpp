#include <iostream>
#include "opencv2/opencv.hpp"
#include "kbhit.h"
#include "CustomPicar.h"
#include "CV_calibration.h"
#include "CV_drivingAngle.h"

using namespace std;
using namespace auto_car;
using namespace cv;

int main()
{
	//board, servoMotor configuration
	PCA9685 pca{};
	pca.set_pwm_freq(60.0);
	Servo steering(pca, Steering);			//set 0 :left,	100:right
	Servo cam_tilt(pca, Tilt);				//set 0 :left,	100:right
	Servo cam_pan(pca, Pan);				//set 0 :up,	100:down
	Wheel DCmotor(pca, LeftWheel, RightWheel);		//여기까진 안봐도됩니다.2
	steering.resetCenter();					//중앙 정렬
	cam_tilt.resetCenter();
	cam_pan.resetCenter();
	cout << "reset complete"<< endl <<endl;
	
	//OpenCV setting
	bool cam_mode = true;
	Mat frame;
	VideoCapture videocap(0);
	if (!videocap.isOpened())
	{
		cerr << "video capture fail!" << endl;
		return -1;
	}
	Size videoSize(Size(cvRound(videocap.get(CAP_PROP_FRAME_WIDTH)), cvRound(videocap.get(CAP_PROP_FRAME_HEIGHT))));
	double fps = videocap.get(CAP_PROP_FPS);
	cout << "video width :" << videoSize.width << endl;
	cout << "video height :" << videoSize.height << endl;
	cout << "video FPS :" << fps << endl << endl;;
	int delay = cvRound(1000 / fps);
	for(int i = 0; cam_mode && i < 5; i++)
	{
		videocap.read(frame);
		imshow("Live", frame);
		waitKey(5);
	}
	cout << "camera test complete" << endl << endl;
	
	//main start
	cout << "program start" << endl << endl;
	cout << "mode 1 : show mode" << endl;
	cout << "mode 2 : manual mode" << endl << endl;
	cout << "select mode : ";

	int mode;
	cin >> mode;
	
	if (mode == 1)//show mode
	{

		steering.setRatio(100);			//바퀴 우측
		steering.setRatio(0);			//바퀴 좌측
		steering.setRatio(100);			//바퀴 우측

		usleep(500000);	// wait 0.5sec

		cam_tilt.setRatio(100);			//카메라 상향
		cam_tilt.setRatio(0);			//카메라 하향
		cam_tilt.resetCenter();			//카메라 중앙 정렬

		usleep(500000);	// wait 0.5sec

		cam_pan++;				//카메라 우측, 상향 으로 10%씩 세번
		cam_tilt++;
		cam_pan++;
		cam_tilt++;
		cam_pan++;
		cam_tilt++;
		cam_pan--;				//카메라 좌측, 하향 으로 10%씩 세번
		cam_tilt--;
		cam_pan--;
		cam_tilt--;
		cam_pan--;
		cam_tilt--;

		usleep(500000);	// wait 0.5sec

		steering.setRatio(0);			//바퀴 좌측
		DCmotor.go();				//dc모터 시작

		usleep(1500000);	// wait 1.5sec

		steering.setRatio(100);			//바퀴 우측

		usleep(1200000);	// wait 1.2sec

		DCmotor.go(100);

		usleep(300000);	// wait 0.3sec

		DCmotor.stop();
	}
	else if (mode == 2)//manual mode
	{
		cout << "---------------------[key setting]------------------" << endl;
		cout << "    w      : go & speed up  |   i      : up" << endl;
		cout << "  a   d    : left, right    | j   l    : left, right" << endl;
		cout << "    s      : stop           |   k      : down" <<endl;
		cout << "  (move)                    | (cam)" << endl;
		cout << "   '0' is exit.             |" << endl;
		cout << "----------------------------------------------------" << endl;
		char c;
		double speed = 40;
		while (c != '0')
		{
			if(cam_mode)
			{
				videocap.read(frame);
				imshow("Live", frame);
				waitKey(1);
			}
			if (linux_kbhit())
			{
				c = linux_kbhit();
				switch (c)
				{
				case 'a':	//steering left
					steering--;
					break;
				case 'd':	//steering right
					steering++;
					break;
				case 's':	//stop
					DCmotor.stop();
					speed = 40;
					cout << "@stop!" << endl;
					break;
				case 'w':	//go and speed up
					DCmotor.go(speed);
					cout << "@current speed : " << speed << endl;
					if(speed <100)speed += 4;
					break;
				case 'j':	//cam left
					cam_pan++;
					break;
				case 'l':	//cam right
					cam_pan--;
					break;
				case 'i':	//cam up
					cam_tilt++;
					break;
				case 'k':	//cam down
					cam_tilt--;
					break;
				default:
					cout << "cam output" << endl;
					break;
				}
				usleep(2000);
			}
		}
	}
	else if (mode == 3) {
	Mat intrinsic;
	Mat disCoeff;
	if (calibImage(videocap, intrinsic, disCoeff)) {
		cout << "Calibration Success!" << endl;
	}
	else {
		cout << "Calibration Failed!" << endl;
	}
	while (1) {
		videocap >> frame;
		imshow(frame);
		char key = linux_kbhit();
		if (key == 'k') {
			continue;
		}
		else if (key == 'c') {
			break;
		}
	}
}
	else if (mode == 4)
	{
	cout << "test github.com" << endl;
 }
	else cout << "invalid mode selection" << endl;

	cout << "program finished" << endl;
	steering.resetCenter();
	cam_pan.resetCenter();
	cam_tilt.resetCenter();
	return 0;
}
