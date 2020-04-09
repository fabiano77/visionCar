#include <iostream>
#include "CustumPicar.h"
using namespace std;
using namespace auto_car;

//void usleep(int); 
// 

int main()
{
	PCA9685 pca{};					//
	pca.set_pwm_freq(60.0);
	Servo steering(pca, Steering);			//set 0 :left,	100:right
	Servo cam_tilt(pca, Tilt);				//set 0 :left,	100:right
	Servo cam_pan(pca, Pan);				//set 0 :up,	100:down
	Wheel DCmotor(pca, LeftWheel, RightWheel);		//여기까진 안봐도됩니다.

	cout << "program start" << endl;
	//position reset
	steering.resetCenter();					//중앙 정렬
	cam_tilt.resetCenter();
	cam_pan.resetCenter();

	usleep(500000);	// wait 0.5sec, this function is at linux library

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


	cout << "program finished" << endl;
	steering.resetCenter();
	cam_pan.resetCenter();
	cam_tilt.resetCenter();
	return 0;
}
