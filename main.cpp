#include <iostream>
#include "CustumPicar.h"
using namespace std;
using namespace auto_car;

//void usleep(int); 


int main()
{
	PCA9685 pca{};
	pca.set_pwm_freq(60.0);
	Servo steering(pca, Steering);			//set 0 :left,	100:right
	Servo cam_tilt(pca, Tilt);				//set 0 :left,	100:right
	Servo cam_pan(pca, Pan);				//set 0 :up,	100:down
	Wheel DCmotor(pca, LeftWheel, RightWheel);

	cout << "program start" << endl;
	//position reset
	steering.resetCenter();
	cam_tilt.resetCenter();
	cam_pan.resetCenter();

	usleep(500000);	// wait 0.5sec, this function is at linux library

	steering.setRatio(100);
	steering.setRatio(0);
	steering.setRatio(100);

	usleep(500000);	// wait 0.5sec

	cam_tilt.setRatio(100);
	cam_tilt.setRatio(0);
	cam_tilt.resetCenter();

	usleep(500000);	// wait 0.5sec

	cam_pan++;
	cam_tilt++;
	cam_pan++;
	cam_tilt++;
	cam_pan++;
	cam_tilt++;
	cam_pan--;
	cam_tilt--;
	cam_pan--;
	cam_tilt--;
	cam_pan--;
	cam_tilt--;

	usleep(500000);	// wait 0.5sec

	steering.setRatio(0);
	DCmotor.go();

	usleep(1500000);	// wait 1.5sec

	steering.setRatio(100);

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
