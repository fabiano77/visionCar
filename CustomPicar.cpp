#include <iostream>
#include "CustomPicar.h"
using namespace std;
using namespace auto_car;
Servo::Servo()
{
}
Servo::Servo(PCA9685 pca_, int motorPin, int timeTerm, bool printFlag)
{
	board = pca_;
	pin = motorPin;
	term = timeTerm;
	if (pin == Steering)
	{
		minVal = 270;
		maxVal = 470;
		centerVal = (minVal + maxVal) / 2;	//370
	}
	else if (pin == Pan)
	{
		minVal = 190;
		maxVal = 530;
		centerVal = 360;
	}
	else if (pin == Tilt)
	{
		minVal = 300;
		maxVal = 650;
		centerVal = 350;
	}
	else
	{
		cout << "Wrong servo object setting." << endl;
		cout << "program exit" << endl;
		exit(1);
	}
	length = maxVal - minVal;
	rate = 10;
	value = centerVal;
}
void Servo::setValue(uint16_t set_val)
{
	if (set_val > maxVal) set_val = maxVal;
	if (set_val < minVal) set_val = minVal;
	//cout << " @pin :" << pin << ", value " << value << ", set_val :" << set_val << endl;
	//서보각 출력.
	//if (print) cout << " @servo pin :" << pin << ", ratio : " << (((double)(set_val - minVal) / (double)length) * 100.0) << endl;
	while (value < set_val)
	{
		value += 1;
		board.set_pwm(pin, 0, value);
		usleep(term);
	}
	while (value > set_val)
	{
		value -= 1;
		board.set_pwm(pin, 0, value);
		usleep(term);
	}
}
void Servo::setRatio(double ratio)
{
	uint16_t val = minVal + length * (ratio / 100);
	setValue(val);
}
void Servo::resetCenter()
{
	if (print) cout << " @pin :" << pin << " centerVal :" << centerVal << endl;
	setValue(centerVal + 10);
	setValue(centerVal - 10);
	setValue(centerVal);
}
void Servo::operator++(int)
{
	uint16_t val = value + (length * (rate / 100));
	if (val > maxVal) val = maxVal;
	setValue(val);
	
	cout << " @pin :" << pin << ", setRatio = " << ((val - minVal)*100.0) / length << endl;
}
void Servo::operator--(int)
{
	uint16_t val = value - (length * (rate / 100));
	if (val < minVal) val = minVal;
	setValue(val);

	cout << " @pin :" << pin << ", setRatio = " << ((val - minVal) * 100.0) / length << endl;
}

Wheel::Wheel()
{
}
Wheel::Wheel(PCA9685 pca_, int leftPin, int rightPin)
{
	if (wiringPiSetup() == -1)
	{
		cout << "wiringPiSetup FAIL!!" << '\n';
		exit(1);
	}

	pinMode(0, OUTPUT);	//BCM_17,
	pinMode(2, OUTPUT);	//BCM_27,

	backwardFlag = false;
	board = pca_;
	left = leftPin;
	right = rightPin;
	maxVal = 4000;
	minVal = 0;
	length = maxVal - minVal;
	rate = 10;
}
void Wheel::go(double speed)
{
	if (backwardFlag)
	{
		backwardFlag = false;
		digitalWrite(0, 0);
		digitalWrite(2, 0);
	}
	if (speed > 100) speed = 100;
	if (speed < 0) speed = 0;
	uint16_t val = length * (speed / 100);
	board.set_pwm(left, 0, val);
	board.set_pwm(right, 0, val);
}
void Wheel::stop()
{
	board.set_pwm(left, 0, 0);
	board.set_pwm(right, 0, 0);
}
void Wheel::backward(double speed)
{
	if (!backwardFlag)
	{
		backwardFlag = true;
		digitalWrite(0, 1);
		digitalWrite(2, 1);
	}
	if (speed > 100) speed = 100;
	if (speed < 0) speed = 0;
	uint16_t val = length * (speed / 100);
	board.set_pwm(left, 0, val);
	board.set_pwm(right, 0, val);
}

UltraSonic::UltraSonic(int trigerPin, int echoPin)
{
	TRIGPIN = trigerPin;
	ECHOPIN = echoPin;

	if (wiringPiSetup() == -1)
	{
		cout << "wiringPiSetup FAIL!!" << '\n';
		exit(1);
	}
	pinMode(TRIGPIN, OUTPUT);
	pinMode(ECHOPIN, INPUT);
}

double UltraSonic::distance()
{
	double distance, start, stop;

	digitalWrite(TRIGPIN, 0);
	delayMicroseconds(2);
	digitalWrite(TRIGPIN, 1);
	delayMicroseconds(20);
	digitalWrite(TRIGPIN, 0);

	while (digitalRead(ECHOPIN) == 0);
	start = micros();
	while (digitalRead(ECHOPIN) == 1);
	stop = micros();

	distance = (stop - start) / 58;
	return distance;
}


ManualMode::ManualMode(PCA9685 pca_, double spd)
{
	M_steering = Servo(pca_, Steering);
	M_cam_tilt = Servo(pca_, Tilt);
	M_cam_pan = Servo(pca_, Pan);
	M_DCmotor = Wheel(pca_, LeftWheel, RightWheel);
	speed = spd;			//basic speed
}

void ManualMode::input(int key_)
{
	switch (key_)
	{
	case 'a':	//steering left
		M_steering--;
		break;
	case 'd':	//steering right
		M_steering++;
		break;
	case 's':	//stop
		M_DCmotor.stop();
		break;
	case 'w':	//go and speed up
		M_DCmotor.go(speed);
		break;
	case 'x':	//go backward
		M_DCmotor.backward(speed);
		break;
	case 'j':	//cam left
		M_cam_pan++;
		break;
	case 'l':	//cam right
		M_cam_pan--;
		break;
	case 'i':	//cam up
		M_cam_tilt++;
		break;
	case 'k':	//cam down
		M_cam_tilt--;
		break;
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		speed = (key_ - '0') * 10;
		cout << "current speed = " << speed << endl;
		break;
	case -1:
		cout << "wait input & camera out" << endl;
		break;
	default:
		cout << "wrong key input." << endl;
		break;
	}
}

void ManualMode::guide()
{
	cout << "---------------------[key setting]------------------" << endl;
	cout << "    w      : go & speed up    |   i      : up" << endl;
	cout << "  a s d    : left, stop,right | j   l    : left, right" << endl;
	cout << "    x      : backward         |   k      : down" << endl;
	cout << "  (move)                      | (cam)" << endl;
	cout << "   '0' is exit.               |" << endl;
	cout << "----------------------------------------------------" << endl;
	cout << "	1, 2, 3, 4, 5, 6, 7, 8, 9 = speed setting" << endl;
	cout << "----------------------------------------------------" << endl;
}

void allServoReset(PCA9685 pca_)
{
	Servo a_steering(pca_, Steering);
	Servo a_cam_tilt(pca_, Tilt);
	Servo a_cam_pan(pca_, Pan);
	a_steering.resetCenter();
	a_cam_tilt.resetCenter();
	a_cam_pan.resetCenter();
}