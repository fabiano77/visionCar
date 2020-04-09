#include <iostream>
#include "CustomPicar.h"
using namespace std;
using namespace auto_car;

Servo::Servo(PCA9685 pca_, int motorPin, int timeTerm)
{
	board = pca_;
	pin = motorPin;
	term = timeTerm;
	if (pin == Steering)
	{
		minVal = 270;
		maxVal = 470;
		centerVal = (minVal + maxVal) / 2;
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
	rate = 20;
	value = centerVal;
}
void Servo::setValue(uint16_t set_val)
{
	if (set_val > maxVal) set_val = maxVal;
	if (set_val < minVal) set_val = minVal;
	cout << "pin :" << pin << ", value " << value << ", set_val :" << set_val << endl;
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
	cout << "pin :" << pin << " centerVal :" << centerVal << endl;
	setValue(centerVal);
}
void Servo::operator++(int)
{
	uint16_t val = value + (length * (rate / 100));
	if (val > maxVal) val = maxVal;
	setValue(val);
}
void Servo::operator--(int)
{
	uint16_t val = value - (length * (rate / 100));
	if (val < minVal) val = minVal;
	setValue(val);
}

Wheel::Wheel(PCA9685 pca_, int leftPin, int rightPin)
{
	board = pca_;
	left = leftPin;
	right = rightPin;
	maxVal = 4000;
	minVal = 0;
	length = maxVal - minVal;
	rate = 20;
}
void Wheel::go(double speed)
{
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
