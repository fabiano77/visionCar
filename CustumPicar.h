#ifndef CUSTUMPICAR_H
#define CUSTUMPICAR_H

#include <iostream>
#include <string>
#include <unistd.h>
#include "PCA9685.h"

class Servo
{
public:
	Servo(PCA9685 pca_, int motorPin, int timeTerm = 2000);
	void setValue(uint16_t set_val);
	void setRatio(double ratio);
	void resetCenter();
	void operator++(int);
	void operator--(int);
private:
	PCA9685 board;
	int pin;
	int term;
	uint16_t value;
	uint16_t centerVal;
	uint16_t maxVal;
	uint16_t minVal;
	uint16_t length;
	double rate;	// moving ratio of operator++, --. set in constructor
};

class Wheel
{
public:
	Wheel(PCA9685 pca_, int leftPin, int rightPin);
	void go(double speed = 40);
	void stop();
private:
	PCA9685 board;
	int left;
	int right;
	uint16_t maxVal;
	uint16_t minVal;
	uint16_t length;
	double rate;
};

namespace auto_car
{
	//PCA9685 board PWM control number
#define Steering	0   //Á¶Çâ
#define Pan         1   //ÁÂ¿ì
#define Tilt        2   //»óÇÏ
#define LeftWheel   4   //ÁÂÃø¹ÙÄû
#define RightWheel  5   //¿ìÃø¹ÙÄû

	//servo motor action speed control time term ex) 2,000us
#define TERM        2000
}

#endif //CUSTUMPICAR_H