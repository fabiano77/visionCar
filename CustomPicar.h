#ifndef CUSTUMPICAR_H
#define CUSTUMPICAR_H

#include <iostream>
#include <string>
#include <unistd.h>
#include "PCA9685.h"

namespace auto_car
{
	//PCA9685 board PWM control number
#define Steering	0   //조향
#define Pan         1   //좌우
#define Tilt        2   //상하
#define LeftWheel   4   //좌측바퀴
#define RightWheel  5   //우측바퀴

	//servo motor action speed control time term ex) 2,000us
#define TERM        2000
}
using namespace auto_car;

class Servo
{
public:
	Servo();
	Servo(PCA9685 pca_, int motorPin, int timeTerm = 2000);		//초기 설정은 만질 일 없음.
	void setValue(uint16_t set_val);	//서보의 각도를 설정하는 함수.인자로 넘겨주는 값 190~530 가용범위이고, 360이 중심이다. 
						//서보가동 pulse width : 0.75 ~1.5~ 2.25ms
						//set_val 인자로 환산시 : 185 ~(370)~ 550
						//0번 조향바퀴 270좌 ~ 370중심 ~ 470우
						//1번 cam좌우 530(550)좌 ~ 360중심 ~ 190(150)우 
									// 괄호는 한계
						//2번 cam상하 650상 ~ 350중심 ~ 300하
						//4번 우측바퀴 1000~4000
						//5번 좌측바퀴 1000~4000
	
	void setRatio(double ratio);		//서보의 각도를 %비율로 설정하는 함수. 가용범위 0~100 이며 50% 이 중심.
	void resetCenter();			//서보의 각도를 중심으로 초기화하는 함수. tilt 서보는 50%이 중심이 아님. 약 20%정도
	void operator++(int);			//서보의 각도를 10% 증가
	void operator--(int);			//서보의 각도를 10% 감소
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
	Wheel();
	Wheel(PCA9685 pca_, int leftPin, int rightPin);		//초기 설정은 만질 일 없음.
	void go(double speed = 40);		//양쪽 뒷 바퀴 속도를 %로 설정하는 함수. 기본인자로 40%.
	void stop();				//말그대로 스톱.
private:
	PCA9685 board;
	int left;
	int right;
	uint16_t maxVal;
	uint16_t minVal;
	uint16_t length;
	double rate;
};

class ManualMode
{
public:
	ManualMode(PCA9685 pca_, double spd = 40.0);
	void input(int key_);
	void guide();

private:
	int key;
	double speed;
	Servo M_steering;
	Servo M_cam_tilt;
	Servo M_cam_pan;
	Servo M_DCmotor;
};

void allServoReset(PCA9685 pca_);

#endif //CUSTUMPICAR_H