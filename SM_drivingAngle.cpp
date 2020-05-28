#include "SM_drivingAngle.h"
#include "ImageProcessing_Constants.h"
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int B = 80;
int G = 80;
int R = 80;

static void on_trackbar(int, void*)
{
}

CheckStart::CheckStart() {
	lower_white = Scalar(180, 180, 180);
	upper_white = Scalar(255, 255, 255);
	lower_black = Scalar(0, 0, 0);
	upper_black = Scalar(100, 100, 100);
	flag_start = -1;
	flag_tunnel = -1;
	check_start = -1;
}

bool CheckStart::isWhite(Mat& frame, double percent) {

	bool returnVal;
	Mat frame_white;

	//cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
	inRange(frame, lower_white, upper_white, frame_white);

	int whitePixel(0);
	for (int i = 0; i < frame_white.cols; i += 10)				// 10픽셀마다 하나씩 검사함 속도를 위해
	{
		for (int j = 0; j < frame_white.rows; j += 10)
		{
			if (frame_white.at<uchar>(j, i))	whitePixel++;		// 흰색이 나오는 픽셀 개수 계산
		}
	}

	double whiteRatio = ((double)whitePixel / ((frame.cols / 10) * (frame.rows / 10)));	//검출된 픽셀수를 전체 픽셀수로 나눈 비율
	whiteRatio *= 100;

	if (whiteRatio > percent) { returnVal = true; }
	else { returnVal = false; }

	return returnVal;
}

bool CheckStart::isBlack(Mat& frame, double percent) {

	bool returnVal;
	Mat frame_black;

	//cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
	inRange(frame, lower_black, upper_black, frame_black);
	imshow("frame_black", frame_black);

	createTrackbar("B", "trackbar", &B, 255, on_trackbar);
	createTrackbar("G", "trackbar", &G, 255, on_trackbar);
	createTrackbar("R", "trackbar", &R, 255, on_trackbar);
	upper_black = Scalar(B, G, R);

	namedWindow("trackbar", WINDOW_NORMAL);
	moveWindow("trackbar", 700, 40);

	int blackPixel(0);
	for (int i = 0; i < frame_black.cols; i += 10)				// 10픽셀마다 하나씩 검사함 속도를 위해
	{
		for (int j = 0; j < frame_black.rows; j += 10)
		{
			if (frame_black.at<uchar>(j, i))	// 검은색이 나오는 픽셀 개수 계산
				blackPixel++;		
		}
	}


	double blackRatio = ((double)blackPixel / ((frame.cols / 10) * (frame.rows / 10)));	//검출된 픽셀수를 전체 픽셀수로 나눈 비율
	blackRatio *= 100;
	cout << "blackRatio : " << blackRatio << endl;
	if (blackRatio > percent) { returnVal = true; }
	else { returnVal = false; }

	return returnVal;
}

bool CheckStart::isDark(Mat& frame, double percent) {

	bool returnVal;
	Mat grayFrame;

	cvtColor(frame, grayFrame, COLOR_RGB2GRAY);

	int pixelCnt(0);
	int pixelValue(0);
	for (int i = 0; i < grayFrame.cols; i += 10)				// 10픽셀마다 하나씩 검사함 속도를 위해
	{
		for (int j = 0; j < grayFrame.rows; j += 10)
		{
			pixelValue += grayFrame.at<uchar>(j, i);
			pixelCnt++;
		}
	}
	int totalValue = pixelCnt * 255;
	double brightRate = ((double)pixelValue / totalValue) * 100.0;

	if (brightRate < (100 - percent))
	{
		putText(grayFrame, "detect tunnel!", Point(frame.cols / 4, frame.rows * 0.65), FONT_HERSHEY_COMPLEX, 3, Scalar(255), 2);
		returnVal = true;
	}
	else returnVal = false;

	return returnVal;
}

bool CheckStart::isStop(Mat& frame, double percent) {
	if (check_start == 0) { // 4. 최종 상황 : 한번이라도 출발했으면 계속 출발상태이다.
		return false;
	}
	else { // 출발하기 전 상황
		if (flag_start > 0) {// 2. 흰색 카드가 검출 된 후 사라졌을 때 1 프레임 당 flag 감소
			flag_start--;
			return true;
		}
		else if (flag_start == 0) { // 3. flag가 0이 될 경우 출발
			putText(frame, "Go!", Point(frame.cols / 4, frame.rows * 0.65), FONT_HERSHEY_COMPLEX, 3.5, Scalar(255), 2);
			if (check_start != 0) {
				check_start = 0; // 출발했다는 표시
			}
			return false;
		}
		else { // 1. 초기 상황
			if (isWhite(frame, percent)) { // 흰색 카드가 검출되면 flag 활성화
				flag_start = 15; // 해당 frame 이후 출발
			}
			return true;
		}
	}
}

bool CheckStart::isTunnel(Mat& frame, double percent) {

	if (isDark(frame, percent)) { // 어두워지면 flag 증가
		if (flag_tunnel < 20) // 최대 임계값
			flag_tunnel++;
	}
	else { // 밝으면 flag 감소
		if (flag_tunnel > 0)
			flag_tunnel--;
	}

	if (flag_tunnel >= 10) { // 최소 임계값. flag가 이보다 크면 터널 안에 있다고 인식
		putText(frame, "Tunnel!", Point(frame.cols / 4, frame.rows * 0.65), FONT_HERSHEY_COMPLEX, 3.5, Scalar(255), 2);
		return true;
	}
	else { // 최소 임계값보다 flag가 작으면 터널 밖이라고 인식
		return false;
	}
}

int CheckStart::GetFlag_start() {
	return flag_start;
}

int CheckStart::GetFlag_tunnel() {
	return  flag_tunnel;
}

//////////////////////////////////////////////////////////////////////////////////////////////

RoundAbout::RoundAbout() {
	flag1_start = -1;
	check1_start = -1;
	lower1_distance = 25;
	uper1_distance = 40;
	flag_wait = -1;

	flag2_start = -1;
	check2_start = -1;
	lower2_distance = 20;
	uper2_distance = 40;
}

bool RoundAbout::isStop(const double Distance) {
	if (check1_start == 0) { // 4. 최종 상황 : 한 번이라도 출발했을 경우 출발을 유지한다.
		cout << "4. : go! " << endl;
		return false;
	}
	else { // 정지선에서 대기 상태
		if (flag1_start > 0) { // 2. 앞의 차량이 일정 거리 이상 멀어질 경우 1프레임 당 flag 감소
			if (Distance >= uper1_distance) {
				flag1_start--;
			}
			cout << "2. : flag1_start = " << flag1_start << endl;
			return true;
		}
		else if (flag1_start == 0) { // 3. flag가 0이 될 경우 출발
			if (check1_start != 0) {
				check1_start = 0; // 출발했다는 표시
			}
			cout << "3. : start! " << endl;
			return false; // 출발
		}
		else // 1. 초기 상황
		{
			if (Distance < lower1_distance) {
				if (flag_wait < 3)
					flag_wait++;
			}
			else {
				if (flag_wait > 0) {
					flag_wait--;
				}
			}
			cout << "1. : flag_wait = " << flag_wait << ",";
			if (flag_wait == 3) {	
				flag_wait = -1;
				flag1_start = 35; // 1초당 10프레임정도 처리
				cout << " flag1_start = " << flag1_start;
			}
			cout << endl;
			return true;
		}
	}
}

bool RoundAbout::isDelay(const double Distance) {
	if (Distance < lower2_distance) { // 앞의 차량이 나타났을 때 flag 활성화
		if (flag_wait < 3) {
			flag_wait++;
		}
		if (flag_wait == 3) {
			flag_wait = -1;
			flag2_start = 35;
		}
		return true; // 정지
	}
	else { // 앞의 차량이 가깝지 않을 때
		if (flag_wait > 0) {
			flag_wait--;
		}
		if (flag2_start < 0) { // flag가 비활성화 되었을 때
			return false; // 출발
		}
		else { // flag가 활성화 되어 있을 때
			if (Distance >= uper2_distance) { // 앞의 차량이 일정 거리 이상으로 멀어질 경우				
				flag2_start--;
			}
			return true;
		}
	}
}
