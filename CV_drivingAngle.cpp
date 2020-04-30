#include "CV_drivingAngle.h"
#include "ImageProcessing_Constants.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
using namespace cv;
using namespace std;

void drivingAngle(Mat& inputImg, vector<Vec4i> lines, double& steering) {
	Vec4f params;
	Point pt1, pt2;
	int x1, y1, x2, y2;
	vector<Vec4i> newLines;//후에 왼쪽오른쪽 하나만 남기기
	const int width = inputImg.size().width;
	const int height = inputImg.size().height;
	vector<float> slopeDegrees;
	float slopeDegree;//항상 라디안으로 만들것
	double preSteering = steering;//이전 값불러오기 최종 조향각도 조절용
	float slopeThreshold = 0.3;//항상 라디안으로 만들 것
	//임시로 1rad(56.6도 정도) 으로해서 모든 라인 다 검출

	//vector point로 선언해보자
	vector<Point> newPoint;
	for (int k = 0; k < lines.size(); k++) {
		params = lines[k];
		x1 = params[0];
		y1 = params[1];
		//x1,y1의 점
		x2 = params[2];
		y2 = params[3];
		//x2,y2의 점

		pt1 = Point(x1, y1);
		pt2 = Point(x2, y2);

		if (x2 - x1 == 0)
			slopeDegree = 999;//x의 변화량이 없는 경우 각도 90도로 만들기
		else slopeDegree = (y2 - y1) / (float)(x2 - x1);
		//slope degree 에 따라 넣을지말지 결정
		if (abs(slopeDegree) > slopeThreshold) {
			newLines.push_back(params);
			slopeDegree = atan(slopeDegree);
			slopeDegrees.push_back(slopeDegree);
		}
	}
	// Split lines into right_lines and left_lines, representing the right and left lane lines
	// Right / left lane lines must have positive / negative slope, and be on the right / left half of the image
	vector<Vec4i> right_lines;
	vector<Vec4i> left_lines;

	for (int i = 0; i < newLines.size(); i++)
	{
		Vec4i line = newLines[i];
		float slope = slopeDegrees[i];
		x1 = line[0];
		y1 = line[1];
		x2 = line[2];
		y2 = line[3];

		float cx = width * 0.5; //x coordinate of center of image
		if (slope > 0 && x1 > cx&& x2 > cx)
			right_lines.push_back(line);
		//slope가 0보다 크면 pi/2+a rad에서 온 것이므로 오른쪽일 것
		else if (slope < 0 && x1 < cx && x2 < cx)
			left_lines.push_back(line);
		//slope가 0보다 작으면 pit/2-a rad에서 온 것이므로 왼쪽일 것임
	}

	//Run linear regression to find best fit line for right and left lane lines
	//Right lane lines
	double right_lines_x[1000];
	double right_lines_y[1000];
	float right_m, right_b;
	vector<Point> rightLines;

	int right_index = 0;
	for (int i = 0; i < right_lines.size(); i++) {

		Vec4i line = right_lines[i];

		x1 = line[0];
		y1 = line[1];
		x2 = line[2];
		y2 = line[3];

		right_lines_x[right_index] = x1;
		right_lines_y[right_index] = y1;
		rightLines.push_back(Point(x1, y1));//
		right_index++;
		right_lines_x[right_index] = x2;
		right_lines_y[right_index] = y2;
		rightLines.push_back(Point(x2, y2));//
		right_index++;
	}

	double left_lines_x[1000];
	double left_lines_y[1000];
	vector<Point> leftLines;
	float left_m, left_b;

	int left_index = 0;
	for (int i = 0; i < left_lines.size(); i++) {

		Vec4i line = left_lines[i];
		x1 = line[0];
		y1 = line[1];
		leftLines.push_back(Point(x1, y1));//
		x2 = line[2];
		y2 = line[3];
		leftLines.push_back(Point(x2, y2));//
		left_lines_x[left_index] = x1;
		left_lines_y[left_index] = y1;
		left_index++;
		left_lines_x[left_index] = x2;
		left_lines_y[left_index] = y2;
		left_index++;
	}

	Vec4f fitLeft, fitRight;

	Point rp1, rp0;//오른쪽
	Point lp1, lp0;//왼쪽
	float s = 1000;//값 
	double dydxLeft, dydxRight;//각 축별 기울기 값
	//방향 벡터 구하는 곳임
	if (left_index > 0) {
		fitLine(leftLines, fitLeft, DIST_L2, 0, 0.01, 0.01);
		lp1.x = cvRound(fitLeft[0] * (+s) + fitLeft[2]);
		lp1.y = cvRound(fitLeft[1] * (+s) + fitLeft[3]);
		lp0.x = cvRound(fitLeft[0] * (-s) + fitLeft[2]);
		lp0.y = cvRound(fitLeft[1] * (-s) + fitLeft[3]);

		line(inputImg, lp1, lp0, Scalar(0, 0, 255), 1);
		dydxLeft = double(fitLeft[1]) / double(fitLeft[0]);
	}
	else { dydxLeft = 0; }//한쪽라인 인식 안되는 예외 처리 부분

	if (right_index > 0) {
		fitLine(rightLines, fitRight, DIST_L2, 0, 0.01, 0.01);
		rp1.x = cvRound(fitRight[0] * s + fitRight[2]);//[0]은 방향 벡터 dx
		rp1.y = cvRound(fitRight[1] * s + fitRight[3]);//[1]은 방향벡터 dy
		rp0.x = cvRound(fitRight[0] * (-s) + fitRight[2]);
		rp0.y = cvRound(fitRight[1] * (-s) + fitRight[3]);

		line(inputImg, rp1, rp0, Scalar(0, 0, 255), 1);
		dydxRight = double(fitRight[1]) / double(fitRight[1]);
	}
	else { dydxRight = 0; } // 한쪽라인 인식 안되는 예외 처리 부분

	//값저장
	double angleThreshold = 10;// 10도 이하는 0으로만들기
	if (atan(abs(dydxLeft + dydxRight)) <= (angleThreshold * CV_PI / 180)) {
		steering = 0;
	}
	else {
		steering = 180 / CV_PI * atan((dydxLeft + dydxRight));
	}
	//steering값은 각도로 나오며 정면기준 0도임

	// 좌우 인식 안되는 경우 알고리즘 인식 부분(회전의 경우) -> 수정 자유롭게 하세요
	if (right_index == 0 && left_index != 0) {//우회전의 경우 인지 판단
		steeringFlag++;

		if (steeringFlag >= steeringThresholdFlag)//일정 프레임동안 발견되지 않는 경우 좌회전으로 인식
		{
			steering = (steering - preSteering) / 2.0 + (-1.0) * preSteering;//변화값 가중치/2
			//기존 직진상태에서는 반대방향으로 조향하게 했으므로 preSteering이 음수로 변환된 후 곱해야함
			cout << "우회전" << endl;
		}
		else {//flag전에는 이전 steering각도 유지
			steering = preSteering;
		}
	}
	else if (right_index != 0 && left_index == 0) {//좌회전의 경우인지 판단
		steeringFlag++;
		if (steeringFlag >= steeringThresholdFlag)//일정 프레임동안 발견되지 않는 경우 좌회전으로 인식
		{
			steering = (steering - preSteering) / 2.0 + (-1.0) * preSteering;//변화값 가중치/2
			//기존 직진상태에서는 반대방향으로 조향하게 했으므로 preSteering이 음수로 변환된 후 곱해야함
			cout << "좌회전" << endl;
		}
		else {//flag전에는 이전 steering각도 유지
			steering = preSteering;
		}
	}
	else if (left_index != 0 && right_index != 0) {//평상시 직진의 경우 변화값의 반대 1/2로 가중치를 줌
		//다른 경우에서 flag증가시킨 것 초기화
		if (steeringFlag <= 0) {
			steering = (-1.0) / 2.0 * (steering);
		}
		else if (steeringFlag > 0) {
			steering = steering * 0.75 + preSteering * 0.25;
			steeringFlag--;
		}
	}

	cout << "steering: " << steering << endl;
	slopeDegrees.clear();
	leftLines.clear();
	rightLines.clear();
	right_lines.clear();
	left_lines.clear();
	newPoint.clear();
	newLines.clear();
}
void regionOfInterest(Mat& src, Mat& dst, Point* points) {// points의 포인터인 이유-> 여러개의 꼭짓점 경우

	Mat maskImg = Mat::zeros(src.size(), CV_8UC1);

	Scalar ignore_mask_color = Scalar(255, 255, 255);
	const Point* ppt[1] = { points };//개의 꼭짓점 :n vertices
	int npt[] = { 4 };

	fillPoly(maskImg, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);
	Mat maskedImg;
	bitwise_and(src, maskImg, maskedImg);
	dst = maskedImg;
}

void imgBlur(Mat& src, Mat& dst, int processingCode) {
	if (processingCode == 1) {//gaussian Blur
		GaussianBlur(src, dst, Size(3, 3), 0, 0);
	}
	else if (processingCode == 2) {//Canny edge
		Canny(src, dst, 50, 150);
	}

}
bool extractLines(Mat& src, vector<Vec4i>& lines) {
	Mat filterImg;
	Mat grayImg, blurImg, edgeImg, roiImg, dstImg;
	int width = src.size().width;
	int height = src.size().height;
	filter_colors(src, filterImg);
	cvtColor(filterImg, grayImg, COLOR_BGR2GRAY);
	imgBlur(grayImg, blurImg, 1);
	imgBlur(blurImg, edgeImg, 2);
	Point pt[4] = { Point(0,height * 2 / 5),Point(width,height * 2 / 5),Point(width,height * 6 / 7),Point(0,height * 6 / 7) };
	//roi point 설정
	regionOfInterest(edgeImg, roiImg, pt);
	vector<Vec4i> extractLines;
	HoughLinesP(roiImg, extractLines, 1, CV_PI / 180.0, 30, 10, 20);
	lines = extractLines;
	return true;

}

void filter_colors(Mat& src, Mat& img_filtered) {
	//
	UMat bgrImg;
	UMat hsvImg;
	UMat maskWhite, whiteImg;
	UMat maskYellow, yellowImg;
	UMat imgCombined;
	src.copyTo(bgrImg);
	Scalar lower_white = Scalar(120, 120, 120); //흰색 차선 (RGB)
	Scalar upper_white = Scalar(255, 255, 255);
	Scalar lower_yellow = Scalar(10, 100, 100); //노란색 차선 (HSV)
	Scalar upper_yellow = Scalar(40, 255, 255);

	//white 변경
	inRange(bgrImg, lower_white, upper_white, maskWhite);
	//lower와 upper사이의 값을 1로 나머지는 0으로 저장
	bitwise_and(bgrImg, bgrImg, whiteImg, maskWhite);

	cvtColor(bgrImg, hsvImg, COLOR_BGR2HSV);
	inRange(hsvImg, lower_yellow, upper_yellow, maskYellow);
	bitwise_and(bgrImg, bgrImg, yellowImg, maskYellow);
	addWeighted(whiteImg, 1.0, yellowImg, 1.0, 0.0, imgCombined);//두 이미지 합치기

	imgCombined.copyTo(img_filtered);
}
