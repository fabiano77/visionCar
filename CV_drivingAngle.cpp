<<<<<<< HEAD
#include"CV_drivingAngle.h"
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
using namespace cv;
using namespace std;
void drivingAngle(Mat& dst, vector<Vec4i> lines, double& stiring) {
	Vec4f params;
	Point pt1, pt2;
	int x1, y1, x2, y2;
	vector<Vec4i> newLines;//후에 왼쪽오른쪽 하나만 남기기
	vector<float> slopeDegrees;
	float slopeDegree;
	float slopeThreshold = 0.5;
	int width = dst.size().width;
	int height = dst.size().height;

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
			slopeDegree = 0.999;
		else slopeDegree = (y2 - y1) / (float)(x2 - x1);
		//slope degree 에 따라 넣을지말지 결정
		if (abs(slopeDegree) > slopeThreshold) {
			//cout << "lines[" << k << "], p1=" << x1 << "," << y1 << endl;
			//cout << "========= p2=" << x2 << "," << y2 << endl;
			//cout << "slopeDegree = " << slopeDegree << endl;

			newLines.push_back(params);//나중에 쓸 parameter

			slopeDegree = atan(slopeDegree);
			slopeDegrees.push_back(slopeDegree);
			//x2,y2의 점
			pt1 = Point(x1, y1);
			pt2 = Point(x2, y2);
			line(dst, pt1, pt2, Scalar(0, 0, 255), 1);//라인 그리기
		}
	}
	//쉬운 드라이빙 각도 계산///////
	float sumSlopeDegree = 0;
	for (int i = 0; i < newLines.size(); i++) {
		sumSlopeDegree += slopeDegrees[i];
	}
	float drivingAngle = sumSlopeDegree / newLines.size() * 2 * CV_PI;
	cout << "----------drivingAngle : " << drivingAngle << endl;

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
		else if (slope < 0 && x1 < cx && x2 < cx)
			left_lines.push_back(line);
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
		dydxLeft = double(fitLeft[1]) / double(fitLeft[0]);
	}
	else { dydxLeft = 0; }
	if (right_index > 0) {
		fitLine(rightLines, fitRight, DIST_L2, 0, 0.01, 0.01);
		rp1.x = cvRound(fitRight[0] * s + fitRight[2]);//[0]은 방향 벡터 dx
		rp1.y = cvRound(fitRight[1] * s + fitRight[3]);//[1]은 방향벡터 dy
		rp0.x = cvRound(fitRight[0] * (-s) + fitRight[2]);
		rp0.y = cvRound(fitRight[1] * (-s) + fitRight[3]);
		dydxRight = double(fitRight[1]) / double(fitRight[1]);
	}
	else { dydxRight = 0; }

	//값저장
	if (abs(dydxLeft + dydxRight) <= tan(5 * 360 / (2 * CV_PI))) {
		stiring = 0;
	}
	else { atan((dydxLeft + dydxRight)); }
	slopeDegrees.clear();
	leftLines.clear();
	rightLines.clear();
	right_lines.clear();
	left_lines.clear();
	newPoint.clear();
	newLines.clear();

}

Mat regionOfInterest(Mat& src, Point* points) {// points의 포인터인 이유-> 여러개의 꼭짓점 경우

	Mat maskImg = Mat::zeros(src.size(), CV_8UC1);

	Scalar ignore_mask_color = Scalar(255, 255, 255);
	Scalar red = Scalar(0, 0, 255);
	const Point* ppt[1] = { points };//개의 꼭짓점 :n vertices
	int npt[] = { 4 };

	fillPoly(maskImg, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);
	Mat maskedImg;
	bitwise_and(src, maskImg, maskedImg);
	return maskedImg;
=======
#include"CV_drivingAngle.h"
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
using namespace cv;
using namespace std;
void drivingAngle(Mat& dst, vector<Vec4i> lines, double& stiring) {
	Vec4f params;
	Point pt1, pt2;
	int x1, y1, x2, y2;
	vector<Vec4i> newLines;//후에 왼쪽오른쪽 하나만 남기기
	vector<float> slopeDegrees;
	float slopeDegree;
	float slopeThreshold = 0.5;
	int width = dst.size().width;
	int height = dst.size().height;

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
			slopeDegree = 0.999;
		else slopeDegree = (y2 - y1) / (float)(x2 - x1);
		//slope degree 에 따라 넣을지말지 결정
		if (abs(slopeDegree) > slopeThreshold) {
			//cout << "lines[" << k << "], p1=" << x1 << "," << y1 << endl;
			//cout << "========= p2=" << x2 << "," << y2 << endl;
			//cout << "slopeDegree = " << slopeDegree << endl;

			newLines.push_back(params);//나중에 쓸 parameter

			slopeDegree = atan(slopeDegree);
			slopeDegrees.push_back(slopeDegree);
			//x2,y2의 점
			pt1 = Point(x1, y1);
			pt2 = Point(x2, y2);
			line(dst, pt1, pt2, Scalar(0, 0, 255), 1);//라인 그리기
		}
	}
	//쉬운 드라이빙 각도 계산///////
	float sumSlopeDegree = 0;
	for (int i = 0; i < newLines.size(); i++) {
		sumSlopeDegree += slopeDegrees[i];
	}
	float drivingAngle = sumSlopeDegree / newLines.size() * 2 * CV_PI;
	cout << "----------drivingAngle : " << drivingAngle << endl;

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
		else if (slope < 0 && x1 < cx && x2 < cx)
			left_lines.push_back(line);
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
		dydxLeft = double(fitLeft[1]) / double(fitLeft[0]);
	}
	else { dydxLeft = 0; }
	if (right_index > 0) {
		fitLine(rightLines, fitRight, DIST_L2, 0, 0.01, 0.01);
		rp1.x = cvRound(fitRight[0] * s + fitRight[2]);//[0]은 방향 벡터 dx
		rp1.y = cvRound(fitRight[1] * s + fitRight[3]);//[1]은 방향벡터 dy
		rp0.x = cvRound(fitRight[0] * (-s) + fitRight[2]);
		rp0.y = cvRound(fitRight[1] * (-s) + fitRight[3]);
		dydxRight = double(fitRight[1]) / double(fitRight[1]);
	}
	else { dydxRight = 0; }

	//값저장
	if (abs(dydxLeft + dydxRight) <= tan(5 * 360 / (2 * CV_PI))) {
		stiring = 0;
	}
	else { atan((dydxLeft + dydxRight)); }
	slopeDegrees.clear();
	leftLines.clear();
	rightLines.clear();
	right_lines.clear();
	left_lines.clear();
	newPoint.clear();
	newLines.clear();

}

Mat regionOfInterest(Mat& src, Point* points) {// points의 포인터인 이유-> 여러개의 꼭짓점 경우

	Mat maskImg = Mat::zeros(src.size(), CV_8UC1);

	Scalar ignore_mask_color = Scalar(255, 255, 255);
	Scalar red = Scalar(0, 0, 255);
	const Point* ppt[1] = { points };//개의 꼭짓점 :n vertices
	int npt[] = { 4 };

	fillPoly(maskImg, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);
	Mat maskedImg;
	bitwise_and(src, maskImg, maskedImg);
	return maskedImg;
>>>>>>> 5c586bc8a1d45903781e2ed57b19a8963640fd05
}