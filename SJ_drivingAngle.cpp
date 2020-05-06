#include "SJ_drivingAngle.h"
#include "ImageProcessing_Constants.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
using namespace cv;
using namespace std;

static int leftCorner_threshold = 0;
static int rightCorner_threshold = 0;
// 일정 갯수 이상의 프레임 동안 연속해서 하나의 차선만 검출되는지 기록하기 위한 변수

Point* drivingAngle(Mat& inputImg, vector<Vec4i> lines, double& steering) {

	double outSteering = 0;
	bool leftCorner_flag = false;
	bool rightCorner_flag = false;
	// 위에서의 기록 변수에 임계값 이상의 프레임에서 한 개의 차선만 검출되면 true로 변환
	// 그 이후에 Corner에 대한 조건문이 실행
	Vec4f params;
	Point pt1, pt2;
	Point returnPt[4];
	int x1, y1, x2, y2;
	vector<Vec4i> newLines;//후에 왼쪽오른쪽 하나만 남기기
	const int width = inputImg.size().width;
	const int height = inputImg.size().height;
	vector<float> slopeDegrees;
	float slopeDegree;//항상 라디안으로 만들것
	double preSteering = steering;//이전 값불러오기 최종 조향각도 조절용
	float slopeThreshold = 0.3;//항상 라디안으로 만들 것
	float headingAngle;
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
		//line(inputImg, lp0, lp1, Scalar(0, 0, 255), LINE_4);

		//cout << "lp1.x : " << lp1.x << " lp1.y : " << lp1.y << "lp0.x : " << lp0.x << " lp0.y : " << lp0.y << endl;
		dydxLeft = double(-fitLeft[1]) / double(fitLeft[0]);
		if (leftCorner_threshold != 0)
			leftCorner_threshold = 0;
	}
	else {
		dydxLeft = 0;
		cout << "왼쪽 차선 검출 안됨 : ";
		leftCorner_threshold++;
		cout << leftCorner_threshold << endl;
		if (leftCorner_threshold >= 5)
			leftCorner_flag = true;
	}//한쪽라인 인식 안되는 예외 처리 부분

	if (right_index > 0) {
		fitLine(rightLines, fitRight, DIST_L2, 0, 0.01, 0.01);
		rp1.x = cvRound(fitRight[0] * (+s) + fitRight[2]);//[0]은 방향 벡터 dx
		rp1.y = cvRound(fitRight[1] * (+s) + fitRight[3]);//[1]은 방향벡터 dy
		rp0.x = cvRound(fitRight[0] * (-s) + fitRight[2]);
		rp0.y = cvRound(fitRight[1] * (-s) + fitRight[3]);
		//line(inputImg, rp0, rp1, Scalar(0, 0, 255), LINE_4);

		//cout << "rp1.x : " << rp1.x << " rp1.y : " << rp1.y << "rp0.x : " << rp0.x << " rp0.y : " << rp0.y << endl;
		dydxRight = double(-fitRight[1]) / double(fitRight[0]);
		if (rightCorner_threshold != 0)
			rightCorner_threshold = 0;
	}
	else {
		dydxRight = 0;
		cout << "오른쪽 차선 검출 안됨 : ";
		rightCorner_threshold++;
		cout << rightCorner_threshold << endl;
		if (rightCorner_threshold >= 5)
			rightCorner_flag = true;
	} // 한쪽라인 인식 안되는 예외 처리 부분

	//값저장
	double angleThreshold = 5;// 10도 이하는 0으로만들기

	if (abs(atan(dydxLeft) + atan(dydxRight)) <= (angleThreshold * CV_PI / 180)) {
		headingAngle = 0;
	}
	else {
		headingAngle = -180 / CV_PI * (atan((dydxLeft)) + atan((dydxRight)));
	}
	// 차량이 바라보는 각도 값을 출력
	// steering값은 각도로 나오며 정면기준 0도임

	steering = -headingAngle; // 위에서 구한 차량이 바라보는 각도 값을 기준으로 차량이 향해야 할 조향각을 구한다.
							  // 양측 차선의 기울기를 기준으로 구한 차량이 취해야 할 조향각을 구한다.
							  // 아래의 조건문에서 검출되는 차선의 조건에 따라 조건문에서 최적화 진행


	if (right_index == 0 && left_index != 0) { // 우회전을 해야하는 경우 (왼쪽 차선만 검출된 경우)

		if (rightCorner_flag) // 일정 프레임동안 발견되지 않는 경우 좌회전으로 인식
		{					  // 임계값 이상의 프레임동안 왼쪽 차선만 검출되면 좌회전으로 인식하고 조향각을 크게 가져간다.

			outSteering = steering; // 한 방향의 차선만 검출될 경우에는 조향각을 그대로 줘서 코너링이 가능하게 한다.
			cout << "----------------------------우회전----------------------------" << endl;
		}
		else { // 임계값 이하의 프레임동안 왼쪽 차선만 검출되면 좀 더 완만한 조향각을 설정한다.
			outSteering = steering * 0.7;
		}
	}

	else if (right_index != 0 && left_index == 0) { // 좌회전을 해야하는 경우 (오른쪽 차선만 검출된 경우)

		if (leftCorner_flag) // 일정 프레임동안 발견되지 않는 경우 우회전으로 인식
		{					 // 임계값 이상의 프레임동안 왼쪽 차선만 검출되면 우회전으로 인식하고 조향각을 크게 가져간다.

			outSteering = steering; // 한 방향의 차선만 검출될 경우에는 조향각을 그대로 줘서 코너링이 가능하게 한다.
			cout << "----------------------------좌회전----------------------------" << endl;
		}
		else { // 임계값 이하의 프레임동안 왼쪽 차선만 검출되면 좀 더 완만한 조향각을 설정한다.
			outSteering = steering * 0.7;
		}
	}

	else if (left_index != 0 && right_index != 0) {
		outSteering = steering * 0.4; // 차선이 두 개 검출되는 직진 상황에서는 완만하게 움직여야 하기 때문에 가중치를 0.5로 함
	}

	cout << "차량 조향 각도 : " << outSteering << endl;

	slopeDegrees.clear();
	leftLines.clear();
	rightLines.clear();
	right_lines.clear();
	left_lines.clear();
	newPoint.clear();
	newLines.clear();
	return returnPt;
}
//void regionOfInterest(Mat& src, Mat& dst, Point* points) {// points의 포인터인 이유-> 여러개의 꼭짓점 경우
//
//	Mat maskImg = Mat::zeros(src.size(), CV_8UC1);
//
//	Scalar ignore_mask_color = Scalar(255, 255, 255);
//	const Point* ppt[1] = { points };//개의 꼭짓점 :n vertices
//	int npt[] = { 4 };
//
//	fillPoly(maskImg, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);
//	Mat maskedImg;
//	bitwise_and(src, maskImg, maskedImg);
//	dst = maskedImg;
//}
//
//void imgBlur(Mat& src, Mat& dst, int processingCode) {
//	if (processingCode == 1) {//gaussian Blur
//		GaussianBlur(src, dst, Size(3, 3), 0, 0);
//	}
//	else if (processingCode == 2) {//Canny edge
//		Canny(src, dst, 50, 150);
//	}
//
//}
bool extractLines(Mat& src, vector<Vec4i>& lines) {
	//Mat filterImg;
	//Mat grayImg, blurImg, edgeImg, roiImg, dstImg;
	//int width = src.size().width;
	//int height = src.size().height;
	//filter_colors(src, filterImg);
	//cvtColor(filterImg, grayImg, COLOR_BGR2GRAY);
	//imgBlur(grayImg, blurImg, 1);
	//imgBlur(blurImg, edgeImg, 2);
	//Point pt[4] = { Point(0,height * 2 / 5),Point(width,height * 2 / 5),Point(width,height * 6 / 7),Point(0,height * 6 / 7) };
	////roi point 설정

	//regionOfInterest(edgeImg, roiImg, pt);
	vector<Vec4i> extractLines;

	HoughLinesP(src, extractLines, 1, CV_PI / 180.0, 30, 10, 20);
	lines = extractLines;
	return true;
}

//void filter_colors(Mat& src, Mat& img_filtered) {
//	//
//	UMat bgrImg;
//	UMat hsvImg;
//	UMat maskWhite, whiteImg;
//	UMat maskYellow, yellowImg;
//	UMat imgCombined;
//	src.copyTo(bgrImg);
//
//	//white 변경
//	//inRange(bgrImg, lower_white, upper_white, maskWhite);
//	//lower와 upper사이의 값을 1로 나머지는 0으로 저장
//	//bitwise_and(bgrImg, bgrImg, whiteImg, maskWhite);
//
//	cvtColor(bgrImg, hsvImg, COLOR_BGR2HSV);
//	inRange(hsvImg, lower_yellow, upper_yellow, maskYellow);
//	bitwise_and(bgrImg, bgrImg, yellowImg, maskYellow);
//	//addWeighted(whiteImg, 1.0, yellowImg, 1.0, 0.0, imgCombined);//두 이미지 합치기
//	yellowImg.copyTo(imgCombined);;//노란색만 검출할때까지 사용
//	imgCombined.copyTo(img_filtered);
//}
