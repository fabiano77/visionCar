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

		dydxLeft = double(-fitLeft[1]) / double(fitLeft[0]);
	}
	else { dydxLeft = 0; }//한쪽라인 인식 안되는 예외 처리 부분

	if (right_index > 0) {
		fitLine(rightLines, fitRight, DIST_L2, 0, 0.01, 0.01);
		rp1.x = cvRound(fitRight[0] * s + fitRight[2]);//[0]은 방향 벡터 dx
		rp1.y = cvRound(fitRight[1] * s + fitRight[3]);//[1]은 방향벡터 dy
		rp0.x = cvRound(fitRight[0] * (-s) + fitRight[2]);
		rp0.y = cvRound(fitRight[1] * (-s) + fitRight[3]);

		dydxRight = double(-fitRight[1]) / double(fitRight[0]);
	}
	else { dydxRight = 0; } // 한쪽라인 인식 안되는 예외 처리 부분

	//값저장
	double angleThreshold = 10;// 10도 이하는 0으로만들기
	if (abs(atan(dydxLeft) + atan(dydxRight)) <= (angleThreshold * CV_PI / 180)) {
		headingAngle = 0;
	}
	else {
		headingAngle = -180 / CV_PI * (atan((dydxLeft)) + atan((dydxRight)));
	}
	//steering값은 각도로 나오며 정면기준 0도임

	// 좌우 인식 안되는 경우 알고리즘 인식 부분(회전의 경우) -> 수정 자유롭게 하세요

	// 아주 기본적인 알고리즘 상steering = -headingAngle;
	//right_index=0일때 오른선 검출X
	//left_index=0일때 왼선 검출
	//heading Angle은 차량이 바라보는 방향
	cout << "steering: " << steering << endl;
	slopeDegrees.clear();
	leftLines.clear();
	rightLines.clear();
	right_lines.clear();
	left_lines.clear();
	newPoint.clear();
	newLines.clear();
}

void drivingAngle_SM(Mat& inputImg, vector<Vec4i> lines, double& steering, double& steering_Before, int Mode) {
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

	double left_interP = 0, right_interP = 0;
	double left_b, right_b;
	//방향 벡터 구하는 곳임
	if (left_index > 0) {
		fitLine(leftLines, fitLeft, DIST_L2, 0, 0.01, 0.01);
		lp1.x = cvRound(fitLeft[0] * (+s) + fitLeft[2]);
		lp1.y = cvRound(fitLeft[1] * (+s) + fitLeft[3]);
		lp0.x = cvRound(fitLeft[0] * (-s) + fitLeft[2]);
		lp0.y = cvRound(fitLeft[1] * (-s) + fitLeft[3]);

		dydxLeft = double(-fitLeft[1]) / double(fitLeft[0]);
		left_b = dydxLeft * lp1.x - lp1.y; // y = ax + b 에서 b를 구하는 식
		left_interP = dydxLeft * width / 2 + left_b; // 차선의 방정식에서 x값이 x축 중심일때 y의 값
	}
	else { dydxLeft = 0; }//한쪽라인 인식 안되는 예외 처리 부분

	if (right_index > 0) {
		fitLine(rightLines, fitRight, DIST_L2, 0, 0.01, 0.01);
		rp1.x = cvRound(fitRight[0] * s + fitRight[2]);//[0]은 방향 벡터 dx
		rp1.y = cvRound(fitRight[1] * s + fitRight[3]);//[1]은 방향벡터 dy
		rp0.x = cvRound(fitRight[0] * (-s) + fitRight[2]);
		rp0.y = cvRound(fitRight[1] * (-s) + fitRight[3]);

		dydxRight = double(-fitRight[1]) / double(fitRight[0]);
		right_b = dydxRight * rp1.x - rp1.y; // y = ax + b 에서 b를 구하는 식
		right_interP = dydxRight * width / 2 + right_b; // 라인의 방정식에서 x값이 x축 중심일때 y의 값
	}
	else { dydxRight = 0; } // 한쪽라인 인식 안되는 예외 처리 부분
	//값저장

	//steering값은 각도로 나오며 정면기준 0도임
	////////////////////////////////////////////////////////////////////
	// 수정된 부분
	////////////////////////////////////////////////////////////////////
	double angleThreshold = 2.5;// 2.5도 이하는 0으로만들기
	if (abs(atan(dydxLeft) + atan(dydxRight)) <= (angleThreshold * CV_PI / 180)) {
		headingAngle = 0;
	}
	else {
		headingAngle = -180 / CV_PI * (atan((dydxLeft)) + atan((dydxRight)));
	}

	double weight = 1.75; // steering에 가중치를 줘서 조향각을 맞춰줌. Mode1에서 사용

	if (Mode == 1) {
		// 범위를 지정해서 해당 범위마다 일정한 조향각을 설정해둠.(헤딩각이 해당 범위에 들어오면 조향각이 설정됨)
		// 차선의 개수에 따라 조향각의 방향을 잡아줌.
		// steering의 각도 조정
		if (headingAngle == 0)
			steering = 0;
		else if (abs(headingAngle) <= 10) {
			if (headingAngle > 0)
				steering = 5;
			else
				steering = -5;
		}
		else if (abs(headingAngle) <= 20) {
			if (headingAngle > 0)
				steering = 10;
			else
				steering = -10;
		}
		else if (abs(headingAngle) <= 30) {
			if (headingAngle > 0)
				steering = 15;
			else
				steering = -15;
		}
		else {
			if (headingAngle > 0)
				steering = 20;
			else
				steering = -20;
		}
		steering *= weight;
		// steering의 방향 조정
		//if ((right_index != 0) && (left_index != 0)) { // 차선이 두 개일 때
			// steering 그대로
		//}
		//else if (((right_index == 0) && (left_index != 0)) || ((right_index != 0) && (left_index == 0))) { // 차선이 한 개일 때
			// steering 반대로
		//	steering = -steering;
		//}
		//else { // 차선이 없을 때
		//	steering = steering_Before;
		//}
		if ((right_index == 0) && (left_index == 0))
			steering = steering_Before;
		else
			steering = -steering;
	}
	else if (Mode == 2) 
	{
		// 한 쪽 차선에 가까워 졌을 때 각도를 반대로 크게 줘서 헤딩각이 0이 될 때 까지 유지시켜 준다.
		// 헤딩각이 0이 되면 조향각을 0으로 바꿔준다.
		if (headingAngle != 0)
		{
			if ((left_interP > 0) || (right_interP > 0)) 
			{ // 한 쪽 라인에 가까워 졌을 때
			        if (left_interP > right_interP) // 오른쪽 차선에 가까워 졌을 때
				    steering = -30;
			        else // 왼쪽 차선에 가까워 졌을 때
				    steering = 30;
		        }
			else{
				steering = steering_Before;
			}
			/*if ((right_index != 0) && (left_index == 0)) {
				steering = -30;
			}
			else if ((right_index == 0) && (left_index != 0)) {
				steering = -30;
			}
			else {
				steering = steering_Before;
			}*/
		}
		else {
			steering = 0;
		}
	}	
	// 아주 기본적인 알고리즘 상steering = -headingAngle;
	//right_index=0일때 오른선 검출X
	//left_index=0일때 왼선 검출
	//heading Angle은 차량이 바라보는 방향
	cout << "headingAngle: " << headingAngle << endl;
	cout << "steering: " << steering << endl << endl;
	
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
	//UMat maskWhite, whiteImg;
	UMat maskYellow, yellowImg;
	UMat imgCombined;
	src.copyTo(bgrImg);

	//white 변경
	//inRange(bgrImg, lower_white, upper_white, maskWhite);
	//lower와 upper사이의 값을 1로 나머지는 0으로 저장
	//bitwise_and(bgrImg, bgrImg, whiteImg, maskWhite);
	Scalar lower_w = Scalar(120, 120, 120); //흰색 차선 (RGB)
	Scalar upper_w = Scalar(255, 255, 255);
	Scalar lower_y(14, 30, 35);
	Scalar upper_y(46, 255, 255);


	cvtColor(bgrImg, hsvImg, COLOR_BGR2HSV);
	inRange(hsvImg, lower_y, upper_y, maskYellow);
	bitwise_and(bgrImg, bgrImg, yellowImg, maskYellow);
	//addWeighted(whiteImg, 1.0, yellowImg, 1.0, 0.0, imgCombined);//두 이미지 합치기
	yellowImg.copyTo(imgCombined);;//노란색만 검출할때까지 사용
	imgCombined.copyTo(img_filtered);
}

double Steer::getSteering() {

	//가중치의 합은 항상 1이 되도록하여야함. 벗어나야 한다면 값의 누적 적용을 낮춰야됨.
	double returnVal;
	bool goLeft = setLeftFlag >= MAX_SAVINGANGLE;
	bool goRight = setRightFlag >= MAX_SAVINGANGLE;
	const int turnAngleThreshold = 20;
	bool goStraight = abs(currentHeading) < turnAngleThreshold;
	if (LeftAngle[currentPos] == 0 && RightAngle[currentPos] == 0) {//둘다 인식이 안되는 경우
		returnVal = steering[predIdx(currentPos)];
	}
	else if (goLeft) {//좌측 라인 인식 X 좌회전 상황
		if (LeftAngle[currentPos] != 0)//좌회전 상황에서 왼쪽차선이 보이는 경우
		{
			setStraightLeftFlag++; //왼쪽에서 직진변환 플래그 증가
		}
		else if (LeftAngle[currentPos] == 0) {
			if (setStraightLeftFlag > 0)//0초과 일때만 플래그 감소 //이상이면 돌아오지 않을수있음
				setStraightLeftFlag--;
		} //왼쪽에서 직진 변환 플래그 감소

		if (setStraightLeftFlag >= MAX_SAVINGANGLE) { //좌회전에서 직진으로 변환되는 상황
			setLeftFlag = 0;
			setRightFlag = 0;
			setStraightLeftFlag = 0;
			//returnVal = steering[predIdx(currentPos)]; 
			returnVal = -25;
		}
		else {
			//returnVal = 0.5 * currentHeading + 0.5 * steering[predIdx(currentPos)];
			returnVal = -40;//좌회전은 확실히 각을 꺾게 만듦
		}
		cout << "좌회전";
	}
	else if (goRight) {//우측 라인 인식 X 우회전 상황
		if (RightAngle[currentPos] != 0) { setStraightRightFlag++; }
		else if (RightAngle[currentPos] == 0) { setStraightRightFlag--; }

		if (setStraightRightFlag >= MAX_SAVINGANGLE) { //우회전에서 직진으로 변환되는 상황
			setLeftFlag = 0;
			setRightFlag = 0;
			setStraightLeftFlag = 0;
			//returnVal = steering[predIdx(currentPos)];
			returnVal = 25;//돌아올때 충격완화
		}
		else {//정상일 시에는
			returnVal = 40;
			//returnVal = 0.5 * currentHeading + 0.5 * steering[predIdx(currentPos)];
		}
		cout << "우회전";
	}
	else if (goStraight) {//직진 상황
		if (RightAngle[currentPos] == 0) { setRightFlag++; }//직진이었는데 뭔가 쎄할 때
		else if (LeftAngle[currentPos] == 0) { setLeftFlag++; }
		else if (RightAngle[currentPos] != 0 && setRightFlag > 0) { setRightFlag--; }
		else if (LeftAngle[currentPos] != 0 && setLeftFlag > 0) { setLeftFlag--; }
		if (abs(currentHeading) > 10) { returnVal = -currentHeading / 2.0; }//오차범위 이내일 경우직진시 헤딩방향 반대로 가게
		else returnVal = 0;
		cout << "직진";
	}
	else if (!goStraight) {
		//직진이 아닌거 같을 때 연산 방식
		if (RightAngle[currentPos] == 0) { setRightFlag++; }
		else if (LeftAngle[currentPos] == 0) { setLeftFlag++; }
		else if (RightAngle[currentPos] != 0 && setRightFlag > 0) { setRightFlag--; }
		else if (LeftAngle[currentPos] != 0 && setLeftFlag > 0) { setLeftFlag--; }
		returnVal = -currentHeading / 4.0;
		cout << "방향각 조정중";
	}
	steering[currentPos] = returnVal;
	cout << "/ 바퀴 조향각 : " << steering[currentPos] << endl;
	return steering[currentPos];
}
Steer::Steer() {
	for (int i = 0; i < MAX_SAVINGANGLE; i++) {
		RightAngle[i] = 0;
		LeftAngle[i] = 0;
		steering[i] = 0;
	}
	currentPos = 0;
}
void Steer::inputData(double dydxRight, double dydxLeft, double currentHead) {
	currentPos = nextIdx(currentPos);
	RightAngle[currentPos] = dydxRight;
	LeftAngle[currentPos] = dydxLeft;
	currentHeading = currentHead;
}
int Steer::nextIdx(int pos) {
	if (pos < MAX_SAVINGANGLE - 1) { return pos + 1; }
	else return 0;
}
int Steer::predIdx(int pos) {
	if (pos <= 0) { return MAX_SAVINGANGLE - 1; }
	else return pos--;
}

bool Steer::gostop() {
	if (RightAngle[currentPos] == 0 && LeftAngle[currentPos] == 0) {
		stopFlag++;
	}
	else { if (stopFlag > 0)stopFlag--; }
	cout << "정지 예고(10회시 종료): " << stopFlag << endl;
	if (stopFlag >= 10) { return false; }
	else true;
}




void imgProcessing(Mat& src, Mat& dst, int processingCode) {
	if (processingCode == 1) {//gaussian Blur
		GaussianBlur(src, dst, Size(3, 3), 0, 0);
	}
	else if (processingCode == 2) {//Canny edge
		Canny(src, dst, 70, 150);
	}

}
void drivingAngle_MS(Mat& inputImg, vector<Vec4i> lines, double& steering,Steer& test) {
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

		dydxLeft = double(-fitLeft[1]) / double(fitLeft[0]);
	}
	else { dydxLeft = 0; }//한쪽라인 인식 안되는 예외 처리 부분

	if (right_index > 0) {
		fitLine(rightLines, fitRight, DIST_L2, 0, 0.01, 0.01);
		rp1.x = cvRound(fitRight[0] * s + fitRight[2]);//[0]은 방향 벡터 dx
		rp1.y = cvRound(fitRight[1] * s + fitRight[3]);//[1]은 방향벡터 dy
		rp0.x = cvRound(fitRight[0] * (-s) + fitRight[2]);
		rp0.y = cvRound(fitRight[1] * (-s) + fitRight[3]);

		dydxRight = double(-fitRight[1]) / double(fitRight[0]);
	}
	else { dydxRight = 0; } // 한쪽라인 인식 안되는 예외 처리 부분

	//값저장
	double angleThreshold = 10;// 10도 이하는 0으로만들기
	if (abs(atan(dydxLeft) + atan(dydxRight)) <= (angleThreshold * CV_PI / 180)) {
		headingAngle = 0;
	}
	else {
		headingAngle = -180 / CV_PI * (atan((dydxLeft)) + atan((dydxRight)));
	}
	test.inputData(dydxRight, dydxLeft, headingAngle);
	steering = test.getSteering();
	//steering값은 각도로 나오며 정면기준 0도임

	// 좌우 인식 안되는 경우 알고리즘 인식 부분(회전의 경우) -> 수정 자유롭게 하세요

	// 아주 기본적인 알고리즘 상steering = -headingAngle;
	//right_index=0일때 오른선 검출X
	//left_index=0일때 왼선 검출
	//heading Angle은 차량이 바라보는 방향
	cout << "steering: " << steering << endl;
	slopeDegrees.clear();
	leftLines.clear();
	rightLines.clear();
	right_lines.clear();
	left_lines.clear();
	newPoint.clear();
	newLines.clear();
}
