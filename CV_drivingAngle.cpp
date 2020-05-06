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
	vector<Vec4i> newLines;//�Ŀ� ���ʿ����� �ϳ��� �����
	const int width = inputImg.size().width;
	const int height = inputImg.size().height;
	vector<float> slopeDegrees;
	float slopeDegree;//�׻� �������� �����
	double preSteering = steering;//���� ���ҷ����� ���� ���Ⱒ�� ������
	float slopeThreshold = 0.3;//�׻� �������� ���� ��
	float headingAngle;
	//�ӽ÷� 1rad(56.6�� ����) �����ؼ� ��� ���� �� ����

	//vector point�� �����غ���
	vector<Point> newPoint;
	for (int k = 0; k < lines.size(); k++) {
		params = lines[k];
		x1 = params[0];
		y1 = params[1];
		//x1,y1�� ��
		x2 = params[2];
		y2 = params[3];
		//x2,y2�� ��

		pt1 = Point(x1, y1);
		pt2 = Point(x2, y2);

		if (x2 - x1 == 0)
			slopeDegree = 999;//x�� ��ȭ���� ���� ��� ���� 90���� �����
		else slopeDegree = (y2 - y1) / (float)(x2 - x1);
		//slope degree �� ���� ���������� ����
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
		//slope�� 0���� ũ�� pi/2+a rad���� �� ���̹Ƿ� �������� ��
		else if (slope < 0 && x1 < cx && x2 < cx)
			left_lines.push_back(line);
		//slope�� 0���� ������ pit/2-a rad���� �� ���̹Ƿ� ������ ����
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

	Point rp1, rp0;//������
	Point lp1, lp0;//����
	float s = 1000;//�� 
	double dydxLeft, dydxRight;//�� �ະ ���� ��
	//���� ���� ���ϴ� ����
	if (left_index > 0) {
		fitLine(leftLines, fitLeft, DIST_L2, 0, 0.01, 0.01);
		lp1.x = cvRound(fitLeft[0] * (+s) + fitLeft[2]);
		lp1.y = cvRound(fitLeft[1] * (+s) + fitLeft[3]);
		lp0.x = cvRound(fitLeft[0] * (-s) + fitLeft[2]);
		lp0.y = cvRound(fitLeft[1] * (-s) + fitLeft[3]);

		dydxLeft = double(-fitLeft[1]) / double(fitLeft[0]);
	}
	else { dydxLeft = 0; }//���ʶ��� �ν� �ȵǴ� ���� ó�� �κ�

	if (right_index > 0) {
		fitLine(rightLines, fitRight, DIST_L2, 0, 0.01, 0.01);
		rp1.x = cvRound(fitRight[0] * s + fitRight[2]);//[0]�� ���� ���� dx
		rp1.y = cvRound(fitRight[1] * s + fitRight[3]);//[1]�� ���⺤�� dy
		rp0.x = cvRound(fitRight[0] * (-s) + fitRight[2]);
		rp0.y = cvRound(fitRight[1] * (-s) + fitRight[3]);

		dydxRight = double(-fitRight[1]) / double(fitRight[0]);
	}
	else { dydxRight = 0; } // ���ʶ��� �ν� �ȵǴ� ���� ó�� �κ�

	//������
	double angleThreshold = 10;// 10�� ���ϴ� 0���θ����
	if (abs(atan(dydxLeft) + atan(dydxRight)) <= (angleThreshold * CV_PI / 180)) {
		headingAngle = 0;
	}
	else {
		headingAngle = -180 / CV_PI * (atan((dydxLeft)) + atan((dydxRight)));
	}
	//steering���� ������ ������ ������� 0����

	// �¿� �ν� �ȵǴ� ��� �˰��� �ν� �κ�(ȸ���� ���) -> ���� �����Ӱ� �ϼ���

	// ���� �⺻���� �˰��� ��steering = -headingAngle;
	//right_index=0�϶� ������ ����X
	//left_index=0�϶� �޼� ����
	//heading Angle�� ������ �ٶ󺸴� ����
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
	vector<Vec4i> newLines;//�Ŀ� ���ʿ����� �ϳ��� �����
	const int width = inputImg.size().width;
	const int height = inputImg.size().height;
	vector<float> slopeDegrees;
	float slopeDegree;//�׻� �������� �����
	double preSteering = steering;//���� ���ҷ����� ���� ���Ⱒ�� ������
	float slopeThreshold = 0.3;//�׻� �������� ���� ��
	float headingAngle;
	//�ӽ÷� 1rad(56.6�� ����) �����ؼ� ��� ���� �� ����

	//vector point�� �����غ���
	vector<Point> newPoint;
	for (int k = 0; k < lines.size(); k++) {
		params = lines[k];
		x1 = params[0];
		y1 = params[1];
		//x1,y1�� ��
		x2 = params[2];
		y2 = params[3];
		//x2,y2�� ��

		pt1 = Point(x1, y1);
		pt2 = Point(x2, y2);

		if (x2 - x1 == 0)
			slopeDegree = 999;//x�� ��ȭ���� ���� ��� ���� 90���� �����
		else slopeDegree = (y2 - y1) / (float)(x2 - x1);
		//slope degree �� ���� ���������� ����
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
		//slope�� 0���� ũ�� pi/2+a rad���� �� ���̹Ƿ� �������� ��
		else if (slope < 0 && x1 < cx && x2 < cx)
			left_lines.push_back(line);
		//slope�� 0���� ������ pit/2-a rad���� �� ���̹Ƿ� ������ ����
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

	Point rp1, rp0;//������
	Point lp1, lp0;//����
	float s = 1000;//�� 
	double dydxLeft, dydxRight;//�� �ະ ���� ��

	double left_interP = 0, right_interP = 0;
	double left_b, right_b;
	//���� ���� ���ϴ� ����
	if (left_index > 0) {
		fitLine(leftLines, fitLeft, DIST_L2, 0, 0.01, 0.01);
		lp1.x = cvRound(fitLeft[0] * (+s) + fitLeft[2]);
		lp1.y = cvRound(fitLeft[1] * (+s) + fitLeft[3]);
		lp0.x = cvRound(fitLeft[0] * (-s) + fitLeft[2]);
		lp0.y = cvRound(fitLeft[1] * (-s) + fitLeft[3]);

		dydxLeft = double(-fitLeft[1]) / double(fitLeft[0]);
		left_b = dydxLeft * lp1.x - lp1.y; // y = ax + b ���� b�� ���ϴ� ��
		left_interP = dydxLeft * width / 2 + left_b; // ������ �����Ŀ��� x���� x�� �߽��϶� y�� ��
	}
	else { dydxLeft = 0; }//���ʶ��� �ν� �ȵǴ� ���� ó�� �κ�

	if (right_index > 0) {
		fitLine(rightLines, fitRight, DIST_L2, 0, 0.01, 0.01);
		rp1.x = cvRound(fitRight[0] * s + fitRight[2]);//[0]�� ���� ���� dx
		rp1.y = cvRound(fitRight[1] * s + fitRight[3]);//[1]�� ���⺤�� dy
		rp0.x = cvRound(fitRight[0] * (-s) + fitRight[2]);
		rp0.y = cvRound(fitRight[1] * (-s) + fitRight[3]);

		dydxRight = double(-fitRight[1]) / double(fitRight[0]);
		right_b = dydxRight * rp1.x - rp1.y; // y = ax + b ���� b�� ���ϴ� ��
		right_interP = dydxRight * width / 2 + right_b; // ������ �����Ŀ��� x���� x�� �߽��϶� y�� ��
	}
	else { dydxRight = 0; } // ���ʶ��� �ν� �ȵǴ� ���� ó�� �κ�
	//������

	//steering���� ������ ������ ������� 0����
	////////////////////////////////////////////////////////////////////
	// ������ �κ�
	////////////////////////////////////////////////////////////////////
	double angleThreshold = 2.5;// 2.5�� ���ϴ� 0���θ����
	if (abs(atan(dydxLeft) + atan(dydxRight)) <= (angleThreshold * CV_PI / 180)) {
		headingAngle = 0;
	}
	else {
		headingAngle = -180 / CV_PI * (atan((dydxLeft)) + atan((dydxRight)));
	}

	double weight = 1.75; // steering�� ����ġ�� �༭ ���Ⱒ�� ������. Mode1���� ���

	if (Mode == 1) {
		// ������ �����ؼ� �ش� �������� ������ ���Ⱒ�� �����ص�.(������� �ش� ������ ������ ���Ⱒ�� ������)
		// ������ ������ ���� ���Ⱒ�� ������ �����.
		// steering�� ���� ����
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
		// steering�� ���� ����
		if ((right_index != 0) && (left_index != 0)) { // ������ �� ���� ��
			// steering �״��
		}
		else if (((right_index == 0) && (left_index != 0)) || ((right_index != 0) && (left_index == 0))) { // ������ �� ���� ��
			// steering �ݴ��
			steering = -steering;
		}
		else { // ������ ���� ��
			steering = steering_Before;
		}
	}
	else if (Mode == 2) {
		// �� �� ������ ����� ���� �� ������ �ݴ�� ũ�� �༭ ������� 0�� �� �� ���� �������� �ش�.
		// ������� 0�� �Ǹ� ���Ⱒ�� 0���� �ٲ��ش�.
		if ((left_interP > 0) || (right_interP > 0)) { // �� �� ���ο� ����� ���� ��
			if (left_interP > right_interP) // ������ ������ ����� ���� ��
				steering = -30;
			else // ���� ������ ����� ���� ��
				steering = 30;
		}
		else if (headingAngle == 0) { // ������� 0�� ��
			steering = 0;
		}
		else // �� �ܿ� ���� ���� ����
			steering = steering_Before;
	}
	else {
		cout << " Mode error" << endl;
	}
	// ���� �⺻���� �˰��� ��steering = -headingAngle;
	//right_index=0�϶� ������ ����X
	//left_index=0�϶� �޼� ����
	//heading Angle�� ������ �ٶ󺸴� ����
	cout << "steering: " << steering << endl;
	slopeDegrees.clear();
	leftLines.clear();
	rightLines.clear();
	right_lines.clear();
	left_lines.clear();
	newPoint.clear();
	newLines.clear();
}

void regionOfInterest(Mat& src, Mat& dst, Point* points) {// points�� �������� ����-> �������� ������ ���

	Mat maskImg = Mat::zeros(src.size(), CV_8UC1);

	Scalar ignore_mask_color = Scalar(255, 255, 255);
	const Point* ppt[1] = { points };//���� ������ :n vertices
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
	filter_colors(src, filterImg, lower_yellow, upper_yellow);
	cvtColor(filterImg, grayImg, COLOR_BGR2GRAY);
	imgBlur(grayImg, blurImg, 1);
	imgBlur(blurImg, edgeImg, 2);
	Point pt[4] = { Point(0,height * 2 / 5),Point(width,height * 2 / 5),Point(width,height * 6 / 7),Point(0,height * 6 / 7) };
	//roi point ����

	regionOfInterest(edgeImg, roiImg, pt);
	vector<Vec4i> extractLines;

	HoughLinesP(roiImg, extractLines, 1, CV_PI / 180.0, 30, 10, 20);
	lines = extractLines;
	return true;

}

void filter_colors(Mat& src, Mat& img_filtered, Scalar& lower, Scalar& upper) {
	//
	UMat bgrImg;
	UMat hsvImg;
	//UMat maskWhite, whiteImg;
	UMat maskYellow, yellowImg;
	UMat imgCombined;
	src.copyTo(bgrImg);

	//white ����
	//inRange(bgrImg, lower_white, upper_white, maskWhite);
	//lower�� upper������ ���� 1�� �������� 0���� ����
	//bitwise_and(bgrImg, bgrImg, whiteImg, maskWhite);

	cvtColor(bgrImg, hsvImg, COLOR_BGR2HSV);
	inRange(hsvImg, lower, upper, maskYellow);
	bitwise_and(bgrImg, bgrImg, yellowImg, maskYellow);
	//addWeighted(whiteImg, 1.0, yellowImg, 1.0, 0.0, imgCombined);//�� �̹��� ��ġ��
	yellowImg.copyTo(imgCombined);;//������� �����Ҷ����� ���
	imgCombined.copyTo(img_filtered);
}

double Steer::getSteering() {
	//����ġ�� ���� �׻� 1�� �ǵ����Ͽ�����. ����� �Ѵٸ� ���� ���� ������ ����ߵ�.
	double returnVal;
	bool goLeft = setLeftFlag >= MAX_SAVINGANGLE;
	bool goRight = setRightFlag >= MAX_SAVINGANGLE;
	bool goStraight = abs(currentHeading) < 20;
	if (goLeft) {//���� ���� �ν� X ��ȸ�� ��Ȳ
		if (LeftAngle[currentPos] != 0)//��ȸ�� ��Ȳ���� ���������� ���̴� ���
		{
			setStraightLeftFlag++;
		}
		else if (LeftAngle[currentPos] == 0) { setStraightLeftFlag--; }

		if (setStraightLeftFlag >= MAX_SAVINGANGLE) { //��ȸ������ �������� ��ȯ�Ǵ� ��Ȳ
			setLeftFlag = 0;
			setRightFlag = 0;
			setStraightLeftFlag = 0;
			returnVal = steering[predIdx(currentPos)];
		}
		else {
			returnVal = 0.5 * currentHeading + 0.5 * steering[predIdx(currentPos)];
		}
	}
	else if (goRight) {//���� ���� �ν� X ��ȸ�� ��Ȳ
		if (RightAngle[currentPos] != 0) { setStraightRightFlag++; }
		else if (RightAngle[currentPos] == 0) { setStraightRightFlag--; }

		if (setStraightRightFlag >= MAX_SAVINGANGLE) { //��ȸ������ �������� ��ȯ�Ǵ� ��Ȳ
			setLeftFlag = 0;
			setRightFlag = 0;
			setStraightLeftFlag = 0;
			returnVal = steering[predIdx(currentPos)];
		}
		else {//������ �ÿ���
			returnVal = 0.5 * currentHeading + 0.5 * steering[predIdx(currentPos)];
		}
	}
	else if (goStraight) {//���� ��Ȳ
		if (RightAngle[currentPos] == 0) { setRightFlag++; }
		else if (LeftAngle[currentPos] == 0) { setLeftFlag++; }
		else if (RightAngle[currentPos] != 0 && setRightFlag > 0) { setRightFlag--; }
		else if (LeftAngle[currentPos] != 0 && setLeftFlag > 0) { setLeftFlag--; }
		returnVal = (-currentHeading) * 2.0;//������ ������� �ݴ�� 1/2��ŭ
	}
	steering[currentPos] = returnVal;
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
	vector<Vec4i> newLines;//�Ŀ� ���ʿ����� �ϳ��� �����
	const int width = inputImg.size().width;
	const int height = inputImg.size().height;
	vector<float> slopeDegrees;
	float slopeDegree;//�׻� �������� �����
	double preSteering = steering;//���� ���ҷ����� ���� ���Ⱒ�� ������
	float slopeThreshold = 0.3;//�׻� �������� ���� ��
	float headingAngle;
	//�ӽ÷� 1rad(56.6�� ����) �����ؼ� ��� ���� �� ����

	//vector point�� �����غ���
	vector<Point> newPoint;
	for (int k = 0; k < lines.size(); k++) {
		params = lines[k];
		x1 = params[0];
		y1 = params[1];
		//x1,y1�� ��
		x2 = params[2];
		y2 = params[3];
		//x2,y2�� ��

		pt1 = Point(x1, y1);
		pt2 = Point(x2, y2);

		if (x2 - x1 == 0)
			slopeDegree = 999;//x�� ��ȭ���� ���� ��� ���� 90���� �����
		else slopeDegree = (y2 - y1) / (float)(x2 - x1);
		//slope degree �� ���� ���������� ����
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
		//slope�� 0���� ũ�� pi/2+a rad���� �� ���̹Ƿ� �������� ��
		else if (slope < 0 && x1 < cx && x2 < cx)
			left_lines.push_back(line);
		//slope�� 0���� ������ pit/2-a rad���� �� ���̹Ƿ� ������ ����
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

	Point rp1, rp0;//������
	Point lp1, lp0;//����
	float s = 1000;//�� 
	double dydxLeft, dydxRight;//�� �ະ ���� ��
	//���� ���� ���ϴ� ����
	if (left_index > 0) {
		fitLine(leftLines, fitLeft, DIST_L2, 0, 0.01, 0.01);
		lp1.x = cvRound(fitLeft[0] * (+s) + fitLeft[2]);
		lp1.y = cvRound(fitLeft[1] * (+s) + fitLeft[3]);
		lp0.x = cvRound(fitLeft[0] * (-s) + fitLeft[2]);
		lp0.y = cvRound(fitLeft[1] * (-s) + fitLeft[3]);

		dydxLeft = double(-fitLeft[1]) / double(fitLeft[0]);
	}
	else { dydxLeft = 0; }//���ʶ��� �ν� �ȵǴ� ���� ó�� �κ�

	if (right_index > 0) {
		fitLine(rightLines, fitRight, DIST_L2, 0, 0.01, 0.01);
		rp1.x = cvRound(fitRight[0] * s + fitRight[2]);//[0]�� ���� ���� dx
		rp1.y = cvRound(fitRight[1] * s + fitRight[3]);//[1]�� ���⺤�� dy
		rp0.x = cvRound(fitRight[0] * (-s) + fitRight[2]);
		rp0.y = cvRound(fitRight[1] * (-s) + fitRight[3]);

		dydxRight = double(-fitRight[1]) / double(fitRight[0]);
	}
	else { dydxRight = 0; } // ���ʶ��� �ν� �ȵǴ� ���� ó�� �κ�

	//������
	double angleThreshold = 10;// 10�� ���ϴ� 0���θ����
	if (abs(atan(dydxLeft) + atan(dydxRight)) <= (angleThreshold * CV_PI / 180)) {
		headingAngle = 0;
	}
	else {
		headingAngle = -180 / CV_PI * (atan((dydxLeft)) + atan((dydxRight)));
	}
	test.inputData(dydxRight, dydxLeft, headingAngle);
	steering = test.getSteering();
	//steering���� ������ ������ ������� 0����

	// �¿� �ν� �ȵǴ� ��� �˰��� �ν� �κ�(ȸ���� ���) -> ���� �����Ӱ� �ϼ���

	// ���� �⺻���� �˰��� ��steering = -headingAngle;
	//right_index=0�϶� ������ ����X
	//left_index=0�϶� �޼� ����
	//heading Angle�� ������ �ٶ󺸴� ����
	cout << "steering: " << steering << endl;
	slopeDegrees.clear();
	leftLines.clear();
	rightLines.clear();
	right_lines.clear();
	left_lines.clear();
	newPoint.clear();
	newLines.clear();
}



