#include "DetectColorSign.h"

//createTrackbar("circleSize", "trackbar", &circleSize, 500, on_trackbar);
//namedWindow("trackbar", WINDOW_NORMAL);
//circle(distortedFrame, Point(imageSize.width * 0.5, imageSize.height * 0.3), circleSize, Scalar(0, 0, 255), -1, LINE_AA);

int DCS_HLP_threshold = 75;
int DCS_HLP_minLineLength = 15;
int DCS_HLP_maxLineGap = 5;

int DCS_HLP_threshold_2 = 30;
int DCS_HLP_minLineLength_2 = 15;
int DCS_HLP_maxLineGap_2 = 5;

int H_minDist = 20;
int H_param1 = 50;
int H_param2 = 35;

static void on_trackbar(int, void*)
{
}

DetectColorSign::DetectColorSign()
{
	for (int i = 0; i < 10; i++)
	{
		pre_brightness[i] = 0;
	}
	ready = false;
	print = true;
	waiting = true;
	startCount = 0;
	lower_red1 = Scalar(0, 100, 200);
	upper_red1 = Scalar(12, 255, 255);
	lower_red2 = Scalar(168, 100, 200);
	upper_red2 = Scalar(180, 255, 255);

	lower_yellow = Scalar(18, 100, 200);
	upper_yellow = Scalar(42, 255, 255);

	lower_green = Scalar(45, 70, 150);
	upper_green = Scalar(75, 255, 255);
}
DetectColorSign::DetectColorSign(bool onPrint)
{
	*(this) = DetectColorSign();
	print = onPrint;
}

bool DetectColorSign::detectTunnel(Mat& frame, double percent)
{
	bool returnVal;
	Mat grayFrame;
	cvtColor(frame, grayFrame, COLOR_RGB2GRAY);

	int pixelCnt(0);
	int pixelValue(0);
	for (int i = 0; i < grayFrame.cols; i += 10)				// 10�ȼ����� �ϳ��� �˻��� �ӵ��� ����
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
		putText(grayFrame, "detect tunnel!", Point(frame.cols / 4, frame.rows * 0.65), FONT_HERSHEY_COMPLEX, 1, Scalar(255), 2);
		returnVal = true;
	}
	else returnVal = false;


	if (print)
	{
		putText(grayFrame, "darkRate : " + to_string(100 - brightRate) + '%', Point(30, 30), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0), 2);
		namedWindow("grayFrame", WINDOW_NORMAL);
		imshow("grayFrame", grayFrame);
		resizeWindow("grayFrame", 320, 240);
		moveWindow("grayFrame", 0, 40);
	}

	return returnVal;
}

bool DetectColorSign::waitingCheck(Mat& frame, double difference)
{
	if (!waiting)
		return false;

	double sum(0), average(0);
	if (ready)
	{
		for (int i = 0; i < 10; i++)
		{
			sum += pre_brightness[0];
		}
		average = sum / 10;
	}
	else if (pre_brightness[9] > 0)
		ready = true;

	Mat grayFrame;
	cvtColor(frame, grayFrame, COLOR_RGB2GRAY);

	int pixelCnt(0);
	int pixelValue(0);
	for (int i = 0; i < grayFrame.cols; i += 10)				// 10�ȼ����� �ϳ��� �˻��� �ӵ��� ����
	{
		for (int j = 0; j < grayFrame.rows; j += 10)
		{
			pixelValue += grayFrame.at<uchar>(j, i);
			pixelCnt++;
		}
	}
	int totalValue = pixelCnt * 255;
	double brightRate = ((double)pixelValue / totalValue) * 100.0;

	if (ready && (brightRate - average > difference || brightRate - average < -difference))
	{
		startCount++;
		if (startCount >= 5)
		{
			waiting = false;
			return false;
		}
	}
	else
	{
		pre_brightness[0] = brightRate;
		for (int i = 0; i < 9; i++)
		{
			pre_brightness[i + 1] = pre_brightness[i];
		}
	}

	putText(frame, "wating~~", Point(frame.cols / 4, frame.rows * 0.65), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 153, 0), 2);

	return true;
}

bool DetectColorSign::priorityStop(Mat& frame, double percent)
{
	bool returnVal;
	if (isRedStop(frame, percent))
	{
		//Ŭ��������� ����Ǿ��ִ� frame_red Ȱ��.
		Mat frame_red = store_red;
		Mat frame_edge;
		Canny(frame_red, frame_edge, 118, 242);	//�������� ���� frame�� ������ 1ä�� Mat��ü�� ����

		vector<Vec4i> lines;		//����� ������ ����� ��ü
		HoughLinesP(frame_edge, lines, 1, CV_PI / 180, DCS_HLP_threshold, DCS_HLP_minLineLength, DCS_HLP_maxLineGap);


		if (lines.size() >= 2) returnVal = true;
		else returnVal = false;

		if (print)
		{
			putText(frame_red, "red Pixel : " + to_string(m_redRatio) + '%', Point(30, 30), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0), 2);
			putText(frame_red, "Line Count : " + to_string(lines.size()), Point(30, 60), FONT_HERSHEY_COMPLEX, 1, Scalar(255), 2);
			putText(frame_red, "Priority STOP signal!", Point(frame.cols / 4, frame.rows * 0.65), FONT_HERSHEY_COMPLEX, 1, Scalar(255), 2);
			namedWindow("frame_red", WINDOW_NORMAL);
			imshow("frame_red", frame_red);
			resizeWindow("frame_red", 320, 240);
			moveWindow("frame_red", 0, 40);
		}
	}
	else returnVal = false;

	return returnVal;
}

bool DetectColorSign::isRedStop(Mat& frame, double percent)
{
	//Mat frame_hsv;
	//Mat frame_red1;
	//Mat frame_red2;

	bool returnVal;
	Mat frame_red;	//�ʱ�ȭ

	cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
	inRange(frame_hsv, lower_red1, upper_red1, frame_red1);
	inRange(frame_hsv, lower_red2, upper_red2, frame_red2);
	frame_red = frame_red1 | frame_red2;

	store_red = frame_red.clone();

	int redPixel(0);
	int height_limit = frame_red.rows / 2;	//ȭ�� ���� 50% �� �˻�
	for (int i = 0; i < frame_red.cols; i += 10)				// 10�ȼ����� �ϳ��� �˻��� �ӵ��� ����
	{
		for (int j = 0; j < height_limit; j += 10)
		{
			if (frame_red.at<uchar>(j, i))	redPixel++;		// �������� ������ �ȼ� ���� ���
		}
	}

	//double redRatio = ((double)redPixel / ((frame.cols) * (frame.rows)));	//����� �ȼ����� ��ü �ȼ����� ���� ����
	double redRatio = ((double)redPixel / ((frame.cols / 10) * (frame.rows / 10)));	//����� �ȼ����� ��ü �ȼ����� ���� ����
	redRatio *= 100;
	m_redRatio = redRatio;

	if (redRatio > percent)
	{
		putText(frame_red, "Red stop signal!", Point(frame.cols / 4, frame.rows * 0.65), FONT_HERSHEY_COMPLEX, 1, Scalar(255), 2);
		returnVal = true;
	}
	else
	{
		returnVal = false;
	}

	if (print)
	{
		putText(frame_red, "red Pixel : " + to_string(redRatio) + '%', Point(30, 30), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0), 2);
		namedWindow("frame_red", WINDOW_NORMAL);
		imshow("frame_red", frame_red);
		resizeWindow("frame_red", 320, 240);
		moveWindow("frame_red", 0, 40);
	}

	return returnVal;
}

bool DetectColorSign::isYellow(Mat& frame, double percent)
{
	bool returnVal;

	Mat frame_yellow;
	cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
	inRange(frame_hsv, lower_yellow, upper_yellow, frame_yellow);

	int yellowPixel(0);
	int height_limit = frame_yellow.rows / 2;		//ȭ�� ���� 50% �� �˻�
	for (int i = 0; i < frame_yellow.cols; i += 10)				// 10�ȼ����� �ϳ��� �˻��� �ӵ��� ����
	{
		for (int j = 0; j < height_limit; j += 10)
		{
			if (frame_yellow.at<uchar>(j, i))	yellowPixel++;		// ������� ������ �ȼ� ���� ���
		}
	}
	//640 x 480 �̹Ƿ� 64x24 = 1536 �ȼ��� �˻�.
	//��ü�ȼ� 640 x 480 = 307,200
	//�����ȼ� 64 x 48 = 3072

	//double yellowRatio = ((double)yellowPixel / ((frame.cols) * (frame.rows)));	//����� �ȼ����� ��ü �ȼ����� ���� ����
	double yellowRatio = ((double)yellowPixel / ((frame.cols / 10) * (frame.rows / 10)));	//����� �ȼ����� ��ü �ȼ����� ���� ����
	yellowRatio *= 100;

	if (yellowRatio > percent)
	{
		putText(frame_yellow, "yellow signal!", Point(frame.cols / 4, frame.rows * 0.65), FONT_HERSHEY_COMPLEX, 1, Scalar(255), 2);
		returnVal = true;
	}
	else
	{
		returnVal = false;
	}

	if (print)
	{
		putText(frame_yellow, "yellow Pixel : " + to_string(yellowRatio) + '%', Point(30, 30), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0), 2);
		namedWindow("frame_yellow", WINDOW_NORMAL);
		imshow("frame_yellow", frame_yellow);
		resizeWindow("frame_yellow", 320, 240);
		moveWindow("frame_yellow", 0 + 320, 40);
	}

	return returnVal;
}

int DetectColorSign::isGreenTurnSignal(Mat& frame, double percent)
{
	//Mat frame_hsv;
	int returnVal;

	Mat frame_green;
	Mat frame_edge;
	cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
	inRange(frame_hsv, lower_green, upper_green, frame_green);
	Canny(frame_green, frame_edge, 118, 242);	//�ʷϻ��� ���� frame�� ������ 1ä�� Mat��ü�� ����

	int greenPixel(0);
	int height_limit = frame_green.rows / 2;		//ȭ�� ���� 50% �� �˻�
	for (int i = 0; i < frame_green.cols; i += 10)				// 10�ȼ����� �ϳ��� �˻��� �ӵ��� ����
	{
		for (int j = 0; j < height_limit; j += 10)
		{
			if (frame_green.at<uchar>(j, i))	greenPixel++;		// �ʷϻ��� ������ �ȼ� ���� ���
		}
	}
	//640 x 480 �̹Ƿ� 64x24 = 1536 �ȼ��� �˻�.

	//double greenRatio = ((double)greenPixel / ((frame.cols) * (frame.rows)));	//����� �ȼ����� ��ü �ȼ����� ���� ����
	double greenRatio = ((double)greenPixel / ((frame.cols / 10) * (frame.rows / 10)));	//����� �ȼ����� ��ü �ȼ����� ���� ����
	greenRatio *= 100;

	vector<Vec4i> lines;		//����� ������ ����� ��ü
	HoughLinesP(frame_edge, lines, 1, CV_PI / 180, DCS_HLP_threshold_2, DCS_HLP_minLineLength_2, DCS_HLP_maxLineGap_2);

	if (greenRatio > percent)
	{
		if (lines.size() >= 3)
		{
			if (print)putText(frame_green, "Green LEFT signal", Point(frame.cols / 5, frame.rows * 0.65), FONT_HERSHEY_COMPLEX, 1, Scalar(255), 2);
			returnVal = 1;
		}
		else
		{
			if (print)putText(frame_green, "Green RIGHT signal", Point(frame.cols / 5, frame.rows * 0.65), FONT_HERSHEY_COMPLEX, 1, Scalar(255), 2);
			returnVal = 2;
		}
	}
	else
	{
		returnVal = 0;
	}

	if (print)
	{
		putText(frame_green, "green Pixel : " + to_string(greenRatio) + '%', Point(30, 30), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0), 2);
		putText(frame_green, "Line Count : " + to_string(lines.size()), Point(30, 60), FONT_HERSHEY_COMPLEX, 1, Scalar(255), 2);
		for (Vec4i l : lines)
		{
			line(frame_green, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2);
		}
		namedWindow("frame_green", WINDOW_NORMAL);
		imshow("frame_green", frame_green);
		resizeWindow("frame_green", 320, 240);
		moveWindow("frame_green", 0 + 320 + 320, 40);
	}

	return returnVal;
}
