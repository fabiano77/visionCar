#include "DetectColorSign.h"

DetectColorSign::DetectColorSign()
{
	print = true;
	lower_red1 = Scalar(0, 100, 100);
	upper_red1 = Scalar(15, 255, 255);
	lower_red2 = Scalar(165, 100, 100);
	upper_red2 = Scalar(180, 255, 255);
}
DetectColorSign::DetectColorSign(bool onPrint)
{
	print = onPrint;
	lower_red1 = Scalar(0, 100, 100);
	upper_red1 = Scalar(15, 255, 255);
	lower_red2 = Scalar(165, 100, 100);
	upper_red2 = Scalar(180, 255, 255);
}

bool DetectColorSign::isRedStop(Mat& frame, int percent)
{
	Mat frame_hsv;
	Mat frame_red;
	Mat frame_red1;
	Mat frame_red2;

	cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
	inRange(frame_hsv, lower_red1, upper_red1, frame_red1);
	inRange(frame_hsv, lower_red2, upper_red2, frame_red2);
	frame_red = frame_red1 | frame_red2;

	int redPixel(0);

	for (int i = 0; i < frame_red.cols; i += 10)				// 10�ȼ����� �ϳ��� �˻��� �ӵ��� ����
	{
		for (int j = 0; j < frame_red.rows; j += 10)
		{
			if (frame_red.at<uchar>(j, i))	redPixel++;		// �������� ������ �ȼ� ���� ���
		}
	}

	double redRatio = (double)redPixel / (frame_red.total() / 100.0);	//����� �ȼ����� ��ü �ȼ����� ���� ����

	if (print) putText(frame, "redRatio : " + to_string(redRatio), Point(30, 30), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0), 2);

	if (redRatio > (percent / 100.0))
	{
		putText(frame, "detected red stop!", Point(frame.cols / 5, frame.rows / 2), FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 255), 2);
		return true;
	}
	else
	{
		return false;
	}
}