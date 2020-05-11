#include "DetectColorSign.h"

//createTrackbar("circleSize", "trackbar", &circleSize, 500, on_trackbar);
//namedWindow("trackbar", WINDOW_NORMAL);
//circle(distortedFrame, Point(imageSize.width * 0.5, imageSize.height * 0.3), circleSize, Scalar(0, 0, 255), -1, LINE_AA);

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
	*(this) = DetectColorSign();
	print = onPrint;
}

bool DetectColorSign::isRedStop(Mat& frame, int percent)
{
	//Mat frame_hsv;
	//Mat frame_red;
	//Mat frame_red1;
	//Mat frame_red2;

	cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
	inRange(frame_hsv, lower_red1, upper_red1, frame_red1);
	inRange(frame_hsv, lower_red2, upper_red2, frame_red2);
	frame_red = frame_red1 | frame_red2;

	int height_limit = frame_red.rows * 0.6;

	int redPixel(0);

	for (int i = 0; i < frame_red.cols; i += 10)				// 10픽셀마다 하나씩 검사함 속도를 위해
	{
		for (int j = 0; j < height_limit; j += 10)
		{
			if (frame_red.at<uchar>(j, i))	redPixel++;		// 붉은색이 나오는 픽셀 개수 계산
		}
	}
	//640 x 480 이므로 64x48 = 3078 픽셀만 검사.

	double redRatio = (double)redPixel / (frame_red.total() / 100.0);	//검출된 픽셀수를 전체 픽셀수로 나눈 비율

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