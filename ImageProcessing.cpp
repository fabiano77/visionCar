#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
#include "ImageProcessing_Constants.h"

using namespace cv;
using namespace std;

class ImageProcessing {
public:
	ImageProcessing();
	ImageProcessing(Mat& inputImg);
	void insert(Mat& inputImg);
	bool calibImage(Mat& inputChessImg);
	bool drivingAngle(double& steering);
	void filter_colors_hsv(Mat& img_filtered);//for show hsv
	void filter_colors_bgr(Mat& img_filtered);
private:
	void filter_colors();
	bool extractLines();
	void imgBlur(Mat& src, Mat& dst, int processingCode);
	void regionOfInterest();
	Mat primaryImg;//�Է� �̹��� �״��
	Mat roiImg; // roi������ ��ŭ�� �̹���
	Mat filteredImg; // colorfiltered �̹���
	Mat edgeImg;
	Mat intrinsic;
	Mat distCoeffs;

	Point pt[numPoint] = { Point(0, height * 2 / 5), Point(width, height * 2 / 5), Point(width, height * 6 / 7), Point(0, height * 6 / 7) };
	Size imgSize;
	int width;
	int height;
	vector<Vec4i> lines;
};

ImageProcessing::ImageProcessing() {
	intrinsic = Mat(3, 3, CV_32FC1);
	distCoeffs = 0;
	primaryImg = 0;
	filteredImg = 0;
	int width = 0;
	int height = 0;
}

ImageProcessing::ImageProcessing(Mat& inputImg) {
	primaryImg.copyTo(inputImg);
	width = inputImg.size().width;
	height = inputImg.size().height;
	imgSize = inputImg.size();
	//roi point ����
	filter_colors();
}
void ImageProcessing::insert(Mat& inputImg) {
	primaryImg.copyTo(inputImg);

	if (width == 0 && height == 0) {
		width = inputImg.size().width;
		height = inputImg.size().height;
		imgSize = inputImg.size();
	}
	filter_colors();
}
void ImageProcessing::filter_colors() {
	UMat bgrImg;
	UMat hsvImg;
	UMat maskWhite, whiteImg;
	UMat maskYellow, yellowImg;
	UMat imgCombined;
	primaryImg.copyTo(bgrImg);

	//white ����
	inRange(bgrImg, lower_white, upper_white, maskWhite);
	//lower�� upper������ ���� 1�� �������� 0���� ����
	bitwise_and(bgrImg, bgrImg, whiteImg, maskWhite);

	cvtColor(bgrImg, hsvImg, COLOR_BGR2HSV);
	inRange(hsvImg, lower_yellow, upper_yellow, maskYellow);
	bitwise_and(bgrImg, bgrImg, yellowImg, maskYellow);
	addWeighted(whiteImg, 1.0, yellowImg, 1.0, 0.0, imgCombined);//�� �̹��� ��ġ��

	imgCombined.copyTo(filteredImg);
}

bool ImageProcessing::drivingAngle(double steering) {

}



bool ImageProcessing::extractLines() {
	Mat filterImg;
	Mat grayImg, blurImg, dstImg;
	cvtColor(filterImg, grayImg, COLOR_BGR2GRAY);
	imgBlur(grayImg, blurImg, 1);
	imgBlur(blurImg, edgeImg, 2);
	//roi point ����
	regionOfInterest();
	vector<Vec4i> extractLines;
	HoughLinesP(roiImg, extractLines, 1, CV_PI / 180.0, 30, 10, 20);
	lines = extractLines;
	return true;
}

void ImageProcessing::imgBlur(Mat& src, Mat& dst, int processingCode) {
	if (processingCode == 1) {//gaussian Blur
		GaussianBlur(src, dst, Size(3, 3), 0, 0);
	}
	else if (processingCode == 2) {//Canny edge
		Canny(src, dst, 50, 150);
	}

}

void ImageProcessing::regionOfInterest() {// points�� �������� ����-> �������� ������ ���

	Mat maskImg = Mat::zeros(imgSize, CV_8UC1);

	Scalar ignore_mask_color = Scalar(255, 255, 255);
	Scalar red = Scalar(0, 0, 255);
	const Point* ppt[1] = { pt };//���� ������ :n vertices
	int npt[] = { 4 };

	fillPoly(maskImg, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);
	Mat maskedImg;
	bitwise_and(edgeImg, maskImg, maskedImg);
	maskedImg.copyTo(roiImg);
}

int main() {
	return 0;
}