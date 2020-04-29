#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class ImageProcessing {
public:
	ImageProcessing();
	ImageProcessing(Mat& inputImg);
	void insert(Mat& insertImg);
	bool drivingAngle(vector<Vec4i>lines, double& steering);
	void filter_colors_hsv(Mat& img_filtered);//for show hsv
	void filter_colors_bgr(Mat& img_filtered);
private:
	void filter_colors();
	Mat primaryImg;
	Mat grayImg;
	Mat hsvImg;
	int width;
	int height;
	vector<Vec4i> lines;
};

ImageProcessing::ImageProcessing() {
	primaryImg = 0;
	grayImg = 0;
	hsvImg = 0;
	int width = 0;
	int height;
}

ImageProcessing::ImageProcessing(Mat&inputImg){
	primaryImg = inputImg;
	grayImg=cvtColor
}