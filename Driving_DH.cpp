#include "Driving_DH.h"

const bool PRINT(true);				// ���� ��� ǥ�� on&off
const bool PRINT_RESULT(true);			// ������� ǥ�� on&off
const bool IMAGE_DEBUG(false);
double STRAIGHT_LEVEL(1.00);	// ���� ���������� ����ġ Ŀ���� ���� ���ƴ
int straightSteer = 7;		// 44~56
int straightLower = 50 - straightSteer;
int straightUpper = 50 + straightSteer;

int threshold_1 = 118;		//215 //340
int threshold_2 = 242;		//330 //500
int HLP_threshold = 80;	//105
int HLP_minLineLength = 120;//115
int HLP_maxLineGap = 500;	//260

int h1 = 14;
int h2 = 46;
int s = 0;		//40
int v = 220;	//90

Scalar lower_yellow(h1, s, v);
Scalar upper_yellow(h2, 255, 255);

Scalar color[7]{
	Scalar(255,0,0),
	Scalar(0, 0, 255),
	Scalar(255,255,0),
	Scalar(0,255,0),
	Scalar(255,0,255),
	Scalar(0,255,255),
	Scalar(192,192,192)
};
Scalar pink(255, 50, 255);
Scalar mint(255, 153, 0);

static void on_trackbar(int, void*)
{
}

void Driving_DH::basicSetting()
{
	printResult = PRINT_RESULT;
	speed = 40;
	steering = 50;
	for (int i = 0; i < 6; i++)
	{
		section[i] = 0;
		value[i] = 0;
	}
	frame_ROI_Line = Mat(Size(frame_size.width, frame_size.height), CV_8UC3, Scalar(0));
	rectangle(frame_ROI_Line		//���� ������ 60%�ڸ��� �ϴ� 0%����
		, Rect(0
			, cvRound(frame_size.height * (60.0 / 100.0))//, cvRound(frame_size.height * (65.0 / 100.0))
			, frame_size.width
			, cvRound(frame_size.height * (40.0 / 100.0)))//, cvRound(frame_size.height * (35.0 / 100.0)))
		, Scalar(255, 255, 255)
		, FILLED);

	RoiCenter = Point(
		cvRound(frame_ROI_Line.size().width * (50.0 / 100.0)),
		cvRound(frame_ROI_Line.size().height * (80.0 / 100.0))
	);
	//RoiCenter�� ������ ��������� �Ǵ� ������ �ȴ�.
}

Driving_DH::Driving_DH()
{
	print = PRINT;
	straightLevel = STRAIGHT_LEVEL;	// ���� ���������� ����ġ

	frame_size = Size(
		640,
		480
	);
	//���� ������� �ҷ�����
	basicSetting();
}
Driving_DH::Driving_DH(bool printFlag, double sLevel)
{
	*(this) = Driving_DH();
	print = printFlag;
	straightLevel = sLevel;	// ���� ���������� ����ġ
}
Driving_DH::Driving_DH(string& filename)
{
	print = PRINT;
	straightLevel = STRAIGHT_LEVEL;	// ���� ���������� ����ġ
	if (filename.find(".mp4") != -1 || filename.find(".avi") != -1)	//�������ϰ��
	{
		VideoCapture videocap(filename);
		if (!videocap.isOpened())
		{
			cerr << "video load fail !" << endl;
		}

		frame_size = Size(
			cvRound(videocap.get(CAP_PROP_FRAME_WIDTH)),
			cvRound(videocap.get(CAP_PROP_FRAME_HEIGHT))
		);
	}
	else	//�̹����� ���
	{
		Mat temp;
		temp = imread(filename, IMREAD_COLOR);
		frame_size = Size(
			temp.size().width,
			temp.size().height
		);
	}
	//�̹��� ������ �ҷ�����
	basicSetting();
}
Driving_DH::Driving_DH(const char* filename)
{
	string name(filename);
	*(this) = Driving_DH(name);
}

void Driving_DH::driving(Mat& frame, double& steerVal, double& speedVal, double basicSpd, double level)
{
	speedVal = basicSpd;			//�⺻ �ӵ�

	imgProcess(frame, steerVal);	//steerVal���� ���Ѵ�.
}

void Driving_DH::imgProcess(Mat& frame, double& steerVal)
{
	//frame_ROI;
	//frame_hsv;
	//yellowThreshold;
	frame_yellow = Mat();
	//frame_edge;

	frame_ROI = frame & frame_ROI_Line;	//���� ROI�� ����Ѵ�.
	cvtColor(frame_ROI, frame_hsv, COLOR_BGR2HSV);	//����� ���� ���� HSV��ȯ
	inRange(frame_hsv, Scalar(h1, s, v), Scalar(h2, 255, 255), yellowThreshold);	//����� �����Ͽ� 1ä�� Mat��ü yellowThreshold����
	bitwise_and(frame_ROI, frame_ROI, frame_yellow, yellowThreshold);	//yellowThreshold��ü�� ���� frame ���͸�.
	Canny(frame_yellow, frame_edge, threshold_1, threshold_2);	//������� ���� frame�� ������ 1ä�� Mat��ü�� ����

	if (IMAGE_DEBUG)
	{
		createTrackbar("h1", "trackbar", &h1, 30, on_trackbar);
		createTrackbar("h2", "trackbar", &h2, 60, on_trackbar);
		createTrackbar("s", "trackbar", &s, 255, on_trackbar);
		createTrackbar("v", "trackbar", &v, 255, on_trackbar);
		createTrackbar("threshold1", "trackbar", &threshold_1, 500, on_trackbar);
		createTrackbar("threshold2", "trackbar", &threshold_2, 500, on_trackbar);
		createTrackbar("H_thresh", "trackbar", &HLP_threshold, 500, on_trackbar);
		createTrackbar("H_minLen", "trackbar", &HLP_minLineLength, 500, on_trackbar);
		createTrackbar("H_maxGap", "trackbar", &HLP_maxLineGap, 500, on_trackbar);
		namedWindow("trackbar", WINDOW_NORMAL);

		namedWindow("frame_yellow", WINDOW_NORMAL);
		imshow("frame_yellow", frame_yellow);
		resizeWindow("frame_yellow", 320, 240);
		moveWindow("frame_yellow", 0, 40);

		namedWindow("frame_edge", WINDOW_NORMAL);
		imshow("frame_edge", frame_edge);
		resizeWindow("frame_edge", 320, 240);
		moveWindow("frame_edge", 320, 40);
	}

	vector<Vec4i> lines;		//����� ������ ����� ��ü
	HoughLinesP(frame_edge, lines, 1, CV_PI / 180, HLP_threshold, HLP_minLineLength, HLP_maxLineGap);

	lowest = Point(0, 0);					//������ ���ϴ� �� ��ǥ(���ϴ� = y��ǥ�ִ밪)
	lowestLine = Vec4i(0, 0, 0, 0);		//���ϴ� ����
	rightLine = Vec4i(0, 0, 0, 0);		//�ֿ��� ����
	leftLine = Vec4i(640, 0, 640, 0);		//������ ����

	for (unsigned int i = 0; i < lines.size(); i++)
	{
		//int height = lines[i][1] - lines[i][3];
		double slope = ((double)lines[i][3] - lines[i][1]) / ((double)lines[i][2] - lines[i][0]);
		//cout << "slope = " << slope << endl;
		if (slope > -0.125 && slope < 0.125)
		{	//�������� ���� ����
			if (print) line(frame, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(255, 255, 255), 2);
			lines.erase(lines.begin() + i);
			i--;
		}
		else
		{
			if (lowest.y < lines[i][1])
			{
				lowest = Point(lines[i][0], lines[i][1]);
				lowestLine = lines[i];
			}
			if (lowest.y < lines[i][3])
			{
				lowest = Point(lines[i][2], lines[i][3]);
				lowestLine = lines[i];
			}
			if (leftLine[0] > lines[i][0])
			{
				leftLine = lines[i];
				//leftSlope = slope;
			}
			if (rightLine[2] < lines[i][2])
			{
				rightLine = lines[i];
				//rightSlope = slope;
			}
		}
	}
	//for���� ������ ���� ���� Ư¡���� ����.
	//cout << "leftSlope = " << leftSlope << ", rightSlope = " << rightSlope << '\n' << '\n';

	if (lowest.x == 0)	//���� ���� ���
	{
		//��� ���� �ൿ�� �����Ѵ�.
		if (printResult) putText(frame, "none", RoiCenter, FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255), 2);
	}
	else if (lowest.x < RoiCenter.x)		//lowest�� ������ ���
	{

		if (rightLine[1] > rightLine[3])	//rightLine�� ������� ��� = ����1��( / ) - ��ȸ��
		{
			resultLine = Vec4i(lowest.x, lowest.y, rightLine[2], rightLine[3]);
			lineExtend(resultLine, 1);
			//�Ǵ��� ���� 1���� �����Ų��.
			steerVal = 50 + mapping(resultLine[3]);
			//���ఢ�� ��ȯ�Ѵ�

			if (printResult)
			{
				line(frame, Point(resultLine[0], resultLine[1]), Point(resultLine[2], resultLine[3]), pink, 5);
				putText(frame, "RIGHT(" + to_string((int)steerVal) + "%)", RoiCenter, FONT_HERSHEY_COMPLEX, 1, mint, 2);
			}

		}
		else	//���� 2�� - ����
		{
			//lineExtend(leftLine, 2);
			//lineExtend(rightLine, 2);

			int slope = (rightLine[3] - leftLine[1]) / 3; // ���� �ϴ��� ������ ��� -> ������ ������ �� ������
			steerVal = 50.0 - slope;
			//���ఢ�� ��ȯ�Ѵ�
			if (steerVal < straightLower) steerVal = straightLower;
			else if (steerVal > straightUpper) steerVal = straightUpper;

			if (printResult)
			{
				line(frame, Point(leftLine[0], leftLine[1]), Point(leftLine[2], leftLine[3]), pink, 5);
				line(frame, Point(rightLine[0], rightLine[1]), Point(rightLine[2], rightLine[3]), pink, 5);
				putText(frame, "straight(" + to_string((int)steerVal) + "%)", RoiCenter, FONT_HERSHEY_COMPLEX, 1, mint, 2);
			}
		}
	}
	else	//lowest�� ������ ��� 
	{

		if (leftLine[1] < leftLine[3])	//leftLine�� �������� ��� = ����1��( �� ) - ��ȸ��
		{
			resultLine = Vec4i(leftLine[0], leftLine[1], lowest.x, lowest.y);
			lineExtend(resultLine, 1);
			//�Ǵ��� ���� 1���� �����Ų��.
			steerVal = 50 - mapping(resultLine[1]);
			//���ఢ�� ��ȯ�Ѵ�

			if (printResult)
			{
				line(frame, Point(resultLine[0], resultLine[1]), Point(resultLine[2], resultLine[3]), pink, 5);
				putText(frame, "LEFT(" + to_string((int)steerVal) + "%)", RoiCenter, FONT_HERSHEY_COMPLEX, 1, mint, 2);
			}
		}
		else	//���� 2�� - ����
		{
			//lineExtend(leftLine, 2);
			//lineExtend(rightLine, 2);

			int slope = (rightLine[3] - leftLine[1]) / 3;
			steerVal = 50.0 - slope;
			//���ఢ�� ��ȯ�Ѵ�
			if (steerVal < straightLower) steerVal = straightLower;
			else if (steerVal > straightUpper) steerVal = straightUpper;

			if (printResult)
			{
				line(frame, Point(leftLine[0], leftLine[1]), Point(leftLine[2], leftLine[3]), pink, 5);
				line(frame, Point(rightLine[0], rightLine[1]), Point(rightLine[2], rightLine[3]), pink, 5);
				putText(frame, "straight(" + to_string((int)steerVal) + "%)", RoiCenter, FONT_HERSHEY_COMPLEX, 1, mint, 2);
			}
		}
	}
	if (print)
	{
		for (int j = 0; j < lines.size(); j++)
		{
			string text("line ");
			text.append(to_string(j + 1));
			text.append(" : (");
			text.append(to_string(lines[j].operator[](0)));
			text.append(", ");
			text.append(to_string(lines[j].operator[](1)));
			text.append("), (");
			text.append(to_string(lines[j].operator[](2)));
			text.append(", ");
			text.append(to_string(lines[j].operator[](3)));
			text.append(")");
			putText(frame, text, Point(30, 30 * (j + 2)), FONT_HERSHEY_COMPLEX, 1, color[j], 2);
			line(frame, Point(lines[j][0], lines[j][1]), Point(lines[j][2], lines[j][3]), color[j], 2);
			circle(frame, Point2i(lines[j].operator[](0), lines[j].operator[](1)), 5, color[j], -1, LINE_AA);
			circle(frame, Point2i(lines[j].operator[](2), lines[j].operator[](3)), 5, color[j], -1, LINE_AA);
		}
		//������ ��ǥ ȭ�鿡 ����غ���.

		//line(frame, Point(RoiCenter.x * 1.3, 0), Point(RoiCenter.x * 1.3, frame_size.height), Scalar(0), 2);
		//line(frame, Point(RoiCenter.x * 0.7, 0), Point(RoiCenter.x * 0.7, frame_size.height), Scalar(0), 2);
	}
}

void Driving_DH::mappingSetSection(double section0_, double section1_, double section2_, double section3_, double section4_, double section5_)
{
	section[0] = section0_;
	section[1] = section1_;
	section[2] = section2_;
	section[3] = section3_;
	section[4] = section4_;
	section[5] = section5_;
}

void Driving_DH::mappingSetValue(double value0_, double value1_, double value2_, double value3_, double value4_, double value5_)
{
	value[0] = value0_;
	value[1] = value1_;
	value[2] = value2_;
	value[3] = value3_;
	value[4] = value4_;
	value[5] = value5_;
}

double Driving_DH::mapping(int linePoint_)
{
	double linePoint = linePoint_;
	double pointRatio = (linePoint / frame_size.height);	// 0 ~ 1.0

	if (pointRatio < section[0])
	{
		return value[0];
	}
	else if (pointRatio < section[1])
	{
		double sectionLen = section[1] - section[0];
		double valueLen = value[1] - value[0];
		double pointPosition = pointRatio - section[0];
		return value[0] + (pointPosition / sectionLen) * valueLen;
	}
	else if (pointRatio < section[2])
	{
		double sectionLen = section[2] - section[1];
		double valueLen = value[2] - value[1];
		double pointPosition = pointRatio - section[1];
		return value[1] + (pointPosition / sectionLen) * valueLen;
	}
	else if (pointRatio < section[3])
	{
		double sectionLen = section[3] - section[2];
		double valueLen = value[3] - value[2];
		double pointPosition = pointRatio - section[2];
		return value[2] + (pointPosition / sectionLen) * valueLen;
	}
	else if (pointRatio < section[4])
	{
		double sectionLen = section[4] - section[3];
		double valueLen = value[4] - value[3];
		double pointPosition = pointRatio - section[3];
		return value[3] + (pointPosition / sectionLen) * valueLen;
	}
	else if (pointRatio < section[5])
	{
		double sectionLen = section[5] - section[4];
		double valueLen = value[5] - value[4];
		double pointPosition = pointRatio - section[4];
		return value[4] + (pointPosition / sectionLen) * valueLen;
	}
	else //50 (1.0)
	{
		return value[5];
	}
}

void Driving_DH::lineExtend(Vec4i& line, int mode)
{
	double a, b;	//����=a, y����=b
	a = ((double)line[3] - line[1]) / ((double)line[2] - line[0]);
	b = line[1] - a * line[0];
	if (mode == 1)
	{
		double y1, y2;	//����y����, ����y����
		y1 = b;	//x = 0
		y2 = (a * frame_size.width + b);	//x = max
		line = Vec4i(0, cvRound(y1), frame_size.width, cvRound(y2));
	}
	else if (mode == 2)
	{
		double x1, x2;
		x1 = (frame_size.height - b) / a; // y = max
		x2 = (-b / a);	//y = 0
		if (x1 < x2) line = Vec4i(cvRound(x1), frame_size.height, cvRound(x2), 0);
		else line = Vec4i(cvRound(x2), 0, cvRound(x1), frame_size.height);
	}
}
