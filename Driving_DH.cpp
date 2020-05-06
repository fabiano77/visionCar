#include "Driving_DH.h"

bool PRINT(true);				// ���� ��� ǥ�� on&off
double STRAIGHT_LEVEL(1.00);	// ���� ���������� ����ġ Ŀ���� ���� ���ƴ
int straightSteer = 15;		// 35~65
int straightLower = 50 - straightSteer;
int straightUpper = 50 + straightSteer;

int threshold_1 = 118;		//215 //340
int threshold_2 = 242;		//330 //500
int HLP_threshold = 100;	//105
int HLP_minLineLength = 120;//115
int HLP_maxLineGap = 500;	//260

Scalar lower_yellow(14, 30, 35);
Scalar upper_yellow(46, 255, 255);
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

void Driving_DH::basicSetting()
{
	printResult = true;
	speed = 40;
	steering = 50;
	frame_ROI_Line = Mat(Size(frame_size.width, frame_size.height), CV_8UC1, Scalar(0));
	rectangle(frame_ROI_Line
		, Rect(cvRound(frame_size.width * (1.0 / 100.0))
			, cvRound(frame_size.height * (65.0 / 100.0))
			, cvRound(frame_size.width * (98.0 / 100.0))
			, cvRound(frame_size.height * (33.0 / 100.0)))
		, Scalar(255, 255, 255)
		, FILLED);
	// frame_ROI_line ����� ����� �ϴ� 1/3�� ä���.

	RoiCenter = Point(
		cvRound(frame_ROI_Line.size().width * (50.0 / 100.0)),
		//cvRound(frame_ROI_Line.size().height * ((81.5) / 100.0))
		cvRound(frame_ROI_Line.size().height * (79.0 / 100.0))
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

	if (level != 0.0)
	{
		double threshold_s(3.0);		//47~53			����(����) ���� �Ӱ谪
		double threshold_c(20.0);		//0~30, 70~100	�ڳ�(����) ���� �Ӱ谪
		double upperLimit(50.0);		//�ְ�ӵ� ����
		double lowerLimit(20.0);		//�����ӵ� ����
		if (steerVal < 50.0 - threshold_c || steerVal > 50.0 + threshold_c)	//�ڳʱ���
		{
			if (steering < 50.0 - threshold_c || steering > 50.0 + threshold_c)	//���� frame�� �ڳʶ��
			{
				speedVal = speed - level;			//���� 38, 36, 34
				if (speedVal < lowerLimit) speedVal = lowerLimit;	//�����ӵ� ����
			}
		}
		else if (steerVal > 50.0 - threshold_s && steerVal < 50.0 + threshold_s)	//��������
		{
			if (steering > 50.0 - threshold_s && steering < 50.0 + threshold_s)	//���� frame�� �����̶��
			{
				speedVal = speed + level;			//���� 42, 44, 46
				if (speedVal > upperLimit) speedVal = upperLimit;	//�ְ�ӵ� ����
			}
		}
	}

	steering = steerVal;	//class ��������� ����.
	speed = speedVal;		//class ��������� ����.
}

void Driving_DH::imgProcess(Mat& frame, double& steerVal)
{
	Mat frame_edge;
	Mat frame_hsv;
	Mat frame_yellow;
	Mat yellowThreshold;
	cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
	inRange(frame_hsv, lower_yellow, upper_yellow, yellowThreshold);
	bitwise_and(frame, frame, frame_yellow, yellowThreshold);
	//frame�� ������� ����� frame->frame_yellow

	//createTrackbar("threshold1", "trackbar", &threshold_1, 500, on_trackbar);
	//createTrackbar("threshold2", "trackbar", &threshold_2, 500, on_trackbar);
	Canny(frame_yellow, frame_edge, threshold_1, threshold_2);
	frame_edge = frame_edge & frame_ROI_Line;
	//canny�Լ��� ������ �����ϰ� �ϴ� 1/3�� �ڸ���.

	//createTrackbar("H_thresh", "trackbar", &HLP_threshold, 500, on_trackbar);
	//createTrackbar("H_minLen", "trackbar", &HLP_minLineLength, 500, on_trackbar);
	//createTrackbar("H_maxGap", "trackbar", &HLP_maxLineGap, 500, on_trackbar);
	//namedWindow("trackbar", WINDOW_NORMAL);
	vector<Vec4i> lines;		//����� ������ ����� vector(�ټ� ������) Vec4i(x1, y1, x2, y2)
	HoughLinesP(frame_edge, lines, 1, CV_PI / 180, HLP_threshold, HLP_minLineLength, HLP_maxLineGap);
	//������ �����ϰ� lines�� ����.

	Point lowest(0, 0);				//������ ���ϴ� �� ��ǥ(���ϴ� = y��ǥ�ִ밪)
	Point rightSide(0, 0);			//������ �ֿ��� �� ��ǥ(x��ǥ �ִ밪)
	Point leftSide(frame_size.width, 0);		//������ ������ �� ��ǥ(x��ǥ �ּҰ�)

	Vec4i rightLine(0, 0, 0, 0);				//�ֿ��� ����
	Vec4i leftLine(frame_size.width, 0, frame_size.width, 0);	//������ ����

	for (Vec4i l : lines)
	{
		int height = l[1] - l[3];	//������ ���Ʒ� ��������
		if (height < 20 && height > -20)
		{
			l[0] = 0;
		}
		else
		{
			if (lowest.y < l[1]) lowest = Point(l[0], l[1]);
			if (lowest.y < l[3]) lowest = Point(l[2], l[3]);
		}
	}
	//for���� ������ ���� lowest�� ���ϴ� ��ǥ ����.

	if (lowest.x == 0)	//���� ���� ���
	{
		//��� ���� �ൿ�� �����Ѵ�.
		if (printResult) putText(frame, "none", RoiCenter, FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255), 2);
	}
	else if (lowest.x < RoiCenter.x)		//lowest�� ������ ���
	{
		for (Vec4i l : lines) 				//rightSide�� �ֿ��� ��ǥ ����(x��ǥ �ִ밪)
		{
			if (l[0] == 0)
			{

			}
			else
			{
				if (rightSide.x < l[0]) rightSide = Point(l[0], l[1]);
				if (rightSide.x < l[2]) rightSide = Point(l[2], l[3]);
			}
		}

		if (rightSide.y < RoiCenter.y)	//rightSide�� ����� ��� = ����1��( / ) - ��ȸ��
		{
			Vec4i resultLine(lowest.x, lowest.y, rightSide.x, rightSide.y);
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
			for (Vec4i l : lines)
			{
				if (l[0] == 0)
				{

				}
				else
				{
					if (leftLine[0] > l[0]) leftLine = l;
					if (rightLine[2] < l[2]) rightLine = l;
				}
			}
			lineExtend(leftLine, 2);
			lineExtend(rightLine, 2);

			Point lineCenter((leftLine[2] + rightLine[0]) / 2, 20);

			double slope = ((double)lineCenter.x - frame_size.width * 0.5) / (frame_size.width * 0.5);
			steerVal = 50 + ((slope * 50) * straightLevel);
			//���ఢ�� ��ȯ�Ѵ�
			if (steerVal < straightLower) steerVal = straightLower;
			else if (steerVal > straightUpper) steerVal = straightUpper;

			if (printResult)
			{
				circle(frame, lineCenter, 5, pink, 2, LINE_AA);
				line(frame, Point(leftLine[0], leftLine[1]), Point(leftLine[2], leftLine[3]), pink, 5);
				line(frame, Point(rightLine[0], rightLine[1]), Point(rightLine[2], rightLine[3]), pink, 5);
				putText(frame, "straight(" + to_string((int)steerVal) + "%)", RoiCenter, FONT_HERSHEY_COMPLEX, 1, mint, 2);
			}
		}
	}
	else	//lowest�� ������ ���
	{
		for (Vec4i l : lines)	//rightSide�� ������ ��ǥ ����(x��ǥ �ּҰ�)
		{
			if (l[0] == 0)
			{

			}
			else
			{
				if (leftSide.x > l[0]) leftSide = Point(l[0], l[1]);
				if (leftSide.x > l[2]) leftSide = Point(l[2], l[3]);
			}
		}

		if (leftSide.y < RoiCenter.y)	//leftSide�� ����� ��� = ����1��(��) - ��ȸ��
		{
			Vec4i resultLine(leftSide.x, leftSide.y, lowest.x, lowest.y);
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
			for (Vec4i l : lines)
			{
				if (l[0] == 0)
				{

				}
				else
				{
					if (leftLine[0] > l[0]) leftLine = l;
					if (rightLine[2] < l[2]) rightLine = l;
				}
			}
			lineExtend(leftLine, 2);
			lineExtend(rightLine, 2);

			Point lineCenter((leftLine[2] + rightLine[0]) / 2, 20);

			double slope = ((double)lineCenter.x - frame_size.width * 0.5) / (frame_size.width * 0.5);
			steerVal = 50 + ((slope * 50) * straightLevel);
			//���ఢ�� ��ȯ�Ѵ�
			if (steerVal < straightLower) steerVal = straightLower;
			else if (steerVal > straightUpper) steerVal = straightUpper;

			if (printResult)
			{
				circle(frame, lineCenter, 5, pink, 2, LINE_AA);
				line(frame, Point(leftLine[0], leftLine[1]), Point(leftLine[2], leftLine[3]), pink, 5);
				line(frame, Point(rightLine[0], rightLine[1]), Point(rightLine[2], rightLine[3]), pink, 5);
				putText(frame, "straight(" + to_string((int)steerVal) + "%)", RoiCenter, FONT_HERSHEY_COMPLEX, 1, mint, 2);
			}
		}
	}
	if (print)
	{
		string text_1("line count : ");
		text_1.append(to_string(lines.size()));
		putText(frame, text_1, Point(100, 30), FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 0), 2);
		//�� ���� ���

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

