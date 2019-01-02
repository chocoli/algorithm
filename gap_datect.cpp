#include <opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

//点序列扫描
void scan(Mat polar, vector<Point>& points)
{
	int r[360];
	for (int j = 0; j < 360; j++)
	{
		bool find = false;
		for (int i = 0; i < polar.rows; i++)
		{
			if (polar.at<uchar>(i, j))
			{
				r[j] = i;
				find = true;
				break;
			}
		}
		if (!find)
		{
			r[j] = 0;
		}
	}
	int disconnect = 30, connect = 10;
	for (int i = 0; i < 360; i++)
	{
		int r1 = r[i];
		int r2 = r[(i + 1) % 360];
		if (r1 != 0 && r2 != 0 && abs(r1 - r2)>disconnect)
		{
			int start = i + 1;
			int end = start;
			while (abs(r[(end + 1) % 360] - r[end]) <= connect)
			{
				end++;
			}
			if (end < start) end += 360;
			for (int j = start; j <= end; j++)
			{
				r[j % 360] = 0;
			}
		}
	}
	for (int i = 0; i < 360; i++)
	{
		if (r[i] != 0 && r[(i + 1) % 360] == 0)
		{
			int start = i, end = start + 1, length = 1;
			while (r[end%360] == 0)
			{
				end++;
				length++;
			}
			for (int j = start+1; j < end; j++)
			{
				r[j % 360] = r[start]+(r[end % 360] - r[start])*(j - start) / length;
			}
		}
	}
	for (int i = 0; i < 360; i++)
	{
		points.push_back(Point(i, r[i]));
	}
}

//轮廓比较函数
bool ContoursSortFun(vector<cv::Point> contour1, vector<cv::Point> contour2)
{
	return contourArea(contour1) > contourArea(contour2);
	return (contour1.size() > contour2.size());
}

//检测函数
bool detect(Mat src, Mat& dst, int min_gap)
{
	bool isNG = false;
	dst = src.clone();
	int height = src.rows, width = src.cols;
	Mat gray; cvtColor(src, gray, CV_BGR2GRAY);
	resize(gray, gray, Size(height, height));
	blur(gray, gray, Size(6, 6));
	int x0 = height / 2, y0 = height / 2;
	Mat polar = Mat::zeros(x0, 360, CV_8UC1);
	for (int r = 0; r < polar.rows; r++)
	{
		for (int theta = 0; theta < polar.cols; theta++)
		{
			double rad = theta*3.1415927 / 180;
			int x = r*cos(rad) + x0;
			int y = r*sin(rad) + y0;
			polar.at<uchar>(r, theta) = gray.at<uchar>(y, x);
		}
	}
	Sobel(polar, polar, CV_8UC1, 0, 2, 7, 2, 0, BORDER_DEFAULT);
	threshold(polar, polar, 128, 255, THRESH_BINARY);
	polar(Rect(0, 0, polar.cols, polar.rows*0.6)).setTo(0);
	vector<vector<Point>> contours; findContours(polar, contours, RETR_TREE, CV_CHAIN_APPROX_NONE); sort(contours.begin(), contours.end(), ContoursSortFun);
	polar.setTo(0);
	for (int i = 0; i < 4; i++)
	{
		if (contours[i].size() < 80) continue;
		drawContours(polar, contours, i, Scalar(255), CV_FILLED);
	}
	vector<Point> points; scan(polar, points);
	vector<bool> isNGPoint;
	for (int i = 0; i < points.size(); i++)
	{
		if (y0 - points[i].y < min_gap)
		{
			isNGPoint.push_back(true); isNG = true;
		}
		else
		{
			isNGPoint.push_back(false);
		}
		double rad = points[i].x*3.1415927 / 180;
		int x = points[i].y*cos(rad) + x0;
		int y = points[i].y*sin(rad) + y0;
		points[i].x = x*width / height;
		points[i].y = y;
	}
	for (int i = 0; i < points.size(); i++)
	{
		if (isNGPoint[i])
			line(dst, points[i], points[(i + 1) % 360], Scalar(0, 0, 255), 1);
		else
			line(dst, points[i], points[(i + 1) % 360], Scalar(0, 255, 0), 1);
	}
	RotatedRect rect(Point2f(width / 2, height / 2), Size(width - 2 * min_gap, height - 2 * min_gap), 0);
	ellipse(dst, rect, Scalar(255, 255, 255), 1);
	return !isNG;
}


int main()
{
	Mat src = imread("C:\\Users\\kekehui\\Desktop\\temp.jpg");
	Mat dst; detect(src, dst, 10);
	cvNamedWindow("mat", 0);
	double t = getTickCount();
	int max = 500;
	for (int i = 0; i < max; i++)
	{
		char file[60]; sprintf_s(file, "C:\\Users\\kekehui\\Desktop\\top\\reright\\%d.jpg", i);
		Mat src = imread(file);
		if (!src.data) continue;
		Mat dst; 
		if (!detect(src, dst, 10))
		{
			sprintf_s(file, "C:\\Users\\kekehui\\Desktop\\top\\ng_of_reright\\%d.jpg", i);
			imwrite(file, dst);
			cout << i << endl;
		}
		imshow("mat", dst); waitKey(0);
	}
	t = (getTickCount() - t) / getTickFrequency();
	cout << "fps:" << max / t << endl;
}

