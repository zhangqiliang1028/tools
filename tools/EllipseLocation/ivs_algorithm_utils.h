#ifndef IVS_ALGORITHM_UTILS_H
#define IVS_ALGORITHM_UTILS_H

#include "IVSParameters.h"
#include <cstdlib>
#include <sys/timeb.h>
#include <opencv2/opencv.hpp>

class TimeTracker
{
private:
	timeb timer[2];

public:
	void start()
	{
		ftime(&timer[0]);
		//return this->timer[0];
	}

	void stop()
	{
		ftime(&timer[1]);
		// return this->timer[1];
	}

	int duration() const
	{
		int secs(this->timer[1].time - this->timer[0].time);
		int msecs(this->timer[1].millitm - this->timer[0].millitm);

		if (secs < 0)
		{
			--secs;
			msecs += 1000;
		}

		return static_cast<int>(secs * 1000 + msecs);//����
	}
};
// ȡ���ĸ�������Сֵ
template<class T>
T IVS_MIN(T a, T b, T c, T d) {
	T result = a;
	if (result > b) {
		result = b;
	}
	if (result > c) {
		result = c;
	}
	if (result > d) {
		result = d;
	}
	return result;
}

// ȡ���ĸ��������ֵ
template<class T>
T IVS_MAX(T a, T b, T c, T d) {
	T result = a;
	if (result < b) {
		result = b;
	}
	if (result < c) {
		result = c;
	}
	if (result < d) {
		result = d;
	}
	return result;
}

//void saveMat(cv::Mat mat, const char *path) {
//	FILE *fp = fopen(path, "w");
//	int i, j;
//	for (i = 0; i < mat.rows; ++i) {
//		for (j = 0; j < mat.cols; ++j) {
//			//            fprintf(fp, "%d ", (mat.ptr + i * mat.step)[j]);
//			fprintf(fp, "%d ", mat.at<uchar>(i, j));
//		}
//		fprintf(fp, "\n");
//	}
//	fclose(fp);
//}
//
//void saveMatf(cv::Mat mat, const char *path) {
//	FILE *fp = fopen(path, "w");
//	int i, j;
//	for (i = 0; i < mat.rows; ++i) {
//		for (j = 0; j < mat.cols; ++j) {
//			//            fprintf(fp, "%d ", (mat.ptr + i * mat.step)[j]);
//			fprintf(fp, "%d ", mat.at<short>(i, j));
//		}
//		fprintf(fp, "\n");
//	}
//	fclose(fp);
//}


void bitmap2Mat(cv::Mat &dst, UINT8 bitmap[], UINT16 width, UINT16 height) {
	for (int i = 0; i < height; ++i) {
		for (int j = 0; j < width; ++j) {
			//�ͻ��˴�������1 0
			dst.at<uchar>(i, j) = static_cast<uchar>(bitmap[i * width + j] * 255);
		}
	}
}

//bitmapҪ��ǰ����ÿռ�
void mat2Bitmap(const cv::Mat &src, UINT8 bitmap[], UINT16 width, UINT16 height) {
	for (int i = 0; i < height; ++i) {
		for (int j = 0; j < width; ++j) {
			bitmap[i * width + j] = src.at<uchar>(i, j) == 255 ? 1 : 0;
		}
	}
}

void Dilation(const cv::Mat &src, cv::Mat &dilation_dst, int size)
{
	int dilation_type = cv::MORPH_RECT;

	cv::Mat element = cv::getStructuringElement(dilation_type,
		cv::Size(size, size));
	///���Ͳ���
	dilate(src, dilation_dst, element);
}

void Erode(const cv::Mat &src, cv::Mat &erode_dst, int size)
{
	int dilation_type = cv::MORPH_RECT;

	cv::Mat element = cv::getStructuringElement(dilation_type,
		cv::Size(size, size));
	///��ʴ����
	erode(src, erode_dst, element);
}


#endif
