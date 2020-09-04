#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<iostream>
#include<vector>
#include<math.h>
using namespace cv;
using namespace std;
int main(){
	Mat image;
	image = imread("1.jpg",0);
	vector<Mat> templatePyramid;
	vector<Mat> testPyramid;
	Mat tempMat = image;
	templatePyramid.push_back(image);
	for(;;){
		vector<Point2f> points;
		for(int i=0;i<tempMat.rows-1;i+=2){
			Point2f point;
			for(int j=0;j<tempMat.cols-1;j+=2){
				float elem1=(float)tempMat.at<uchar>(i,j);
				float elem2=(float)tempMat.at<uchar>(i,j+1);
				float elem3=(float)tempMat.at<uchar>(i+1,j);
				float elem4=(float)tempMat.at<uchar>(i+1,j+1);
				float elem= (elem1+elem2+elem3+elem4)/4;
				
			}
		}
	}
	return 0;
}
