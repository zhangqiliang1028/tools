#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<iostream>
#include<vector>
#include<math.h>
using namespace cv;
using namespace std;
/*Mat image;            输入图像
Mat ROI;                ROI区域
int distanceThreshold;  距离阈值
int denoisingParam;     去噪阈值       
int edgeThreshold;      边缘阈值
int minLength;          最小边缘长度
int maxLength;          最大边缘长度
int minAngle;           最小角度
int maxAngle;           最大角度
*/
Mat image= imread("4.jpg");
typedef struct scanArea{     //ROI区域表示
	Point p1;
	Point p2;
	Point p3;
	Point p4;

}scanArea;

void pointsDetection(Mat src,scanArea area, int edgeThreshold,float denoisingParam,vector<vector<Point2f>> &points )           //边缘点检测
{	
	Point2f p11,p12,p21,p22;
	p11=area.p1;
	p12=area.p2;
	p21=area.p3;
	p22=area.p4;
	blurLength=2;
	int* blurKernel = (int*)malloc((blurLength * 2 + 1) * sizeof(int)); //申请滤波核的存储器
	int index=0;
	int blurLength=2;
	for ( index=0;index < blurLength;index++)
	{
		blurKernel[index] = -1;
	}
	blurKernel[index] = 0;
	index++;
	for (; index < blurLength * 2 + 1; index++)
	{
		blurKernel[index] = 1;
	}
	Mat blurImage,cannyImage;
	int denoisy=denoisingParam/2*2+1;
	GaussianBlur(src,blurImage,Size(denoisy,denoisy),0,0);
	
	vector<vector<Point2f>> results;            //将Mat转为二维向量
	for(int i=0;i<src.rows;i++){
		vector<float> v;
		for(int j=0;j<src.cols;j++){
			float ff=(float)src.at<uchar>(i,j); 
			v.push_back(ff);
		}
		results.push_back(v);
	}

	vector<Point2f> acc;                          //左到右水平扫描
	for(int i=0;i<results.size();i++){
		for(int j=blurLength;j<results[i].size()-blurlength;j++){
			int index=0;
			float value=0;
			for(int k=j-blurLength;k<=j+blurLength;k++){
				value+=results[i][k]*blurKernel[index];
			}
			value/=blurLength;
			if(fabs(value)>edgeThreshold){
				acc.push_back(Point2f(i,j));
				break;
			}
		}

	}
	points.push_back(acc);

	vector<Point2f> acc;                         //右到左水平扫描
	for(int i=0;i<results.size();i++){
		for(int j=results[i].size()-blurlength-1;j>=blurLength;j--){
			int index=0;
			float value=0;
			for(int k=j-blurLength;k<=j+blurLength;k++){
				value+=results[i][k]*blurKernel[index];
			}
			value/=blurLength;
			if(fabs(value)>edgeThreshold){
				acc.push_back(Point2f(i,j));
				break;
			}
		}

	}
	points.push_back(acc);

	vector<vector<Point2f>> results;            //将Mat转为二维向量
	for(int i=0;i<src.cols;i++){
		vector<float> v;
		for(int j=0;j<src.rows;j++){
			float ff=(float)src.at<uchar>(i,j); 
			v.push_back(ff);
		}
		results.push_back(v);
	}
	vector<Point2f> acc;                         //上到下水平扫描
	for(int i=0;i<results.size();i++){
		for(int j=blurLength;j<results[i].size()-blurlength;j++){
			int index=0;
			float value=0;
			for(int k=j-blurLength;k<=j+blurLength;k++){
				value+=results[i][k]*blurKernel[index];
			}
			value/=blurLength;
			if(fabs(value)>edgeThreshold){
				acc.push_back(Point2f(i,j));
				break;
			}
		}

	}
	points.push_back(acc);

	vector<Point2f> acc;                         //下到上水平扫描
	for(int i=0;i<results.size();i++){
		for(int j=results[i].size()-blurlength-1;j>=blurLength;j--){
			int index=0;
			float value=0;
			for(int k=j-blurLength;k<=j+blurLength;k++){
				value+=results[i][k]*blurKernel[index];
			}
			value/=blurLength;
			if(fabs(value)>edgeThreshold){
				acc.push_back(Point2f(i,j));
				break;
			}
		}

	}
	points.push_back(acc);

	//HoughLinesP(cannyImage,lines,1,CV_PI/180,30,30,10);
	

}
int edgeDetect(Mat src,scanArea area, int distanceThreshold,float denoisingParam,int edgeThreshold,int minLength,int maxLength,int minAngle,int maxAngle,Edge &edge){

	
	//将原图转化为灰度图
	if(src.channels()>1){
		cvtColor(src,src, COLOR_BGR2GRAY); 
	}
	vector<Vec4i> lines;
	vector<vector<Point2f>> points;
	pointsDetection(src,area,edgeThreshold,denoisingParam,points );
	cout<<"+++1 :"<<lines.size()<<endl;
	vector<Vec4f> lines;
	Vec4f line_;
	for(int i=0;i<4;i++){
		if(points[i].size()>3){
			fitLine(points[i], line_, DIST_L2, 0, 0.01,0.01);
			float x0,y0,k,b;
			x0 = line_[2];
			y0 = line_[3];
			k = line_[1] / line_[0];
			b=y0-k*x0;
			Point2f begin,end;
			begin.x=points[0].x;
			begin.y=k*begin.x+b;
			end.x=points[points.size()-1].x;
			end.y=k*end.x+b;
			lines.push_back(line_);
		}
		
	}
	
	return 1;
}

int main()
{
	
	Mat src;
	scanArea area;
	image.copyTo(src);
	area={Point(9,9),Point(image.cols-9,9),Point(9,image.rows-9),Point(image.cols-9,image.rows-9)};//初始化ROI的四个端点
	//edgeDetect(area,distanceThreshold,denoisingParam,edgeThreshold,minLength,maxLength,minAngle,maxAngle);
	Edge edge;
	edgeDetect(src,area,2, 2.0,20,20,50,0,360,edge);  //边检测和绘制(段错误)
	
	imshow("image", image);
	waitKey(0);
	return 0;
}
