#ifndef IVS_LINEDETECTTOOL_H
#define IVS_LINEDETECTTOOL_H
#include "ToolBase.h"
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;
/*Mat image;         输入图像
Mat ROI;             ROI区域
int selectPointCount;选点数量
int blurWidth;       滤波宽度
int edgeThreshold;  边缘阈值
int selectionMethod; 选点方式，0为峰值点，1为最先找到点
int colorChange;     颜色变换，0为黑到白，1为白到黑
int blurLength;      差分滤波器长
int edgeDepth;       边缘深度
int minLength;       最小边缘长度
int maxLength;       最大边缘长度
int angleDiffer;     测出边缘与ROI的最大角度差异*/
typedef struct scanArea{             //检测区域
	Point p1;
	Point p2;
	Point p3;
	Point p4;
}scanArea;
typedef struct Node{
	Point point;//位置
	double value;//求平均值后的结果
	double result=0;//滤波后的结果
}node;
typedef struct Line{
	Point p1;
	Point p2;
}Line;  
   
Mat ROI;                          //在ROI区域
//Mat mask;                       //获取ROI使用的掩膜 
scanArea area_;          //ROI
class EdgeDetectTool :public ToolBase
{
private:
	std::vector<int> relyNums={};
	Mat image;
	int selectPointCount;          //选点数量·
	int blurWidth;                    //滤波宽度
	int edgeThreshold;            //边缘阈值
	int selectionMethod;       //选点方式，0为峰值点，1为最先找到点
	int colorChange;             //颜色变换，0为黑到白，1为白到黑
	int blurLength;                 //差分滤波器长度
	int edgeDepth;             //深度阈值
	int minLength;                   //最小长度
	int maxLength;	          //最大长度
	int angleDiffer;       //角度差异
	int step;
	scanArea area;
	scanArea sample_area;
	cv::Rect detectRect;//模板检测矩形，也是待测图检测矩形
	Point2f center;//矩形旋转中心坐标：模板特征定位区域中心点
	float center_angle;//模板旋转角度
	Point2f tempDatum_point;//矩形计算相对位移基准点坐标：模板特征定位区域中心点
	Point2f testDatum_point;//矩形计算相对位移基准点坐标：待测图定位的中心点
	int sample_width;//待测图的宽度高度
        int sample_height;
	Line line;

	int  Rotate_rect(cv::Point2f center, double center_angle, cv::Point2f tempDatum_point, cv::Point2f testDatum_point,scanArea &sample_area);
	
	static bool cmp(Point &m,Point &n){
		
		int d1,d2;
		if(area_.p1.x==area_.p2.x){
			d1=abs(area_.p1.x-m.x);	
				d2=abs(area_.p1.x-n.x);
			}
		else{
			double k=(double)(area_.p2.y-area_.p1.y)/(area_.p2.x-area_.p1.x);
			double b=area_.p1.y-k*area_.p1.x;
			d1=(int)abs(k*m.x+b-m.y)/sqrt(k*k+1);
			d2=(int)abs(k*n.x+b-n.y)/sqrt(k*k+1);
		}
		return d1<d2;		
	
	};
	Point getEndPoint(vector<Point> points,int xy,int be);
	void workPoints(Mat src,scanArea area1, int selectPointCount, int blurWidth,vector<vector<Point>> &results);
	void pointsDetection(Mat src,vector<vector<Point>> scanPoints, int edgeThreshold, int selectionMethod, int colorChange, int blurLength,int blurWidth,int edgeDepth,vector<Point> &points );
	int edgeDetect(Mat src,scanArea area1,int selectPointCount, int blurWidth,int blurLength,int edgeThreshold, int selectionMethod,int colorChange,int edgeDepth,int minLength,int maxLength,int angle,Line &line);
	
public:

	EdgeDetectTool(){};
	virtual~EdgeDetectTool() {};
        virtual int getRelyOnParam(std::vector<int> &nums);
        virtual int initTool(Json::Value json);
        virtual void updateTool(Json::Value json){};
	virtual int run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result);
        virtual int run(cv::Mat src,cv::Mat &dst ){};
        virtual int emptyTool(){};

};
#endif 
