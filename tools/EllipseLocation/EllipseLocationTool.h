#ifndef IVS_LINEDETECTTOOL_H
#define IVS_LINEDETECTTOOL_H
#include "ToolBase.h"
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;
/*
【颜色变化】设置椭圆的颜色；
	【白色椭圆】椭圆内侧相对外侧是白色；
	【黑色椭圆】椭圆内侧相对外侧是黑色；
	【两者皆可】以上两者均可椭圆是基于边缘轮廓分析得到的，以下两个参数设置边缘检出参数
【边缘尺度】边缘尺寸设置边缘检出的尺度。当图像噪声大或者边缘模糊的时候，需要增大边缘尺度。
【边缘阈值】边缘检出阈值
【半径范围】设置椭圆半长轴和半短轴的范围，用来缩小椭圆搜索的范围；
【圆度范围】圆度定义为椭圆短轴长度与长轴长度之比，用来缩小椭圆搜索的范围。找到椭圆后需要对结果进行过滤，
【距离阈值】当边缘点和椭圆的距离小于此参数时，边缘点在椭圆上。
【检出阈值】椭圆上的边缘点覆盖椭圆一圈的比例就是椭圆的得分100分制。大于此阈值的才被检出。可以用来过滤椭圆弧。
【挑选类型】用来挑选得到唯一的一个椭圆来定位。

Mat  src                ：输入图像
scanArea scanArea       :扫描区域 
int colorChange         ：颜色变换，0为两者皆可，1为白到黑，-1为黑到白
int edgeDimension       : 边缘尺度
int edgeThreshold       ：边缘阈值
int maxRadius           :最大半径
int minRadius           :最小半径
int maxRoundness        :最大圆度
int minRoundness        :最小圆度
int distanceThreshold   :距离阈值
int score               :找圆得分(检出阈值)
int selectType          :挑选类型，0为最小椭圆，1为最大椭圆
*/
typedef struct scanArea{
	Point2f p1;
	Point2f p2;
	Point2f p3;
	Point2f p4;
}scanArea;

typedef struct Ellipse{
	Point2f center;
	float radius1;
	float radius2;
	float angle;
	
}Ellipse;

class EllipseLocationTool :public ToolBase
{
private:
	std::vector<int> relyNums={};
	scanArea detectArea ;      //扫描区域 
	Ellipse ellipse;           //检测结果
	Mat image;                //存储输入图片
	int colorChange;         //颜色变换，0为两者皆可，1为白到黑，-1为黑到白
	int edgeDimension;       // 边缘尺度
	int edgeThreshold;       //边缘阈值
	int maxRadius;           //最大半径
	int minRadius;           //最小半径
	int maxRoundness;        //最大圆度
	int minRoundness;        //最小圆度
	int distanceThreshold;   //距离阈值
	int score;               //找圆得分(检出阈值)
	int selectType;          //挑选类型，0为最小圆，1为最大圆
	float center_angle;     //模板旋转角度
	scanArea templateArea;    //模板区域
	Point2f center;       //矩形旋转中心坐标：模板特征定位区域中心点
	Point2f tempDatum_point;//矩形计算相对位移基准点坐标：模板特征定位区域中心点
	Point2f testDatum_point;//矩形计算相对位移基准点坐标：待测图定位的中心点
	bool isInside(RotatedRect box,Point2f point);
	bool isInRect(scanArea area,Point2f p);
	int ellipseDetect(Mat src,scanArea area, int colorChange,int edgeDimension,int edgeThreshold,int maxRadius,int minRadius,int maxRoundness,int minRoundness,int distanceThreshold,int score,int selectType,Ellipse ellipse);
	int move_templateArea(Point2f center, float center_angle, Point2f tempDatum_point,Point2f testDatum_point,scanArea &detectArea);
	void workPoints(Mat src,scanArea area,Mat &mat);

public:

	EllipseLocationTool(){};
	virtual~EllipseLocationTool() {};
        virtual int getRelyOnParam(std::vector<int> &nums);
        virtual int initTool(Json::Value json);
        virtual void updateTool(Json::Value json){};
	virtual int run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result);
        virtual int run(cv::Mat src,cv::Mat &dst ){};
        virtual int emptyTool(){};

};
#endif 
