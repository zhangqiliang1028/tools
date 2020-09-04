#ifndef IVS_LINEDETECTTOOL_H
#define IVS_LINEDETECTTOOL_H
#include "ToolBase.h"
#include <iostream>

//线定义
struct Line{
	cv::Point p1;//起始点与终点
        cv::Point p2;
};

class LineDetectTool :public ToolBase
{
private:
        std::vector<int> relyNums={};
	cv::Point2f center;//矩形旋转中心坐标：模板特征定位区域中心点
	float center_angle;//模板旋转角度
	cv::Point2f tempDatum_point;//矩形计算相对位移基准点坐标：模板特征定位区域中心点
	cv::Point testDatum_point;//矩形计算相对位移基准点坐标：待测图定位的中心点
	cv::Rect detectRect;//模板检测矩形，也是待测图检测矩形
	int sample_width;//待测图的宽度高度
        int sample_height;
	Line line;

	int  Rotate_rect(cv::Point2f center, float center_angle, cv::Point2f tempDatum_point, cv::Point testDatum_point, cv::Rect &detectRect, int t_w, int t_h);
	int LineDetect(cv::Mat input, cv::Rect ROI, Line &l);

public:
	LineDetectTool(){};
	virtual~LineDetectTool() {};
        virtual int getRelyOnParam(std::vector<int> &nums);
        virtual int initTool(Json::Value json);
        virtual void updateTool(Json::Value json){};
	virtual int run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result);
        virtual int run(cv::Mat src,cv::Mat &dst ){};
        virtual int emptyTool(){};

};
#endif 
