#ifndef IVS_DISTANCEMEASURE_H
#define IVS_DISTANCEMEASURE_H
#include "ToolBase.h"
#include <iostream>

//线定义
struct Line{
	cv::Point p1;//起始点与终点
        cv::Point p2;
};

class DistanceMeasureTool :public ToolBase
{
private:
        std::vector<int> relyNums={};
        int type;//0：线-线距离，1：点-点距离，2：点-线距离
	std::vector<Line> lines;
        //std::vector<cv::Point> points;

        int GetLineDistance(Line l1, Line l2,float &distance);
        //float GetLineDistance(cv::Point p, Line l);

public:
	DistanceMeasureTool(){};
	virtual~DistanceMeasureTool() {};
        virtual int getRelyOnParam(std::vector<int> &nums);
        virtual int initTool(Json::Value json);
        virtual void updateTool(Json::Value json){};
	virtual int run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result);
        virtual int run(cv::Mat src,cv::Mat &dst ){};
        virtual int emptyTool(){};

};
#endif 
