#ifndef IVS_TOOL_H
#define IVS_TOOL_H
#include <json/json.h>
//#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

//工具基类
class ToolBase
{
private:
        
public:
        virtual int getRelyOnParam(std::vector<int> &nums)=0;       
	virtual int initTool(Json::Value json)=0 ;
        virtual void updateTool(Json::Value json) =0;
	virtual int run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result)=0;
        virtual int run(cv::Mat src,cv::Mat &dst )=0;
        virtual int emptyTool()=0;
};
extern "C" ToolBase *CreateTool(void);
#endif 
