#include "ToolBase.h"

class GreyChangeTool :public ToolBase
{
/*
        greyChange灰度变换：提高亮度，但是使得灰度值较低域内的像素点值变得更大些
        Mat &src:既为输入图像，又为输出
        int threshold:用于区分灰度值较低的区域
        int d1:用于灰度值较低的区域的值增长倍数
        int d2:用于非灰度值较低的区域的值增长倍数
        */
private:
        //cv::Mat src;
        int threshold;
        float d1;
        float d2;
      
public:       
       GreyChangeTool(){};
       virtual~GreyChangeTool(){};
       virtual int getRelyOnParam(std::vector<int> &nums){};
       virtual int initTool(Json::Value json);
       virtual int run(cv::Mat src,  cv::Mat &dst);       
       virtual void updateTool(Json::Value json){};
       virtual int run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result){};
       virtual int emptyTool(){};
};
