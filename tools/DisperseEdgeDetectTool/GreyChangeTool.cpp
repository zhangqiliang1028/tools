#include <stdio.h>
#include <unistd.h>
#include"GreyChangeTool.h"

ToolBase *CreateTool(void)
{
    return (new GreyChangeTool());
}
int GreyChangeTool::initTool(Json::Value json)
{
    if(json["threshold"].isNull()||json["d1"].isNull()||json["d2"].isNull())
    {
        std::cout<<"Error:GreyChangeTool tool init error:NULL!"<<std::endl;
        //throw std::exception();
        return 0;
    }
    threshold =json["threshold"].asInt();
    d1=json["d1"].asFloat();
    d2=json["d2"].asFloat();
    std::cout << "init GreyChange down" << std::endl;
    return 1;
         
}

int GreyChangeTool::run(cv::Mat src,  cv::Mat &dst)
{
    std::cout << "start GreyChange" << std::endl;
    try
    {
        if(src.channels() != 1)
              cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
        for (int i = 0; i < src.rows; i++)
        {
            for (int j = 0; j < src.cols; j++)
	    {
		    if (src.at<uchar>(i, j) < threshold)
			    dst.at<uchar>(i, j) = src.at<uchar>(i, j)*d1;
		    else
			    dst.at<uchar>(i, j) = src.at<uchar>(i, j)*d2;
	    }
        }
    }
    catch (std::exception e)
    {
        std::cout<<e.what()<<" :GreyChange fail!"<< std::endl;   //捕获异常，然后程序结束
        return 0;
    }
    std::cout << "GreyChange down" << std::endl;
    return 1;
}
