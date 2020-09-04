#include "DistanceMeasureTool.h"
#include "ivs_algorithm_utils.h"
#include <string>
ToolBase *CreateTool(void)
{
    return (new DistanceMeasureTool());
}

int DistanceMeasureTool::initTool(Json::Value json)
{
        if(json["toolRelyNums"].isNull()||json["type"].isNull())
        {
           std::cout<<"Error:DistanceMeasureTool tool init error:NULL!"<<std::endl;
           //throw std::exception();
           return 0;
        }
        std::cout<<"DistanceMeasure tool init start"<<std::endl;
	//初始化参数
        type = json["type"].asInt();
        unsigned int rely_size =  json["toolRelyNums"].size();
        for (unsigned int i = 0; i < rely_size; ++i)
        {
              int num=json["toolRelyNums"][i].asInt();
              relyNums.push_back(num);
        }
	
        std::cout<<"DistanceMeasure tool init down"<<std::endl;
        return 1;
}
int DistanceMeasureTool::getRelyOnParam(std::vector<int> &nums)
{
        //通过依赖工具参数获取当前工具的结果编号nums
        if(relyNums.size()==0)
          return 0;
        nums=relyNums;
        return 1;      
}


int DistanceMeasureTool::run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result)
{
        TimeTracker time;
        time.start();
        float distance=0;
        int distanceflag=0;
        lines={};
        try
        {
        	for (int i = 0; i < relyOnResults.size(); i++)
		{
                     Line line;
                     //cv::Point p;
                     //从依赖工具的结果中读参数
                     if(!relyOnResults[i]["linePoint1X"].isNull()||!relyOnResults[i]["linePoint1Y"].isNull()||!relyOnResults[i]["linePoint2X"].isNull()||!relyOnResults[i]["linePoint2Y"].isNull())
                     {
                        line.p1.x = relyOnResults[i]["linePoint1X"].asInt();
                        line.p1.y = relyOnResults[i]["linePoint1Y"].asInt();
                        line.p2.x = relyOnResults[i]["linePoint2X"].asInt();
                        line.p2.y = relyOnResults[i]["linePoint2Y"].asInt();
                        lines.push_back(line);
                     }
               	     //if(relyOnResults[i]["point1X"].isNull()||relyOnResults[i]["point1Y"].isNull())
                     //{
                        //p.x = relyOnResults[i]["point1X"].asInt();
                        //p.y = relyOnResults[i]["point1Y"].asInt();
                        //points.push_back(p);
                     //}
                 }
       	 	//std::cout<<lines[0].p1<<std::endl;
       	 	//std::cout<<lines[1].p1<<std::endl;
		switch (type)
        	{
           	    case 0:
                        if(lines.size() < 2)
                      	{
                            std::cout<<"Error:DistanceMeasureTool lines error !"<<std::endl;
                            throw std::exception();
                         
                       	}
              	        distanceflag=GetLineDistance(lines[0], lines[1],distance);
               		break;
                    case 1:
               	        //distanceflag=GetLineDistance(points[0], points[1],distance);
              	        break;
          	    case 2:
               	        //distanceflag=GetLineDistance(points[0], lines[0],distance);
              	        break;
           	    default:
              	        distanceflag=GetLineDistance(lines[0], lines[1],distance);
              	        break;
        	}
        }
        catch (std::exception e)
        {
           std::cout<<e.what()<<" :DistanceMeasure fail!"<< std::endl;   //捕获异常，然后程序结束
           return 0;
        }
        time.stop();
        int times=time.duration();
        int toolIsOk = 0; 
        if(distanceflag)
           toolIsOk = 1;
	//将匹配结果以Json形式存储
        std::cout<<"distance:"<<distance<<std::endl;
        std::string resultStr="{\"ToolIsOk\":"+std::to_string(toolIsOk)+",\"diatance\":"+std::to_string(zdistance)+",\"RunTimes\":"+std::to_string(times)+"}";
        std::cout<<resultStr.c_str()<<std::endl; 
        Json::Reader reader;
        reader.parse(resultStr, result);
        return 1;

}
/*
线与线距离getLineDistance：计算一条直线的中点到另一条直线的距离
Line l1:输入线1
Line l2: 输入线2
输出：float，线1与线2之间的距离
*/
int DistanceMeasureTool::GetLineDistance(Line l1, Line l2,float &distance)
{
	//一条直线的中点到另一条直线的距离
	cv::Point p = cv::Point((l1.p1.x + l1.p2.x) / 2, (l1.p1.y + l1.p2.y) / 2);
	float A = l2.p2.y - l2.p1.y;
	float B = l2.p1.x - l2.p2.x;
	float C = l2.p2.x*l2.p1.y - l2.p1.x*l2.p2.y;
        distance=fabs((A*p.x + B * p.y + C) / sqrtf(A*A + B * B));
	return 1;
}

