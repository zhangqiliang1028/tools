#include <stdio.h>
#include <unistd.h>
#include "ivs_algorithm_utils.h"
#include"CircleDetectTool.h"
ToolBase *CreateTool(void)
{
    return (new CircleDetectTool());
}
int CircleDetectTool::initTool(Json::Value json)
{
	cout << "start init CircleDetect" <<endl;
    if(json["selectPointCount"].isNull()||json["blurWidth"].isNull()||json["colorChange"].isNull()||json["edgeThreshold"].isNull()||json["selectionMethod"].isNull()||json["blurLength"].isNull()||json["detectArea"].isNull()||json["templateArea"].isNull()||json["score"].isNull()||json["depthThreshold"].isNull())
    {
        cout<<"Error:CircleDetectTool tool init error:NULL!"<<endl;
        //throw exception();
        return 0;
    }
    unsigned int rely_size =  json["toolRelyNums"].size();
        for (unsigned int i = 0; i < rely_size; ++i)
        {
              int num=json["toolRelyNums"][i].asInt();
              relyNums.push_back(num);
        }
     selectPointCount=json["selectPointCount"].asInt();
     blurWidth=json["blurWidth"].asInt();
     colorChange=json["colorChange"].asInt();
     edgeThreshold=json["edgeThreshold"].asInt();
     selectionMethod=json["selectionMethod"].asInt();
     blurLength=json["blurLength"].asInt();
     depthThreshold=json["depthThreshold"].asInt();
     detectArea.center.x=json["detectArea"][0].asInt();
     detectArea.center.y=json["detectArea"][1].asInt();
     detectArea.exRadius=json["detectArea"][2].asInt();
     detectArea.inRadius=json["detectArea"][3].asInt();
     templateArea.center.x=json["templateArea"][0].asInt();
     templateArea.center.y=json["templateArea"][1].asInt();
     templateArea.exRadius=json["templateArea"][2].asInt();
     templateArea.inRadius=json["templateArea"][3].asInt();
     center.x = json["centerX"].asDouble();
     center.y = json["centerY"].asDouble();
     tempDatum_point.x = json["templatePointX"].asDouble();
     tempDatum_point.y = json["templatePointY"].asDouble();
     
    cout << "init CircleDetect down" <<endl;
    return 1;
         
}
int CircleDetectTool::getRelyOnParam(vector<int> &nums) 
{
        //通过依赖工具参数获取当前工具的结果编号nums
        if(relyNums.size()==0)
          return 0;
        nums=relyNums;
        return 1;
}
int CircleDetectTool::run(Mat src,vector<Json::Value> relyOnResults, Json::Value &result)
{
	TimeTracker time;
        time.start();
	Mat midImg,dstR;
	src.copyTo(image);
	Circle circle;
	int toolIsOk=0;
	try
	{
		 for (int i = 0; i < relyOnResults.size(); i++)
	    {
                //std::cout<<"CenterX:"<<relyOnResults[i]["CenterX"].asInt()<<std::endl;
                //std::cout<<"CenterY:"<<relyOnResults[i]["CenterY"].asInt()<<std::endl;
		//从依赖工具的结果中读参数
                testDatum_point.x = relyOnResults[i]["CenterX"].asDouble();
                testDatum_point.y = relyOnResults[i]["CenterY"].asDouble();
		center_angle=relyOnResults[i]["Angle"].asDouble();
	    }

		move_circle(center, center_angle, tempDatum_point,testDatum_point,detectArea);
		cv::circle(image,detectArea.center,detectArea.exRadius,Scalar(0,0,255));
		cv::circle(image,detectArea.center,detectArea.inRadius,Scalar(0,0,255));
		Mat im;
		
	        toolIsOk=circleDetect( src,detectArea, selectPointCount, blurWidth, blurLength, edgeThreshold, selectionMethod,colorChange,depthThreshold,circle);    //用circle存储检测结果
		//cv::circle(image,circle.center,circle.radius,Scalar(0,0,255),2);
		resize(image,im,Size(1000,800));
		imshow("image",im);waitKey();
		
	}
	catch (exception e)
	{
	        cout<<e.what()<<" :CircleDetect fail!"<< endl;   //捕获异常，然后程序结束
        	return 0;
	}
	time.stop();
        int times=time.duration();
	//将匹配结果以Json形式存储
        std::string resultStr="{\"ToolIsOk\":"+std::to_string(toolIsOk)+",\"circleCenterX\":"+std::to_string(circle.center.x)+",\"circleCentrY\":"+std::to_string(circle.center.y)+",\"circleRadius\":"+std::to_string(circle.radius)+",\"RunTimes\":"+std::to_string(times)+"}";
        std::cout<<resultStr.c_str()<<std::endl; 
        Json::Reader reader;
        reader.parse(resultStr, result);
	
        return 1;

}

void CircleDetectTool::workPoints(Mat src,scanArea area, int selectPointCount, int blurWidth,vector<vector<Point>>&results)//选出所有起作用的点
{
	float step;
	if(selectPointCount!=0){
		step=2*CV_PI/(selectPointCount+1);
	}
	else{
		step=CV_PI/180;
	}
	float angle=0;
	vector<Point> vex,vin;
	int x0,y0,x1,y1;
	for(int i=0;i<selectPointCount;i++){                   //获取所有扫描线的起点和终点
		x0=cos(angle)*area.exRadius+area.center.x;
		y0=sin(angle)*area.exRadius+area.center.y;
		x1=cos(angle)*area.inRadius+area.center.x;
		y1=sin(angle)*area.inRadius+area.center.y;
		vex.push_back(Point(x0,y0));
		vin.push_back(Point(x1,y1));
		angle+=step;
		
	}
	int connectivity = 4;
	int count;
	for(int i=0;i<vin.size();i++){                         //根据扫描线的起点和终点找到所有扫描线所在坐标
		LineIterator iterator(src, vex[i], vin[i], connectivity, false);
		count=iterator.count;
		vector<Point> point;
		for(int j=0;j<count;j++){
			point.push_back(iterator.pos());
			if(j%10==0&&i%3==0){
				cv::circle(image,iterator.pos(),1,Scalar(0,0,255),-1);
			}
			iterator++;
		}
		results.push_back(point);
	}
}

int CircleDetectTool::pointsDetection(Mat src,scanArea area,vector<vector<Point>> scanPoints, int edgeThreshold, int selectionMethod, int colorChange, int blurLenth,int blurWidth,int depthThreshold,vector<Point2f> &points )
{
				
	float* blurKernel = (float*)malloc((blurLenth * 2 + 1) * sizeof(float)); //申请滤波核的存储器
	vector<vector<node>> nodes;
	int index=0;
	for ( index=0;index < blurLenth;index++)
	{
		blurKernel[index] = -1;
	}
	blurKernel[index] = 0;
	index++;
	for (; index < blurLenth * 2 + 1; index++)
	{
		blurKernel[index] = 1;
	}

	for (int i = 0; i < scanPoints.size(); i++){                           //扫描线宽度滤波
		vector<node> node_;
		for(int j=0;j<scanPoints[i].size()-1;j++){
			node node0;
			node0.point=scanPoints[i][j];
			float u=0;
			int x_=(abs(scanPoints[i][j+1].x-scanPoints[i][j].x));
			int y_=(abs(scanPoints[i][j+1].y-scanPoints[i][j].y));
			for(int k=1;k<=blurWidth/2;k++){
				int elem=(int)src.at<uchar>(Point(scanPoints[i][j].x-y_*k,scanPoints[i][j].y-x_*k));
				u=u+elem;
			}
			u=u+(int)src.at<uchar>(scanPoints[i][j]);
			for(int k=1;k<=blurWidth/2;k++){
				int elem=(int)src.at<uchar>(Point(scanPoints[i][j].x+y_*k,scanPoints[i][j].y+x_*k));
				u=u+elem;
			}
			u=u/blurWidth;
			node0.value=u;
			node_.push_back(node0);
		}
		nodes.push_back(node_);
	}

	for (int i = 0; i < nodes.size(); i++)
	{
		for (int j = blurLenth; j < nodes[i].size() - blurLenth; j++)  //差分滤波窗口
		{
			int index = 0;
			float blurResult = 0;
			for (int k = j - blurLenth; k <= j + blurLenth; k++)
			{
				blurResult += nodes[i][k].value * blurKernel[index];
				index++;
			}
			nodes[i][j].result=blurResult/(blurLenth*2+1);
		}
	}

	//若【选点方式】为峰值点，选择边缘对比度大于【边缘阈值】中最显著的。
	if (selectionMethod == 0)
	{
		//颜色变换判断，选择最大值
		if (colorChange == 0)//0为黑到白,差分为正是黑到白的边缘
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				int max_value = 0;Point p=area.center;
				for(int j=0;j<nodes[i].size();j++){
					if(max_value<nodes[i][j].result&&nodes[i][j].result>=edgeThreshold){
						p=nodes[i][j].point;
						max_value=nodes[i][j].result;
					}
				}
				points.push_back(p);
			}
		}
		else if (colorChange == 1)//1为白到黑,差分为负是白到黑边缘
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				int min_value = 0;Point p=area.center;
				for(int j=0;j<nodes[i].size();j++){
					if(min_value>nodes[i][j].result&&nodes[i][j].result<=-edgeThreshold){
						p=nodes[i][j].point;
						min_value=nodes[i][j].result;
					}
				}
				points.push_back(p);
			}
		}
	}
	//若【选点方式】为最先找到的点，选择第一个大于【边缘阈值】的点。
	else if (selectionMethod == 1)
	{
		//首先找到第一个与颜色判断相对应的点
		//颜色变换判断，选择最大值
		if (colorChange == 0)//0为黑到白,差分为正是黑到白的边缘
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				Point p=area.center;
				for(int j=0;j<nodes[i].size();j++){
					int f=1;
					if(nodes[i][j].result>edgeThreshold){
						for(int k=j+1;k<=j+depthThreshold&&k<nodes[i].size();k++){ //深度阈值判断
							if(nodes[i][k].result<-edgeThreshold){
								f=0;
								break;
							}
						}
						if(f=1){
							p=nodes[i][j].point;
							break;
						}
					}
				}
				points.push_back(p);
			}
		}
		else if (colorChange == 1)//1为白到黑,差分为负是白到黑边缘
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				Point p=area.center;
				for(int j=0;j<nodes[i].size();j++){
					int f=1;
					if(nodes[i][j].result<-edgeThreshold){
						for(int k=j+1;k<=j+depthThreshold&&k<nodes[i].size();k++){  //深度阈值判断
							if(nodes[i][k].result>edgeThreshold){
								f=0;
								break;
							}
						}
						if(f=1){
							p=nodes[i][j].point;
							break;
						}
					}
				}
				points.push_back(p);
			}
		}
	}
	return 1;
}

bool CircleDetectTool::fittingCircle(std::vector<cv::Point2f>& pts, cv::Point2f& center, float& radius)
{
	center = cv::Point(0, 0);
	radius = 0.0;
	if (pts.size() < 3) return false;;

	double sumX = 0.0;
	double sumY = 0.0;
	double sumX2 = 0.0;
	double sumY2 = 0.0;
	double sumX3 = 0.0;
	double sumY3 = 0.0;
	double sumXY = 0.0;
	double sumX1Y2 = 0.0;
	double sumX2Y1 = 0.0;
	const double N = (double)pts.size();
	for (int i = 0; i < pts.size(); ++i)
	{
		double x = pts.at(i).x;
		double y = pts.at(i).y;
		double x2 = x * x;
		double y2 = y * y;
		double x3 = x2 * x;
		double y3 = y2 * y;
		double xy = x * y;
		double x1y2 = x * y2;
		double x2y1 = x2 * y;

		sumX += x;
		sumY += y;
		sumX2 += x2;
		sumY2 += y2;
		sumX3 += x3;
		sumY3 += y3;
		sumXY += xy;
		sumX1Y2 += x1y2;
		sumX2Y1 += x2y1;
	}
	double C = N * sumX2 - sumX * sumX;
	double D = N * sumXY - sumX * sumY;
	double E = N * sumX3 + N * sumX1Y2 - (sumX2 + sumY2) * sumX;
	double G = N * sumY2 - sumY * sumY;
	double H = N * sumX2Y1 + N * sumY3 - (sumX2 + sumY2) * sumY;

	double denominator = C * G - D * D;
	if (std::abs(denominator) < DBL_EPSILON) return false;
	double a = (H * D - E * G) / (denominator);
	denominator = D * D - G * C;
	if (std::abs(denominator) < DBL_EPSILON) return false;
	double b = (H * C - E * D) / (denominator);
	double c = -(a * sumX + b * sumY + sumX2 + sumY2) / N;

	center.x = a / (-2);
	center.y = b / (-2);
	radius = std::sqrt(a * a + b * b - 4 * c) / 2;
	return true;
}

int CircleDetectTool::circleDetect(Mat src,scanArea area, int selectPointCount, int blurWidth,int blurLength,int edgeThreshold, int selectionMethod,int colorChange,int depthThreshold,Circle &circle){
	if(src.channels()>1){
		cvtColor(src,src, COLOR_BGR2GRAY);
	}
	vector<vector<Point>> results;
	vector<Point2f> points;
	workPoints(src,area, selectPointCount, blurWidth,results);
	pointsDetection( src,area,results,edgeThreshold,selectionMethod,colorChange,blurLength,blurWidth,depthThreshold,points );

	if(points.size()<3){
		cout<<"未检测到圆!"<<endl;
		return 0;
	}
	
	//RotatedRect box = fitEllipse(points);              //椭圆拟合
	//circle.center=Point(box.center);
	//circle.radius=(box.size.width+box.size.height)/2;

	fittingCircle(points,circle.center,circle.radius);	//最小二乘法拟合圆形
	                                                     
	float x0=circle.center.x;                                //得分判断
	float y0=circle.center.y;
	float r=circle.radius;      
	int hit=0;
	for(int i=0;i<points.size();i++){  
		            
		float x_=fabs(points[i].x-x0);
		float y_=fabs(points[i].y-y0);
		cout<<sqrt(x_*x_+y_*y_)<<endl;
		if(fabs(sqrt((x_*x_)+(y_*y_))-r)<=2){
			hit++;
		}
	}
	score=((float)hit/selectPointCount*100);
	cout<<"+++score is : "<<score<<endl;
	//cv::circle(image,circle.center,r,Scalar(0,255,0),2);
	if(score<60){
		cout<<"未检测到圆"<<endl;
		return 0;
	}

	return 1;
}

int CircleDetectTool::move_circle(Point2f center, float center_angle, Point2f tempDatum_point,Point2f testDatum_point,scanArea &detectArea){
	
	Point2f h11=templateArea.center;  //模板中心
	Point2f h1;                       //旋转后的中心
	if (center_angle > 180)
	{
		center_angle = 360 - center_angle;
		double angle = center_angle * CV_PI / 180; // 弧度
		h1.x = (h11.x - center.x)*cos(angle) - (h11.y - center.y)*sin(angle) + center.x;
		h1.y = (h11.x - center.x)*sin(angle) + (h11.y - center.y)*cos(angle) + center.y;
		
	}
	else
	{
		double angle = center_angle * CV_PI / 180; // 弧度
		h1.x = (h11.x - center.x)*cos(angle) - (h11.y - center.y)*sin(angle) + center.x;
		h1.y = (h11.y - center.y)*cos(angle) + (h11.x - center.x)*sin(angle) + center.y;
		
	}
	float x_=testDatum_point.x-tempDatum_point.x; //计算相对位移
	float y_=testDatum_point.y-tempDatum_point.y;
	detectArea.center.x=h1.x+x_;
	detectArea.center.y=h1.y+y_;

	return 1;
}

