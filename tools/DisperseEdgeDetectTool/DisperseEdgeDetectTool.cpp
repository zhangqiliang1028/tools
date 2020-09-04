#include <stdio.h>
#include <unistd.h>
#include "ivs_algorithm_utils.h"
#include"DisperseEdgeDetectTool.h"
ToolBase *CreateTool(void)
{
    return (new DisperseEdgeDetectTool());
}
int DisperseEdgeDetectTool::initTool(Json::Value json)
{
	cout << "start init DisperseEdgeDetect" <<endl;
    if(json["selectPointCount"].isNull()||json["blurWidth"].isNull()||json["colorChange"].isNull()||json["edgeThreshold"].isNull()||json["selectionMethod"].isNull()||json["blurLength"].isNull()||json["templateArea"].isNull()||json["depthThreshold"].isNull()||json["roiHeight"].isNull()||json["pointnumThreshold"].isNull()||json["minLength"].isNull()||json["maxLength"].isNull()||json["angleDiffer"].isNull())
    {
        cout<<"Error:DisperseEdgeDetectTool tool init error:NULL!"<<endl;
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
     templateArea.p1.x=json["templateArea"][0].asInt();
     templateArea.p1.y=json["templateArea"][1].asInt();
     templateArea.p2.x=json["templateArea"][2].asInt();
     templateArea.p2.y=json["templateArea"][3].asInt();
     center.x = json["centerX"].asDouble();
     center.y = json["centerY"].asDouble();
     tempDatum_point.x = json["templatePointX"].asDouble();
     tempDatum_point.y = json["templatePointY"].asDouble();
     roiHeight=json["roiHeight"].asInt();
     pointnumThreshold=json["pointnumThreshold"].asInt();
     minLength=json["minLength"].asInt();
     maxLength=json["maxLength"].asInt();
     angleDiffer=json["angleDiffer"].asInt();
    cout << "init DisperseEdgeDetect down" <<endl;
    return 1;
         
}
int DisperseEdgeDetectTool::getRelyOnParam(vector<int> &nums) 
{
        //通过依赖工具参数获取当前工具的结果编号nums
        if(relyNums.size()==0)
          return 0;
        nums=relyNums;
        return 1;
}
int DisperseEdgeDetectTool::run(Mat src,vector<Json::Value> relyOnResults, Json::Value &result)
{
	TimeTracker time;
        time.start();
	Mat midImg,dstR;
	src.copyTo(image);
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
		move_templateArea(center, center_angle, tempDatum_point,testDatum_point,detectArea);
	        toolIsOk=edgeDetect(src,detectArea, selectPointCount, blurWidth,blurLength,roiHeight,edgeThreshold,depthThreshold, pointnumThreshold,selectionMethod,colorChange,minLength,maxLength,angleDiffer,line);    //用line存储检测结果
		
		imshow("image",image(Rect(400,0,1000,1000)));waitKey();
		
	}
	catch (exception e)
	{
	        cout<<e.what()<<" :DisperseEdgeDetect fail!"<< endl;   //捕获异常，然后程序结束
        	return 0;
	}
	time.stop();
        int times=time.duration();
	//将匹配结果以Json形式存储
        std::string resultStr="{\"ToolIsOk\":"+std::to_string(toolIsOk)+",\"linepoint1X\":"+std::to_string(line.p1.x)+",\"linepoint1Y\":"+std::to_string(line.p1.y)+",\"linepoint2X\":"+std::to_string(line.p2.x)+",\"linepoint2Y\":"+std::to_string(line.p2.y)+",\"linecenterX\":"+std::to_string(line.center.x)+",\"linecenterY\":"+std::to_string(line.center.y)+",\"RunTimes\":"+std::to_string(times)+"}";
        std::cout<<resultStr.c_str()<<std::endl; 
        Json::Reader reader;
        reader.parse(resultStr, result);
	
        return 1;

}

void DisperseEdgeDetectTool::workPoints(Mat src,scanArea detectArea, int selectPointCount, int blurWidth,int roiHeight,vector<vector<Point2f>>& results)//选出所有起作用的点
{
	
	float x1=detectArea.p1.x;
	float y1=detectArea.p1.y;
	float x2=detectArea.p2.x;
	float y2=detectArea.p2.y;
	float angle=atan((y2-y1)/(x2-x1));
	Point2f center=detectArea.p1;    //旋转中心
	float length=sqrt((y2-y1)*(y2-y1)+(x2-x1)*(x2-x1));
	Point2f h1,h2,h3,h4;            //水平对照矩形区域顶点坐标
	Point2f p1,p2,p3,p4;	        //倾斜矩形区域顶点坐标
	h1.x=x1;h1.y=y1-roiHeight/2;
	h2.x=x1;h2.y=y1+roiHeight/2;
	h3.x=x1+length;h3.y=h1.y;
	h4.x=x1+length;h4.y=h2.y;
	p1.x = (h1.x - center.x)*cos(angle) - (h1.y - center.y)*sin(angle) + center.x;
	p1.y = (h1.x - center.x)*sin(angle) + (h1.y - center.y)*cos(angle) + center.y;
	p2.x = (h2.x - center.x)*cos(angle) - (h2.y - center.y)*sin(angle) + center.x;
	p2.y = (h2.x - center.x)*sin(angle) + (h2.y - center.y)*cos(angle) + center.y;
	p3.x = (h3.x - center.x)*cos(angle) - (h3.y - center.y)*sin(angle) + center.x;
	p3.y = (h3.x - center.x)*sin(angle) + (h3.y - center.y)*cos(angle) + center.y;
	p4.x = (h4.x - center.x)*cos(angle) - (h4.y - center.y)*sin(angle) + center.x;
	p4.y = (h4.x - center.x)*sin(angle) + (h4.y - center.y)*cos(angle) + center.y;
		
	int connectivity=4;
	LineIterator iterator1(src, p1,p3, connectivity, false);
	LineIterator iterator2(src, p2,p4, connectivity, false);
	int count=min(iterator1.count,iterator2.count);
	vector<Point2f> vec1,vec2;
	for(int i=0;i<count;i++){
		vec1.push_back(iterator1.pos());
		vec2.push_back(iterator2.pos());
		iterator1++;iterator2++;
	}
	float step=(count-blurWidth)/(selectPointCount-1);
	vector<Point2f> vertex;
	for(int i=blurWidth/2;i<count;i+=step){
		for(int j=i-blurWidth/2;j<=i+blurWidth/2;j++){
			LineIterator it(src, vec1[j],vec2[j], connectivity, false);
			vector<Point2f> point_;
			for(int k=0;k<it.count;k++){
				point_.push_back(it.pos());
				if((j==i-blurWidth/2||j==i+blurWidth/2)&&(k==0||k==it.count-1)){
					vertex.push_back(it.pos());			
				}
				it++;			
			}
			results.push_back(point_);
		}
		
	}
	for(int i=0;i<vertex.size();i+=4){
		cv::line(image,vertex[i],vertex[i+1],Scalar(0,255,0),1);
		cv::line(image,vertex[i+1],vertex[i+3],Scalar(0,255,0),1);
		cv::line(image,vertex[i+3],vertex[i+2],Scalar(0,255,0),1);
		cv::line(image,vertex[i],vertex[i+2],Scalar(0,255,0),1);
	}
}

int DisperseEdgeDetectTool::pointsDetection(Mat src,scanArea area,vector<vector<Point2f>> scanPoints, int edgeThreshold, int selectionMethod, int colorChange, int blurLenth,int blurWidth,int depthThreshold,vector<Point2f> &points )
{			
	float* blurKernel = (float*)malloc((blurLenth * 2 + 1) * sizeof(float)); //申请滤波核的存储器
	vector<vector<NNode>> nodes;
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
	
	for (int i = blurWidth/2; i < scanPoints.size(); i+=blurWidth){
		vector<NNode> node_;
		for(int j=0;j<scanPoints[i].size();j++){
			NNode node0;
			node0.point=scanPoints[i][j];
			float u=0;
			for(int k=i-blurWidth/2;k<=i+blurWidth/2;k++){
				int n=(int)src.at<uchar>(scanPoints[k][j]);
				u=u+n;
			}
			u=u/blurWidth;
			node0.value=u;
			node_.push_back(node0);
		}
		nodes.push_back(node_);
	}
	
	for (int i = 0; i < nodes.size(); i++)
	{
		
		for (int j = blurLenth; j < nodes[i].size() - blurLenth; j++)//滤波窗口
		{
			float blurResult = 0;
			int index = 0;
			for (int k = j - blurLenth; k <= j + blurLenth; k++)
			{
				blurResult += nodes[i][k].value * blurKernel[index];
				index++;
			}
			nodes[i][j].result=(float)blurResult/(blurLenth);
		}
	}
	/*for(int i=0;i<nodes.size();i++){
		for(int j=0;j<nodes[i].size();j++){
			cout<<nodes[i][j].result<<" ";
		}
		cout<<endl;
	}*/
	//若【选点方式】为峰值点，选择边缘对比度大于【边缘阈值】中最显著的。
	if (selectionMethod == 0)
	{
		//颜色变换判断，选择最大值
		if (colorChange == 0)//0为黑到白,差分为正是黑到白的边缘
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				int max_value = 0;Point p=Point(0,0);
				for(int j=0;j<nodes[i].size();j++){
					if(max_value<nodes[i][j].result&&nodes[i][j].result>=edgeThreshold){
						p=nodes[i][j].point;
						max_value=nodes[i][j].result;
					}
				}
				if(p!=Point(0,0))
				points.push_back(p);
			}
		}
		else if (colorChange == 1)//1为白到黑,差分为负是白到黑边缘
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				int min_value = 0;Point p=Point(0,0);
				for(int j=0;j<nodes[i].size();j++){
					if(min_value>nodes[i][j].result&&nodes[i][j].result<=-edgeThreshold){
						p=nodes[i][j].point;
						min_value=nodes[i][j].result;
					}
				}
				if(p!=Point(0,0))
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
				Point p=Point(0,0);
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
				if(p!=Point(0,0))
				points.push_back(p);
			}
		}
		else if (colorChange == 1)//1为白到黑,差分为负是白到黑边缘
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				Point p=Point(0,0);
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
				if(p!=Point(0,0))
				points.push_back(p);
			}
		}
	}

	return 1;
}

int  DisperseEdgeDetectTool::edgeDetect(Mat src,scanArea area, int selectPointCount, int blurWidth,int blurLength,int roiHeight,int edgeThreshold,int depthThreshold,int pointnumThreshold,int selectionMethod,int colorChange,int minLength,int maxLength,int angleDiffer,Line &line){
	if(src.channels()>1){
		cvtColor(src, src, COLOR_BGR2GRAY);//转灰度图
	}
	vector<vector<Point2f>> results;
	vector<Point2f> points;
	workPoints(src,area, selectPointCount, blurWidth, roiHeight, results);
	pointsDetection( src,area,results,  edgeThreshold, selectionMethod,colorChange, blurLength, blurWidth,depthThreshold,points );
	/*for(int i=0;i<points.size();i++){
		cout<<points[i]<<endl;
	}*/

	if(points.size()<pointnumThreshold){
		cout<<"检测出的点数过少，未检测到边缘!"<<endl;
		return 0;
	}
	cv::Vec4f line_; 
	cv::fitLine(points, line_, DIST_L2, 0, 0.01,0.01);
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
	float ll=sqrt((end.y-begin.y)*(end.y-begin.y)+(end.x-begin.x)*(end.x-begin.x));
	if((ll<minLength&&maxLength!=0)||(ll>maxLength&&maxLength!=0)){
		cout<<"长度不符合，未检测到边缘!"<<endl;
		return 0;
	}
	if(fabs(atan((area.p2.y-area.p1.y)/(area.p2.x-area.p1.x))-atan((end.y-begin.y)/(end.x-begin.x)))>angleDiffer*CV_PI/180){
		cout<<"角度不符合，未检测到边缘!"<<endl;
		return 0;
	}
	line.p1=begin;
	line.p2=end;
	line.center=Point2f((begin.x+end.x)/2,(begin.y+end.y)/2);
	cv::line(image,begin,end,Scalar(0,255,0),1);
	return 1;
}

int DisperseEdgeDetectTool::move_templateArea(Point2f center, float center_angle, Point2f tempDatum_point,Point2f testDatum_point,scanArea &detectArea){
	
	Point2f h11=templateArea.p1;
	Point2f h12=templateArea.p2;  //模板坐标
	Point2f h1,h2;                       //旋转后的中心
	if (center_angle > 180)
	{
		center_angle = 360 - center_angle;
		double angle = center_angle * CV_PI / 180; // 弧度
		h1.x = (h11.x - center.x)*cos(angle) - (h11.y - center.y)*sin(angle) + center.x;
		h1.y = (h11.x - center.x)*sin(angle) + (h11.y - center.y)*cos(angle) + center.y;
		h2.x = (h12.x - center.x)*cos(angle) - (h12.y - center.y)*sin(angle) + center.x;
		h2.y = (h12.x - center.x)*sin(angle) + (h12.y - center.y)*cos(angle) + center.y;
		
	}
	else
	{
		double angle = center_angle * CV_PI / 180; // 弧度
		h1.x = (h11.x - center.x)*cos(angle) - (h11.y - center.y)*sin(angle) + center.x;
		h1.y = (h11.y - center.y)*cos(angle) + (h11.x - center.x)*sin(angle) + center.y;
		h2.x = (h12.x - center.x)*cos(angle) - (h12.y - center.y)*sin(angle) + center.x;
		h2.y = (h12.y - center.y)*cos(angle) + (h12.x - center.x)*sin(angle) + center.y;
		
	}
	float x_=testDatum_point.x-tempDatum_point.x; //计算相对位移
	float y_=testDatum_point.y-tempDatum_point.y;
	detectArea.p1.x=h1.x+x_;
	detectArea.p1.y=h1.y+y_;
	detectArea.p2.x=h2.x+x_;
	detectArea.p2.y=h2.y+y_;

	return 1;
}

