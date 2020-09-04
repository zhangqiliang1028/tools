#include <stdio.h>
#include <unistd.h>
#include "ivs_algorithm_utils.h"
#include"EdgeDetectTool.h"
ToolBase *CreateTool(void)
{
    return (new EdgeDetectTool());
}
int EdgeDetectTool::initTool(Json::Value json)
{
	cout << "start init PointsDetect" <<endl;
    if(json["selectPointCount"].isNull()||json["blurWidth"].isNull()||json["colorChange"].isNull()||json["edgeThreshold"].isNull()||json["selectionMethod"].isNull()||json["blurLength"].isNull()||json["minLength"].isNull()||json["maxLength"].isNull()||json["angleDiffer"].isNull()||json["area"].isNull())
    {
        cout<<"Error:EdgeDetectTool tool init error:NULL!"<<endl;
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
     edgeDepth=json["edgeDepth"].asInt();
     minLength=json["minLength"].asInt();                 
     maxLength=json["maxLength"].asInt();	         
     angleDiffer=json["angleDiffer"].asInt();               
     area.p1=Point(json["area"][0].asInt(),json["area"][1].asInt());
     area.p2=Point(json["area"][2].asInt(),json["area"][3].asInt());
     area.p3=Point(json["area"][4].asInt(),json["area"][5].asInt());
     area.p4=Point(json["area"][6].asInt(),json["area"][7].asInt());
     /*detectRect.x = json["detectRect"]["extRectX"].asInt();
     detectRect.y = json["detectRect"]["extRectY"].asInt();
     detectRect.width = json["detectRect"]["extRectWidth"].asInt();
     detectRect.height = json["detectRect"]["extRectHeight"].asInt();*/
     center.x = json["centerX"].asDouble();
     center.y = json["centerY"].asDouble();
     tempDatum_point.x = json["templatePointX"].asDouble();
     tempDatum_point.y = json["templatePointY"].asDouble();
     sample_width=json["sampleImageWidth"].asInt();
     sample_height=json["sampleImageHeight"].asInt();
     angleDiffer=json["angleDiffer"].asInt();
    cout << "init EdgeDetect down" <<endl;
    return 1;
         
}
int EdgeDetectTool::getRelyOnParam(vector<int> &nums) 
{
        //通过依赖工具参数获取当前工具的结果编号nums
        if(relyNums.size()==0)
          return 0;
        nums=relyNums;
        return 1;
}
int EdgeDetectTool::run(Mat src,vector<Json::Value> relyOnResults, Json::Value &result)
{
	TimeTracker time;
        time.start();
	Mat midImg,dstR;
	src.copyTo(image);
	
	int toolIsOk=0;
	Line line;
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
		
		//cv::circle(image,testDatum_point,20,0);
		Rotate_rect(center, center_angle, tempDatum_point,testDatum_point,sample_area);
		cv::line(image,sample_area.p1,sample_area.p2,Scalar(0,0,255),1);
		cv::line(image,sample_area.p3,sample_area.p4,Scalar(0,0,255),1);
		
	        toolIsOk=edgeDetect(src,sample_area,selectPointCount, blurWidth,blurLength,edgeThreshold,  				selectionMethod,colorChange,edgeDepth,minLength,maxLength,angleDiffer,line);    //用line存储检测结果
		Mat im;
		cv::resize(image,im,Size(800,800));
		imshow("image",im);waitKey();
		
	}
	catch (exception e)
	{
	        cout<<e.what()<<" :PointsDetect fail!"<< endl;   //捕获异常，然后程序结束
        	return 0;
	}
	time.stop();
        int times=time.duration();
	//将匹配结果以Json形式存储
        std::string resultStr="{\"ToolIsOk\":"+std::to_string(toolIsOk)+",\"linePoint1X\":"+std::to_string(line.p1.x)+",\"linePoint1Y\":"+std::to_string(line.p1.y)+",\"linePoint2X\":"+std::to_string(line.p2.x)+",\"linePoint2Y\":"+std::to_string(line.p2.y)+",\"RunTimes\":"+std::to_string(times)+"}";
        std::cout<<resultStr.c_str()<<std::endl; 
        Json::Reader reader;
        reader.parse(resultStr, result);
	
        return 1;

}

Point EdgeDetectTool::getEndPoint(vector<Point> points,int xy,int be){  //获取点集中的最左最右最上最下点
	int index=0;
	if(xy==0&&be==0){
		for(int i=1;i<points.size();i++){
			if(points[i].x<points[index].x){
				index=i;
			}
		}
	}
	else if(xy==0&&be==1){
		for(int i=1;i<points.size();i++){
			if(points[i].x>points[index].x){
				index=i;
			}
		}
	}
	else if(xy==1&&be==0){
		for(int i=1;i<points.size();i++){
			if(points[i].y<points[index].y){
				index=i;
			}
		}
	}
	else if(xy==1&&be==1){
		for(int i=1;i<points.size();i++){
			if(points[i].y>points[index].y){
				index=i;
			}
		}
	}
	return points[index];
}
void EdgeDetectTool::workPoints(Mat src,scanArea area1, int selectPointCount, int blurWidth,vector<vector<Point>> &results) //选出所有ROI中能用到的点
{
	int connectivity = 4;

	LineIterator iterator1(src, area1.p1, area1.p2, connectivity, false);
	LineIterator iterator2(src, area1.p3,area1.p4, connectivity, false);
	vector<Point> startPoints;
	vector<Point> endPoints;
	int count=min(iterator1.count,iterator2.count);
	for(int i=0;i<count;i++){
		startPoints.push_back(iterator1.pos());
		endPoints.push_back(iterator2.pos());
		iterator1++;iterator2++;
	}
	
	if(selectPointCount==0){
		step=1;
	}
	else{
		step = count / (selectPointCount+1);
	}
	
	for(int i=blurWidth/2;i<count-blurWidth/2;i+=step){
		for(int j=i-blurWidth/2;j<=i+blurWidth/2;j++){
			LineIterator it(src, startPoints[j],endPoints[j], connectivity, false);
			vector<Point> result_;
			for(int k=0;k<it.count;k++){
				result_.push_back(it.pos());
				if(k%5==0&&i==j){
					circle(image,it.pos(),1,Scalar(0),-1);
				}
				it++;
			}
			results.push_back(result_);
		}
	}		
	
}

void EdgeDetectTool::pointsDetection(Mat src,vector<vector<Point>> scanPoints, int edgeThreshold, int selectionMethod, int colorChange, int blurLength,int blurWidth,int edgeDepth,vector<Point> &points )                             //边缘点检测
{	
	int* blurKernel = (int*)malloc((blurLength * 2 + 1) * sizeof(int)); //申请滤波核的存储器
	vector<vector<node> > nodes;
	int index=0;
	for ( index=0;index < blurLength;index++)
	{
		blurKernel[index] = -1;
	}
	blurKernel[index] = 0;
	index++;
	for (; index < blurLength * 2 + 1; index++)
	{
		blurKernel[index] = 1;
	}
	
	for (int i = blurWidth/2; i < scanPoints.size()-blurWidth/2; i+=blurWidth){    //扫描线宽度均值
		vector<node> node_;
		for(int j=0;j<scanPoints[i].size()-blurLength;j++){
			node node0;
			node0.point=scanPoints[i][j];
			node0.result=0;
			node0.value=(int)src.at<uchar>(scanPoints[i][j]);
			double u=0;
			for(int k=i-blurWidth/2;k<=i+blurWidth/2;k++){
				
				int intensity = (int)src.at<uchar>(scanPoints[k][j]);
				u=u+intensity;
			}
			u=u/blurWidth;
			node0.value=u;
			node_.push_back(node0);		
		}
		
		nodes.push_back(node_);
	}
	
	for (int i = 0; i < nodes.size(); i++)
	{
		int blurResult = 0;
		for (int j = blurLength; j < nodes[i].size() - blurLength; j++)//差分滤波窗口
		{
			int index = 0;blurResult=0;
			for (int k = j - blurLength; k <= j + blurLength; k++)
			{
				blurResult += (nodes[i][k].value * blurKernel[index]);	
				index++;
			}
			nodes[i][j].result=blurResult/(double)(blurLength*2+1);
		}
	}

	//若选点方式为峰值点，选择边缘对比度大于边缘阈值中最显著的。
	if (selectionMethod == 0){
	
		//颜色变换判断，选择最大值
		if (colorChange == 0)//0为黑到白,差分为正是黑到白的边缘
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				int max_value = 0;
				Point p;
				node n1;n1.result=0;
				for(int j=0;j<nodes[i].size();j++){
					if(max_value<nodes[i][j].result&&nodes[i][j].result>edgeThreshold){
						int isResult=1;
						if(isResult==0) continue;
						n1=nodes[i][j];
						max_value=nodes[i][j].result;	
					}
				}
				if(n1.result!=0){
					points.push_back(n1.point);
				}
			}
		}
		else if (colorChange == 1)//1为白到黑,差分为负是白到黑边缘
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				int min_value = 0;
				Point p;
				node n1;n1.result=0;
				for(int j=0;j<nodes[i].size();j++){
					if(min_value>nodes[i][j].result&&nodes[i][j].result<-edgeThreshold){
						int isResult=1;
						if(isResult==0) continue;
						n1=nodes[i][j];
						min_value=nodes[i][j].result;
					}
				}
				if(n1.result!=0){
					points.push_back(n1.point);
				}
			}
		}
	}
	//若选点方式为最先找到的点，选择第一个大于边缘阈值的点。
	else if (selectionMethod == 1)
	{
		//颜色变换判断，选择最大值
		if (colorChange == 0)//0为黑到白,差分为正是黑到白的边缘
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				Point p;
				node n1;n1.result=0;
				for(int j=0;j<nodes[i].size();j++){
					if(nodes[i][j].result>edgeThreshold){
						int isResult=1;
						for(int k=j+1;k<j+edgeDepth&&k<=nodes[i].size();k++){ //判断边缘深度
							if(nodes[i][k].result<-edgeThreshold){
								isResult=0;break;
							}
						}
						if(isResult==0) continue;
						n1=nodes[i][j];break;	
					}
				}
				if(n1.result!=0){
					points.push_back(n1.point);
				}	
			}
		}
		else if (colorChange == 1)//1为白到黑,差分为负是白到黑边缘
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				Point p;
				node n1;n1.result=0;
				for(int j=0;j<nodes[i].size();j++){
					if(nodes[i][j].result<-edgeThreshold){
						int isResult=1;
						for(int k=j+1;k<j+edgeDepth&&k<=nodes[i].size();k++){  //判断边缘深度
							if(nodes[i][k].result>edgeThreshold){
								isResult=0;break;
							}
						}
						if(isResult==0) continue;
						n1=nodes[i][j];break;
					}
				}
				if(n1.result!=0){
					points.push_back(n1.point);
				}
			}
		}
	}
}
int EdgeDetectTool::edgeDetect(Mat src,scanArea sample_area,int selectPointCount, int blurWidth,int blurLength,int edgeThreshold, int selectionMethod,int colorChange,int edgeDepth,int minLength,int maxLength,int angle,Line &line){

	int getLine=1; //判断是否检测到边缘
	Point begin,end;//边缘两个端点
	//将原图转化为灰度图
	if(src.channels()>1)
		cvtColor(src,src, COLOR_BGR2GRAY); 
  
		vector<vector<Point>> results;
		vector<Point> points;
		workPoints(src,sample_area,selectPointCount, blurWidth,results);
		pointsDetection(src,results,edgeThreshold,selectionMethod, colorChange, blurLength, blurWidth,edgeDepth,points );

	//边缘检测点处理
	if(points.size()<3){
		getLine=0;cout<<"未检测到边缘!"<<endl;return 0;
	}
	else{	
		area_=sample_area;
		//对边缘点根据与扫描起点的距离大小排序，计算所有边缘点到扫描起点的距离
		vector<Point> init_points=points;
		sort(points.begin(),points.end(),cmp);  
		vector<int> dis;
		
		for(int i=0;i<points.size();i++){       
			int d;
			if(sample_area.p1.x==sample_area.p2.x){
				d=abs(sample_area.p1.x-points[i].x);
			}
			else{
				double k=(double)(sample_area.p2.y-sample_area.p1.y)/(sample_area.p2.x-sample_area.p1.x);
				double b=sample_area.p1.y-k*sample_area.p1.x;
				d=(int)(abs(k*points[i].x+b-points[i].y)/sqrt(k*k+1));
			}
			dis.push_back(d);
		}
		
		//寻找距离满足要求的一簇边缘点
		vector<Point> points_;  
		int max=0,flag=1,i,finish=0;
		for( i=1;i<dis.size();i++){   
			if(dis[i]-dis[i-1]<=1){             ////此处可以判断边缘角度是否满足条件
				flag++;
			}
			else{
				if(max<flag){max=flag;finish=i-1;}
				flag=1;
			}	
		}
		if(max<flag){max=flag;finish=i-1;}
		for(int i=finish-max+1;i<=finish;i++){
			points_.push_back(points[i]);
		
		}
		//消除孤立点
		for(int i=0;i<points_.size();i++){
			int flag=0;
			for(int j=0;j<points_.size();j++){
				if(j==i)continue;
				if((points_[i].y-points_[j].y)*(points_[i].y-points_[j].y)+(points_[i].x-points_[j].x)*(points_[i].x-points_[j].x)<=4*step*step+2){flag++;break;}
			}
			if(flag==0){
				vector<Point>::iterator it = points_.begin()+i;
    				points_.erase(it);
			}
		}
		if(points_.size()<3){
			getLine=0;cout<<"未检测到边缘!!"<<endl;return 0;
		}		
		
		//直线拟合
		Vec4f line0; 
		fitLine(points_, line0,2, 0, 0.01, 0.01);  //计算出的直线line为cv::Vec4f类型。注意points为空时函数出错。
		double cos_theta = line0[0];
		double sin_theta = line0[1];
		double x0 = line0[2], y0 = line0[3];
            
		//寻找拟合直线的两个端点  
	        if(line0[0]<0.05){          //边缘是竖直线
			Point point1=getEndPoint(points_,1,0),point2=getEndPoint(points_,1,1);
			begin=Point(x0,point1.y);
			end=Point(x0,point2.y);
		}                                                                         
		else if(line0[0]>=0.05){   //边缘不是竖直线
			double k = sin_theta / cos_theta;//浮点数例外(除以0),（核心已转出）
			double b = y0 - k * x0;
			Point point1=getEndPoint(points_,0,0),point2=getEndPoint(points_,0,1);
			//begin=point1;
			//end=point2;
			begin=Point(point1.x,point1.x*k+b);
			end=Point(point2.x,point2.x*k+b);
		}
		int lineLength=(int)sqrt((begin.y-end.y)*(begin.y-end.y)+(begin.x-end.x)*(begin.x-end.x));  //判断边缘长度是否满足
		if(maxLength!=0&&(lineLength>maxLength||lineLength<minLength)){    
			getLine=0;
		
		}
		if(fabs(atan((sample_area.p1.y-sample_area.p2.y)/(sample_area.p1.x-sample_area.p2.x))-atan((end.y-begin.y)/(end.x-begin.x)))>angleDiffer*CV_PI/180){
			getLine=0;
		}
	}
	if(getLine==1){
		line.p1=begin;
		line.p2=end;
		cv::line(image,begin,end,Scalar(0),1);
		return 1;
	}
	else{
		cout<<"未检测到边缘!!!"<<endl;
		 return 0;
	}	
	
	return 0;
}

int  EdgeDetectTool::Rotate_rect(cv::Point2f center, double center_angle, cv::Point2f tempDatum_point,cv::Point2f testDatum_point,scanArea &sample_area)
{
	cv::Point h11, h12, h13, h14;
	cv::Point h1, h2, h3, h4;
	//旋转前坐标，依次为左上、左下、右上、右下

        if(center_angle==0){
		sample_area.p1.x=area.p1.x;
        sample_area.p1.y=area.p1.y;
        sample_area.p2.x=area.p2.x;
        sample_area.p2.y=area.p2.y;
        sample_area.p3.x=area.p3.x;
        sample_area.p3.y=area.p3.y;
        sample_area.p4.x=area.p4.x;
        sample_area.p4.x=area.p4.y;
		//return 0;	
	}
	h11=area.p1;
	h12=area.p2;
	h13=area.p3;
	h14=area.p4;
	h1=area.p1;
	h2=area.p2;
	h3=area.p3;
	h4=area.p4;
	//旋转后点坐标
	if(center_angle!=0){
	if (center_angle > 180)
	{
		center_angle = 360 - center_angle;
		double angle = center_angle * CV_PI / 180; // 弧度
		h1.x = (h11.x - center.x)*cos(angle) - (h11.y - center.y)*sin(angle) + center.x;
		h1.y = (h11.x - center.x)*sin(angle) + (h11.y - center.y)*cos(angle) + center.y;
		h2.x = (h12.x - center.x)*cos(angle) - (h12.y - center.y)*sin(angle) + center.x;
		h2.y = (h12.x - center.x)*sin(angle) + (h12.y - center.y)*cos(angle) + center.y;
		h3.x = (h13.x - center.x)*cos(angle) - (h13.y - center.y)*sin(angle) + center.x;
		h3.y = (h13.x - center.x)*sin(angle) + (h13.y - center.y)*cos(angle) + center.y;
		h4.x = (h14.x - center.x)*cos(angle) - (h14.y - center.y)*sin(angle) + center.x;
		h4.y = (h14.x - center.x)*sin(angle) + (h14.y - center.y)*cos(angle) + center.y;
	}
	else
	{
		double angle = center_angle * CV_PI / 180; // 弧度
		h1.x = (h11.x - center.x)*cos(angle) - (h11.y - center.y)*sin(angle) + center.x;
		h1.y = (h11.y - center.y)*cos(angle) + (h11.x - center.x)*sin(angle) + center.y;
		h2.x = (h12.x - center.x)*cos(angle) - (h12.y - center.y)*sin(angle) + center.x;
		h2.y = (h12.y - center.y)*cos(angle) + (h12.x - center.x)*sin(angle) + center.y;
		h3.x = (h13.x - center.x)*cos(angle) - (h13.y - center.y)*sin(angle) + center.x;
		h3.y = (h13.y - center.y)*cos(angle) + (h13.x - center.x)*sin(angle) + center.y;
		h4.x = (h14.x - center.x)*cos(angle) - (h14.y - center.y)*sin(angle) + center.x;
		h4.y = (h14.y - center.y)*cos(angle) + (h14.x - center.x)*sin(angle) + center.y;
	}
	}
	//计算相对位移
	cv::Point pp1 = cv::Point(h1.x - tempDatum_point.x, h1.y - tempDatum_point.y);
	cv::Point pp2 = cv::Point(h2.x - tempDatum_point.x, h2.y - tempDatum_point.y);
	cv::Point pp3 = cv::Point(h3.x - tempDatum_point.x, h3.y - tempDatum_point.y);
	cv::Point pp4 = cv::Point(h4.x - tempDatum_point.x, h4.y - tempDatum_point.y);
	//新的坐标
	h1.x = testDatum_point.x + pp1.x;
	h1.y = testDatum_point.y + pp1.y;
	h2.x = testDatum_point.x + pp2.x;
	h2.y = testDatum_point.y + pp2.y;
	h3.x = testDatum_point.x + pp3.x;
	h3.y = testDatum_point.y + pp3.y;
	h4.x = testDatum_point.x + pp4.x;
	h4.y = testDatum_point.y + pp4.y;

	sample_area.p1=h1;
	sample_area.p2=h2;
	sample_area.p3=h3;
	sample_area.p4=h4;
	return 1;
}

