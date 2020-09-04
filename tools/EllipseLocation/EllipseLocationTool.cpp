#include <stdio.h>
#include <unistd.h>
#include "ivs_algorithm_utils.h"
#include"EllipseLocationTool.h"
ToolBase *CreateTool(void)
{
    return (new EllipseLocationTool());
}
int EllipseLocationTool::initTool(Json::Value json)
{
	cout << "start init EllipseLocation" <<endl;
    if(json["edgeDimension"].isNull()||json["edgeThreshold"].isNull()||json["maxRadius"].isNull()||json["minRadius"].isNull()||json["maxRoundness"].isNull()||json["minRoundness"].isNull()||json["detectArea"].isNull()||json["templateArea"].isNull()||json["score"].isNull()||json["distanceThreshold"].isNull()||json["selectType"].isNull())
    {
        cout<<"Error:EllipseLocationTool tool init error:NULL!"<<endl;
        //throw exception();
        return 0;
    }
    unsigned int rely_size =  json["toolRelyNums"].size();
        for (unsigned int i = 0; i < rely_size; ++i)
        {
              int num=json["toolRelyNums"][i].asInt();
              relyNums.push_back(num);
        }
     edgeDimension=json["edgeDimension"].asInt();
     edgeThreshold=json["edgeThreshold"].asInt();
     maxRadius=json["maxRadius"].asInt();
     minRadius=json["minRadius"].asInt();
     maxRoundness=json["maxRoundness"].asInt();
     minRoundness=json["minRoundness"].asInt();
     distanceThreshold=json["distanceThreshold"].asInt();
     selectType=json["selectType"].asInt();
     detectArea.p1.x=json["detectArea"][0].asInt();
     detectArea.p1.y=json["detectArea"][1].asInt();
     detectArea.p2.x=json["detectArea"][2].asInt();
     detectArea.p2.y=json["detectArea"][3].asInt();
     detectArea.p3.x=json["detectArea"][4].asInt();
     detectArea.p3.y=json["detectArea"][5].asInt();
     detectArea.p4.x=json["detectArea"][6].asInt();
     detectArea.p4.y=json["detectArea"][7].asInt();
     templateArea.p1.x=json["templateArea"][0].asInt();
     templateArea.p1.y=json["templateArea"][1].asInt();
     templateArea.p2.x=json["templateArea"][2].asInt();
     templateArea.p2.y=json["templateArea"][3].asInt();
     templateArea.p3.x=json["templateArea"][4].asInt();
     templateArea.p3.y=json["templateArea"][5].asInt();
     templateArea.p4.x=json["templateArea"][6].asInt();
     templateArea.p4.y=json["templateArea"][7].asInt();
     center.x = json["centerX"].asDouble();
     center.y = json["centerY"].asDouble();
     tempDatum_point.x = json["templatePointX"].asDouble();
     tempDatum_point.y = json["templatePointY"].asDouble();
     
    cout << "init EllipseLocation down" <<endl;
    return 1;
         
}
int EllipseLocationTool::getRelyOnParam(vector<int> &nums) 
{
        //通过依赖工具参数获取当前工具的结果编号nums
        if(relyNums.size()==0)
          return 0;
        nums=relyNums;
        return 1;
}
int EllipseLocationTool::run(Mat src,vector<Json::Value> relyOnResults, Json::Value &result)
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
		//cout<<"++++++"<<detectArea.p1<<" "<<detectArea.p2<<" "<<detectArea.p3<<" "<<detectArea.p4<<endl;
		move_templateArea(center, center_angle, tempDatum_point, testDatum_point,detectArea);
		//cout<<"++++++"<<detectArea.p1<<" "<<detectArea.p2<<" "<<detectArea.p3<<" "<<detectArea.p4<<endl;
		cv::line(image,detectArea.p1,detectArea.p2,Scalar(0,255,0),4);
		cv::line(image,detectArea.p1,detectArea.p3,Scalar(0,255,0),4);
		cv::line(image,detectArea.p4,detectArea.p2,Scalar(0,255,0),4);
		cv::line(image,detectArea.p4,detectArea.p3,Scalar(0,255,0),4);
		Mat im;
	        toolIsOk=ellipseDetect(src,detectArea, colorChange,edgeDimension,edgeThreshold, maxRadius,minRadius, maxRoundness, minRoundness, distanceThreshold, score,selectType,ellipse);    //用ellipse存储检测结果
		resize(image,im,Size(1000,800));
		//imshow("image",im);waitKey();
		
	}
	catch (exception e)
	{
	        cout<<e.what()<<" :EllipseLocation fail!"<< endl;   //捕获异常，然后程序结束
        	return 0;
	}
	time.stop();
        int times=time.duration();
	//将匹配结果以Json形式存储
        std::string resultStr="{\"ToolIsOk\":"+std::to_string(toolIsOk)+",\"ellipseCenterX\":"+std::to_string(ellipse.center.x)+",\"ellipseCentrY\":"+std::to_string(ellipse.center.y)+",\"ellipseRadius1\":"+std::to_string(ellipse.radius1)+",\"ellipseRadius2\":"+std::to_string(ellipse.radius2)+",\"ellipseAngle\":"+std::to_string(ellipse.angle)+",\"RunTimes\":"+std::to_string(times)+"}";
        std::cout<<resultStr.c_str()<<std::endl; 
        Json::Reader reader;
        reader.parse(resultStr, result);
	
        return 1;

}

bool EllipseLocationTool::isInside(RotatedRect box,Point2f point){  //判断点point是否在box内部
	float b=box.size.width/2;                         
	float a=box.size.height/2;
	float x=box.center.x;
	float y=box.center.y;
	float x0=point.x;
	float y0=point.y;
	float re=(x0-x)*(x0-x)/(a*a)+(y0-y)*(y0-y)/(b*b);
	if(re<=1){
		return true;
	}
	return false;	
	
}

bool EllipseLocationTool::isInRect(scanArea area,Point2f p){      //判断点p是否在矩形area内部
	Point2f p1=area.p1;
	Point2f p2=area.p2;
	Point2f p3=area.p3;
	Point2f p4=area.p4;
	Point2f AB=Point2f(p2.y-p1.y,p2.x-p1.x);
	Point2f AE=Point2f(p.y-p1.y,p.x-p1.x);
	Point2f DC=Point2f(p3.y-p4.y,p3.x-p4.x);
	Point2f DE=Point2f(p.y-p4.y,p.x-p4.x);
	Point2f AC=Point2f(p3.y-p1.y,p3.x-p1.x);
	Point2f DB=Point2f(p2.y-p4.y,p2.x-p4.x);
	float r1=AB.x*AE.y-AB.y*AE.x;
	float r2=DC.x*DE.y-DC.y*DE.x;
	float r3=AC.x*AE.y-AC.y*AE.x;
	float r4=DB.x*DE.y-DB.y*DE.x;
	if(r1*r2>=0&&r3*r4>=0){
		return true;
	}
	return false;	
}

int EllipseLocationTool::ellipseDetect(Mat src,scanArea area, int colorChange,int edgeDimension,int edgeThreshold,int maxRadius,int minRadius,int maxRoundness,int minRoundness,int distanceThreshold,int score,int selectType,Ellipse ellipse_){
	Mat canny_src;
	if(src.channels()>1){
		cvtColor(src,src, COLOR_BGR2GRAY);
	}
	score=0;
	Mat mat=Mat::zeros(src.rows,src.cols,CV_8UC1);
	vector<vector<Point>> resultsCon;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	blur(src,canny_src,Size(3,3));
	Canny(canny_src,canny_src,edgeThreshold,2*edgeThreshold,3);
	workPoints(canny_src,area,mat);   //将非ROI区域置零,返回mat
	//imshow("mat",mat);waitKey();
	findContours(mat,contours,hierarchy,RETR_CCOMP, CHAIN_APPROX_SIMPLE); //寻找轮廓
	if(contours.size()==0){
		cout<<"no ecllipse!"<<endl;
		return 0;
	}
	
	vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合
        Point2f rect[4];
        for(int i=0; i<contours.size(); i++)
        {
           box[i] = minAreaRect(Mat(contours[i]));  //计算每个轮廓最小外接矩形
           box[i].points(rect);
           float a=box[i].size.width/2;                         
	   float b=box[i].size.height/2;
	   float x0=box[i].center.x;
	   float y0=box[i].center.y;
	   float area1 = contourArea(contours[i], true);
	   float area2 = CV_PI*a*b;
	  
	   if(fabs(area1-area2)<=area2/20&&area1>20&&min(a,b)>=minRadius&&max(a,b)<=maxRadius&&a/b>=minRoundness&&a/b<=maxRoundness&&contours[i].size()>5){
		resultsCon.push_back(contours[i]);         //将符合条件的轮廓放入resultsCon
	   }
	}
	if(resultsCon.size()==0){
		cout<<"no ecllipse!"<<endl;
		return 0;
	}
	vector<int> color_;
	vector<float> score_;
	vector<float> area_;
	vector<RotatedRect> ellipses;
	for(int i=0; i<resultsCon.size(); i++){

		RotatedRect tempbox = fitEllipse(resultsCon[i]);
		//ellipse(image,tempbox,Scalar(0,255,0),2,8);
		ellipses.push_back(tempbox);			
		float a=tempbox.size.width/2;    //注意区分长和宽!!!                 
		float b=tempbox.size.height/2;
		float x0=tempbox.center.x;
		float y0=tempbox.center.y;
		float angle=tempbox.angle;
		area_.push_back(CV_PI*a*b);	  //面积

		int hit=0;			  //得分
		float a1=a+distanceThreshold;     
		float a2=a-distanceThreshold;
		float b1=b+distanceThreshold;
		float b2=b-distanceThreshold;
		
		for(int j=0;j<resultsCon[i].size();j++){  
			//circle(image,resultsCon[i][j],1,Scalar(0,0,255),-1);
			float x=resultsCon[i][j].x;
			float y=resultsCon[i][j].y;
			float mmin=(x0-x)*(x0-x)/(b1*b1)+(y0-y)*(y0-y)/(a1*a1);
			float mmax=(x0-x)*(x0-x)/(b2*b2)+(y0-y)*(y0-y)/(a2*a2);
			if(mmin<1&&mmax>1){
				hit++;
			}
		}
		score_.push_back((float)hit/resultsCon[i].size()*100);

		RotatedRect extendBox = tempbox;                 //颜色判断
		extendBox.size.width+=10;
		extendBox.size.height+=10; 
		//ellipse(image,extendBox,Scalar(0,0,255),1,8);
		Rect boundRect = boundingRect(Mat(resultsCon[i]));
		float exColor=0,inColor=0;
		float exnum=0,innum=0;	
		for(int j=boundRect.tl().x-5;j<=boundRect.br().x+5;j++){
			for(int k=boundRect.tl().y-5;k<=boundRect.br().y+5;k++){
				if(j<0||j>src.cols||k<0||k>src.rows)
					continue;
				if(isInside(tempbox,Point2f(j,k))){
					//circle(image,Point2f(j,k),1,Scalar(0,0,255),-1);
					int elem=(int)src.at<uchar>(Point2f(j,k));
					inColor+=elem;
					innum++;
				}
				else if(isInside(extendBox,Point2f(j,k))){
					//circle(image,Point2f(j,k),1,Scalar(0,255,0),-1);
					int elem=(int)src.at<uchar>(Point2f(j,k));
					exColor+=elem;
					exnum++;
				}
			}
		}
		inColor/=innum;
		exColor/=exnum;
		if(inColor<exColor)
			color_.push_back(-1);
		else
			color_.push_back(1);
	}
	int sign=-1;
	if(selectType==1){            //面积最大
		float maxArea=0;
		for(int i=0;i<resultsCon.size();i++){
			if((colorChange==0||colorChange==color_[i])&&area_[i]>maxArea&&score_[i]>=60){
				sign=i;
				maxArea=area_[i];
			}
		}	
	}
	else if(selectType==0){       //面积最小
		float minArea=500000;
		for(int i=0;i<resultsCon.size();i++){
			if((colorChange==0||colorChange==color_[i])&&area_[i]<minArea&&score_[i]>=60){
				sign=i;
				minArea=area_[i];
			}
		}	
	}
	if(sign==-1){
		cout<<"no ellipse!!!"<<endl;	
		return 0;
	}
	float b=ellipses[sign].size.width/2;                   
	float a=ellipses[sign].size.height/2;
	float x0=ellipses[sign].center.x;
	float y0=ellipses[sign].center.y;
	float angle=ellipses[sign].angle;
	cv::ellipse(image,ellipses[sign],Scalar(0,255,0),2,8);
	//cv::line(image,ellipses[sign].center,Point2f(ellipses[sign].center.x+10*cos(angle),ellipses[sign].center.y+10*sin(angle)),Scalar(0,255,0),1,8);
	//cv::line(image,ellipses[sign].center,Point2f(ellipses[sign].center.x+10*cos(angle+CV_PI/2),ellipses[sign].center.y+10*sin(angle+CV_PI/2)),Scalar(0,255,0),1,8);
	cout<<"position :"<<ellipses[sign].center<<endl;
	cout<<"angle :"<<ellipses[sign].angle<<endl;
	cout<<"color :"<<color_[sign]<<endl;
	cout<<"area  :"<<area_[sign]<<endl;
	cout<<"score :"<<score_[sign]<<endl;
	ellipse_.center=ellipses[sign].center;
	ellipse_.angle=ellipses[sign].angle;
	ellipse_.radius1=ellipses[sign].size.width;
	ellipse_.radius2=ellipses[sign].size.height;
	return 1;
}

int EllipseLocationTool::move_templateArea(Point2f center, float center_angle, Point2f tempDatum_point,Point2f testDatum_point,scanArea &detectArea){           //根据特征定位结果移动检测区域
	
	Point2f h11=templateArea.p1;
	Point2f h12=templateArea.p2;  //模板坐标
	Point2f h13=templateArea.p3;
	Point2f h14=templateArea.p4;
	Point2f h1,h2,h3,h4;                       //旋转后的中心
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
	float x_=testDatum_point.x-tempDatum_point.x; //计算相对位移
	float y_=testDatum_point.y-tempDatum_point.y;
	detectArea.p1.x=h1.x+x_;if(detectArea.p1.x<0) detectArea.p1.x=0;if(detectArea.p1.x>=image.cols) detectArea.p1.x==image.cols-1;
	detectArea.p1.y=h1.y+y_;if(detectArea.p1.y<0) detectArea.p1.y=0;if(detectArea.p1.y>=image.rows) detectArea.p1.x==image.rows-1;
	detectArea.p2.x=h2.x+x_;if(detectArea.p2.x<0) detectArea.p2.x=0;if(detectArea.p2.x>=image.cols) detectArea.p2.x==image.cols-1;
	detectArea.p2.y=h2.y+y_;if(detectArea.p2.y<0) detectArea.p2.y=0;if(detectArea.p2.y>=image.rows) detectArea.p2.y==image.rows-1;
	detectArea.p3.x=h3.x+x_;if(detectArea.p3.x<0) detectArea.p3.x=0;if(detectArea.p3.x>=image.cols) detectArea.p3.x==image.cols-1;
	detectArea.p3.y=h3.y+y_;if(detectArea.p3.y<0) detectArea.p3.y=0;if(detectArea.p3.y>=image.rows) detectArea.p3.y==image.rows-1;
	detectArea.p4.x=h4.x+x_;if(detectArea.p4.x<0) detectArea.p4.x=0;if(detectArea.p4.x>=image.cols) detectArea.p4.x==image.cols-1;
	detectArea.p4.y=h4.y+y_;if(detectArea.p4.y<0) detectArea.p4.y=0;if(detectArea.p4.y>=image.rows) detectArea.p4.y==image.rows-1;

	return 1;
}

void EllipseLocationTool::workPoints(Mat src,scanArea area,Mat &mat){  //根据检测区域将非检测区域置零
	int connectivity=4;
	LineIterator iterator1(src, area.p1,area.p3, connectivity, false);
	LineIterator iterator2(src, area.p2,area.p4, connectivity, false);
	int count=min(iterator1.count,iterator2.count);
	vector<Point> vec1,vec2;
	for(int i=0;i<count;i++){
		vec1.push_back(iterator1.pos());
		vec2.push_back(iterator2.pos());
		iterator1++;iterator2++;
	}
	
	for(int i=0;i<vec1.size();i++){
		LineIterator it(src, vec1[i],vec2[i], connectivity, false);
		for(int j=0;j<it.count;j++){
			mat.at<uchar>(it.pos())=src.at<uchar>(it.pos());
			it++;
		}
	}
}
