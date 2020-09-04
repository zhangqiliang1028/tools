#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<iostream>
#include<vector>
#include<math.h>
using namespace cv;
using namespace std;
/*Mat image;         输入图像
Mat ROI;             ROI区域
int selectPointCount;选点数量
int blurWidth;       滤波宽度
int edgeThreshold;  边缘阈值
int selectionMethod; 选点方式，0为峰值点，1为最先找到点
int colorChange;     颜色变换，0为黑到白，1为白到黑
int blurLength;      差分滤波器长
int edgeDepth;       边缘深度
int minLength;       最小边缘长度
int maxLength;       最大边缘长度
int angleDiffer;     测出边缘与ROI的最大角度差异*/
int step;
Mat image; //原图
Mat ROI; //在ROI区域
Mat mask;//获取ROI使用的掩膜
//Mat getROI(Mat &src,struct scanArea area);
//void workPoints(Mat &src,scanArea area, int selectPointCount, int blurWidth,vector<vector<Point>> &results);
//void pointsDetection(Mat &src,vector<vector<Point>> scanPoints, int edgeThreshold, int selectionMethod, int colorChange, int blurLength,int blurWidth,int edgeDepth,vector<Point> &points );
struct scanArea{     //ROI区域表示
	Point p1;
	Point p2;
	Point p3;
	Point p4;

};
struct scanArea area; //扫描区域，也是ROI的四个端点
typedef struct Node{   //存储处理的点的坐标和值
	Point point;//位置
	int value;//求平均值后的结果
	int result=0;//滤波后的结果
}node;
bool cmp(Point &m,Point &n){
		
	int d1,d2;
	if(area.p1.x==area.p2.x){
		d1=abs(area.p1.x-m.x);	
			d2=abs(area.p1.x-n.x);
		}
	else{
		double k=(area.p2.y-area.p1.y)/(area.p2.x-area.p1.x);
		double b=area.p1.y-k*area.p1.x;
		d1=(int)abs(k*m.x+b-m.y)/(k*k+1);
		d2=(int)abs(k*n.x+b-n.y)/(k*k+1);
	}
	return d1<d2;		

}
Point getEndPoint(vector<Point> points,int xy,int be){  //获取点集中的最左最右最上最下点
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
void workPoints(Mat &src,scanArea area, int selectPointCount, int blurWidth,vector<vector<Point>> &results) //选出所有ROI中能用到的点
{

	int connectivity = 4;
	LineIterator iterator1(src, area.p1, area.p2, connectivity, false);
	LineIterator iterator2(src, area.p3,area.p4, connectivity, false);
	int count=iterator1.count;
	step = count / (selectPointCount);
	int flag=0;int f=0;
	
	while(f!=count){
		if(flag<blurWidth){
			LineIterator iterator(src,iterator1.pos(),iterator2.pos(),connectivity, false);
			vector<Point> result_;
			for(int j=0;j<iterator.count;j++){
				result_.push_back(iterator.pos());
				iterator++;
			}
			results.push_back(result_);
			flag++;
			f++;
			iterator1++;iterator2++;
		}
		else if(flag==blurWidth){
			flag=0;	
			if(step-blurWidth>=0){
				for(int j=0;j<step-blurWidth;j++){
					iterator1++;iterator2++;
					f++;if(f==count)break;
				}
			}
			else{
				step=blurWidth; //当blurWidth*selectPointCount>count时，调整step.
				//for(int j=0;j<blurWidth-step;j++){
				//	iterator1--;iterator2--;
				//	f--;
				//}
			}
		}
	}		
}

void pointsDetection(Mat src,vector<vector<Point>> scanPoints, int edgeThreshold, int selectionMethod, int colorChange, int blurLength,int blurWidth,int edgeDepth,vector<Point> &points )                             //边缘点检测
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
	for (int i = blurWidth/2; i < scanPoints.size(); i+=blurWidth){    //均值滤波
		vector<node> node_;
		for(int j=0;j<scanPoints[i].size();j++){
			node node0;
			node0.point=scanPoints[i][j];
			node0.result=0;
			node0.value=(int)src.at<uchar>(scanPoints[i][j]);
			int u=0;
			for(int k=i-blurWidth/2;k<=i+blurWidth/2;k++){
				
				int intensity = (int)src.at<uchar>(scanPoints[k][j]);
				u=u+intensity;
			}
			u/=blurWidth;
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
			nodes[i][j].result=blurResult;
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
void edgeDetect(Mat src,scanArea area, int selectPointCount, int blurWidth,int blurLength,int edgeThreshold, int selectionMethod,int colorChange,int edgeDepth,int minLength,int maxLength,int angle){

	int getLine=1; //判断是否检测到边缘
	Point begin,end;//边缘两个端点
	//将原图转化为灰度图
	if(src.channels()>1)
		cvtColor(src,src, COLOR_BGR2GRAY); 
  
		vector<vector<Point>> results;
		vector<Point> points;
		workPoints(src,area, selectPointCount, blurWidth,results);
		pointsDetection(src,results,edgeThreshold,selectionMethod, colorChange, blurLength, blurWidth,edgeDepth,points );

	//边缘检测点处理
	if(points.size()<3){
		getLine=0;
	}
	else{	
		
		//对边缘点根据与扫描起点的距离大小排序，计算所有边缘点到扫描起点的距离
		vector<Point> init_points=points;
		sort(points.begin(),points.end(),cmp);  
		vector<int> dis;
		for(int i=0;i<points.size();i++){       
			int d;
			if(area.p1.x==area.p2.x){
				d=abs(area.p1.x-points[i].x);	
			}
			else{
				double k=(area.p2.y-area.p1.y)/(area.p2.x-area.p1.x);
				double b=area.p1.y-k*area.p1.x;
				d=(int)abs(k*points[i].x+b-points[i].y)/(k*k+1);
			}
			dis.push_back(d);
		}
		//寻找距离满足要求的一簇边缘点
		vector<Point> points_;  
		int max=0,flag=1,i,finish=0;
		for( i=1;i<dis.size();i++){   
			if(dis[i]-dis[i-1]<=2){             ////此处可以判断边缘角度是否满足条件
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
		for(int i=0;i<points_.size();i++){
			circle(ROI,points_[i],2,Scalar(0,0,255),-1);
		}
		//消除孤立点
		/*for(int i=0;i<points_.size();i++){
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
			getLine=0;
		}*/		
		
		//直线拟合
		Vec4f line0; 
		fitLine(points_, line0,2, 0, 0.01, 0.01);  //计算出的直线line为cv::Vec4f类型。注意points为空时函数出错。
		double cos_theta = line0[0];
		double sin_theta = line0[1];
		double x0 = line0[2], y0 = line0[3];
            
		//寻找拟合直线的两个端点  
	        if(line0[0]<0.05){          //边缘是竖直线
			begin=getEndPoint(points_,1,0);
			end=getEndPoint(points_,1,1);
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
	}
	if(getLine==1){
		//line(ROI, begin,end, Scalar(0,255,0),2);
	}
	else{
		cout<<"未检测到边缘!"<<endl;
	}	
	ROI.copyTo(image,mask);    //将处理完成的ROI附给原图,(注意超出范围)
}
Mat getROI(Mat src,struct scanArea area){              //获取ROI区域
	
	Mat dst;
	mask= Mat::zeros(src.size(), CV_8UC1);         //通过绘制四边形和满水填充得到掩膜
	line(mask, area.p1, area.p2, Scalar(255));
	line(mask, area.p3, area.p4, Scalar(255));
	line(mask, area.p1, area.p3, Scalar(255));
	line(mask, area.p2, area.p4, Scalar(255));
	Point seed=Point((int)(area.p1.x+area.p4.x)/2,(int)(area.p1.y+area.p4.y)/2);
	floodFill(mask, seed, 255, NULL, cvScalarAll(0), cvScalarAll(0), CV_FLOODFILL_FIXED_RANGE);
	src.copyTo(dst, mask);                        //根据掩膜得到ROI
	return dst;
}
void drawScanLine(Mat &src,struct scanArea area){
	line(src, area.p1, area.p2, Scalar(0,255,0),2); //绘制扫描线
	line(src, area.p3, area.p4, Scalar(0,255,0),2);
	LineIterator iterator1(src, area.p1, area.p2, 4, false);
	LineIterator iterator2(src, area.p3, area.p4, 4, false);
	int f=0;
	while(f<iterator1.count){
		LineIterator iterator(src, iterator1.pos(),iterator2.pos(), 4, false);
		for(int j=0;j<iterator.count;j++){
			if(j%10==0)
				circle(src,iterator.pos(),1,Scalar(0,255,0),-1);	
			iterator++;
		}
		for(int j=0;j<step*3;j++){
			iterator1++;iterator2++;f++;
			if(f==iterator1.count)break;
		}		
	}
}
int  Rotate_rect(Point center, float center_angle, Point tempDatum_point,Point testDatum_point,Rect detectRect)
{

        if(center_angle==0)
           return 1;
	cv::Point2f h11, h12, h13, h14;
	//旋转前坐标，依次为左上、左下、右上、右下
	h11.x = detectRect.x;
	h11.y = detectRect.y;
	h12.x = detectRect.x;
	h12.y = detectRect.y + detectRect.height;
	h13.x = detectRect.x + detectRect.width;
	h13.y = detectRect.y;
	h14.x = detectRect.x + detectRect.width;
	h14.y = detectRect.y + detectRect.height;

	//旋转后点坐标
	cv::Point h1, h2, h3, h4;
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

	area.p1=h1;
	area.p2=h3;
	area.p3=h2;
	area.p4=h4;
	return 1;
}
int main()
{
        image= imread("/home/zql/图片/testImage.bmp");
	area={Point(900,0),Point(1500,0),Point(900,400),Point(1500,400)};//初始化ROI的四个端点
	//edgeDetect(src,area,selectPointCount,blurWidth,blurLength,edgeThreshold, selectionMethod,colorChange,edgeDepth,minLength,maxLength,angleDiffer);
	ROI=getROI(image,area);   //获取ROI区域
	edgeDetect(ROI,area,30, 3,2,100, 1,0,20,200,600,CV_PI/2);  //边检测和绘制(段错误)
	drawScanLine(image,area);
	//Rotate_rect(cv::Point center, float center_angle, cv::Point tempDatum_point,cv::Point testDatum_point,cv::Rect &detectRect);
	//Rotate_rect(Point(900,200), 10, Point(900,400),Point(1200,600),Rect(900,0,600,400));
	//drawScanLine(image,area);
	namedWindow("image",1);//创建窗口
	//cvResizeWindow("image", 1000,600); //创建一个1000*600大小的窗口
	imshow("image", image);
	waitKey(0);
	return 0;
}
