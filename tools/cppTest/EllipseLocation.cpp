#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<time.h>
using namespace cv;
using namespace std;
/*
【颜色变化】设置椭圆的颜色；
	【白色椭圆】椭圆内侧相对外侧是白色；
	【黑色椭圆】椭圆内侧相对外侧是黑色；
	【两者皆可】以上两者均可椭圆是基于边缘轮廓分析得到的，以下两个参数设置边缘检出参数
【边缘尺度】边缘尺寸设置边缘检出的尺度。当图像噪声大或者边缘模糊的时候，需要增大边缘尺度。
【边缘阈值】边缘检出阈值
【半径范围】设置椭圆半长轴和半短轴的范围，用来缩小椭圆搜索的范围；
【圆度范围】圆度定义为椭圆短轴长度与长轴长度之比，用来缩小椭圆搜索的范围。找到椭圆后需要对结果进行过滤，
【距离阈值】当边缘点和椭圆的距离小于此参数时，边缘点在椭圆上。
【检出阈值】椭圆上的边缘点覆盖椭圆一圈的比例就是椭圆的得分100分制。大于此阈值的才被检出。可以用来过滤椭圆弧。
【挑选类型】用来挑选得到唯一的一个椭圆来定位。

/*
Mat  src                ：输入图像
scanArea scanArea       :扫描区域 
int colorChange         ：颜色变换，0为两者皆可，1为白到黑，-1为黑到白
int edgeDimension       : 边缘尺度
int edgeThreshold       ：边缘阈值
int maxRadius           :最大半径
int minRadius           :最小半径
int maxRoundness        :最大圆度
int minRoundness        :最小圆度
int distanceThreshold   :距离阈值
int score               :找圆得分(检出阈值)
int selectType          :挑选类型，0为最小圆，1为最大圆
*/
typedef struct scanArea{
	Point2f p1;
	Point2f p2;
	Point2f p3;
	Point2f p4;
}scanArea;

Mat image = imread("9.jpg");
void workPoints(Mat src,scanArea area,Mat &mat);
bool isInside(RotatedRect box,Point2f point){
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

bool isInRect(scanArea area,Point2f p){
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

int ellipseDetect(Mat src,scanArea area, int colorChange,int edgeDimension,int edgeThreshold,int maxRadius,int minRadius,int maxRoundness,int minRoundness,int distanceThreshold,int score,int selectType){
	Mat canny_src;
	if(src.channels()>1){
		cvtColor(src,src, COLOR_BGR2GRAY);
	}
	Mat mat=Mat::zeros(src.rows,src.cols,CV_8UC1);
	vector<vector<Point>> resultsCon;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	blur(src,canny_src,Size(3,3));
	Canny(canny_src,canny_src,edgeThreshold,2*edgeThreshold,3);

	clock_t aa,bb;
        aa=clock();
	workPoints(canny_src,area,mat);   //将非ROI区域置零,返回mat,0.025s
	
	imshow("mat",mat);waitKey();
	findContours(mat,contours,hierarchy,RETR_CCOMP, CHAIN_APPROX_SIMPLE); //寻找轮廓
	/*
	for(int i=0;i<canny_src.cols;i++){                         //将非ROI区域的边缘去掉.0.018s
		for(int j=0;j<canny_src.rows;j++){
			int elem=(int)canny_src.at<uchar>(j,i);
			if(elem>0.5&&!isInRect(area,Point2f(i,j))){
				canny_src.at<uchar>(j,i)=0;
			}
		}
	}
	imshow("canny_src",canny_src);
	findContours(canny_src,contours,hierarchy,RETR_CCOMP, CHAIN_APPROX_SIMPLE); //寻找轮廓
	*/
	bb=clock();

	cout<<"+++times :"<<(float)(bb-aa)/CLOCKS_PER_SEC<<endl;
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
	ellipse(image,ellipses[sign],Scalar(0,255,0),2,8);
	line(image,ellipses[sign].center,Point2f(ellipses[sign].center.x+10*cos(angle),ellipses[sign].center.y+10*sin(angle)),Scalar(0,255,0),1,8);
	line(image,ellipses[sign].center,Point2f(ellipses[sign].center.x+10*cos(angle+CV_PI/2),ellipses[sign].center.y+10*sin(angle+CV_PI/2)),Scalar(0,255,0),1,8);
	cout<<"position :"<<ellipses[sign].center<<endl;
	cout<<"angle :"<<ellipses[sign].angle<<endl;
	cout<<"color :"<<color_[sign]<<endl;
	cout<<"area  :"<<area_[sign]<<endl;
	cout<<"score :"<<score_[sign]<<endl;
	return 1;
}

void workPoints(Mat src,scanArea area,Mat &mat){  //根据检测区域将非检测区域置零
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
int main()
{

	Mat src;
	image.copyTo(src);
	scanArea area;
	clock_t first, second;
	area={Point2f(10,10),Point2f(10,180),Point2f(250,10),Point2f(250,180)};
	line(image,area.p1,area.p2,Scalar(0,255,0),1);
	line(image,area.p1,area.p3,Scalar(0,255,0),1);
	line(image,area.p4,area.p2,Scalar(0,255,0),1);
	line(image,area.p4,area.p3,Scalar(0,255,0),1);
	//ellipseDetect(Mat src,scanArea area, int colorChange,int edgeDimension,int edgeThreshold,int maxRadius,int minRadius,int maxRoundness,int minRoundness,int distanceThreshold,int score,int selectType)
	first=clock();
	ellipseDetect(src,area,-1,3,30,200, 20,5,0.2,2,60,1);
	second=clock();
	imshow("image", image);
	cout<<"time :"<<(float)(second-first)/CLOCKS_PER_SEC<<"s"<<endl;
	waitKey(0);
	return 0;
}
