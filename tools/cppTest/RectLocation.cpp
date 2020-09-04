#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
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
int maxDiagonal         :最大对角线长
int minDiagonal         :最小对角线长
int edgeThreshold       ：边缘阈值
int maxLenWidRatio      :最大长宽比值
int minLenWidRatio      :最小长宽比值
int angleDiffer		:角差阈值
int selectType          :挑选类型，1为面积最大，0为面积最小
*/
typedef struct scanArea{
	Point2f p1;
	Point2f p2;
	Point2f p3;
	Point2f p4;
}scanArea;

Mat image = imread("4.jpg");
bool isInside(RotatedRect box,Point2f p);
bool isInRect(scanArea area,Point2f p);

bool isInside(RotatedRect box,Point2f p){
	Point2f rect[4];
	box.points(rect);
	scanArea area={rect[0],rect[1],rect[3],rect[2]};
	
	if(isInRect(area,p)){
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

int rectDetect(Mat src,scanArea area, int colorChange,int minDiagonal,int maxDiagonal,int edgeThreshold,int minLenWidRatio,int maxLenWidRatio,int angleDiffer,int selectType){
	Mat canny_src;
	if(src.channels()>1){
		cvtColor(src,src, COLOR_BGR2GRAY);
	}

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	blur(src,canny_src,Size(3,3));
	Canny(canny_src,canny_src,edgeThreshold,2*edgeThreshold,3);

	for(int i=0;i<canny_src.cols;i++){                         //将非ROI区域的边缘去掉
		for(int j=0;j<canny_src.rows;j++){
			int elem=(int)canny_src.at<uchar>(j,i);
			if(elem>0.5&&!isInRect(area,Point2f(i,j))){
				canny_src.at<uchar>(j,i)=0;
			}
		}
	}
	imshow("canny_src",canny_src);
	findContours(canny_src,contours,hierarchy,RETR_CCOMP, CHAIN_APPROX_SIMPLE); //寻找轮廓
	if(contours.size()==0){
		cout<<"no rectangle!"<<endl;
		return 0;
	}
	
	vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合
	vector<RotatedRect> rectangles;
	vector<Rect> boundRects;
        Point2f rect[4];
        for(int i=0; i<contours.size(); i++)
        {
           box[i] = minAreaRect(Mat(contours[i]));  //计算每个轮廓最小外接矩形
           box[i].points(rect);
           float a=box[i].size.width;                         
	   float b=box[i].size.height;
	   float x0=box[i].center.x;
	   float y0=box[i].center.y;
	   float area1 = contourArea(contours[i], true);
	   float area2 = a*b;
	   float diagonal=sqrt(a*a+b*b);
	   float LenWidRatio =a/b;
	   if(fabs(area1-area2)<=area2/20&&area1>20&&diagonal>=minDiagonal&&diagonal<=maxDiagonal&&contours[i].size()>10&&LenWidRatio<=maxLenWidRatio&&LenWidRatio>=minLenWidRatio){
		rectangles.push_back(box[i]);         //将符合条件的最小外接矩形放入rectangles
		boundRects.push_back(boundingRect(Mat(contours[i])));//将符合条件的外接矩形放入boundRects
	   }
	}
	if(rectangles.size()==0){
		cout<<"no rectangle!"<<endl;
		return 0;
	}
	vector<int> color_;
	vector<float> area_;
	
	for(int i=0; i<rectangles.size(); i++){
		
		float a=rectangles[i].size.width;    //注意区分长和宽!!!                 
		float b=rectangles[i].size.height;
		float x0=rectangles[i].center.x;
		float y0=rectangles[i].center.y;
		float angle=rectangles[i].angle;
		area_.push_back(a*b);	  //面积

		RotatedRect extendBox = rectangles[i];                 //颜色判断
		extendBox.size.width+=10;
		extendBox.size.height+=10; 
		Rect boundRect = boundRects[i];
		float exColor=0,inColor=0;
		float exnum=0,innum=0;	
		for(int j=boundRect.tl().x-5;j<=boundRect.br().x+5;j++){
			for(int k=boundRect.tl().y-5;k<=boundRect.br().y+5;k++){
				if(j<0||j>src.cols||k<0||k>src.rows)
					continue;
				if(isInside(rectangles[i],Point2f(j,k))){
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
	
	/*for(int i=0;i<rectangles.size();i++){
		cout<<color_[i]<<"  "<<area_[i]<<endl;
	}*/
	int sign=-1;
	if(selectType==1){            //面积最大
		float maxArea=0;
		for(int i=0;i<rectangles.size();i++){
			if((colorChange==0||colorChange==color_[i])&&area_[i]>maxArea){
				sign=i;
				maxArea=area_[i];
			}
		}	
	}
	else if(selectType==0){       //面积最小
		float minArea=500000;
		for(int i=0;i<rectangles.size();i++){
			if((colorChange==0||colorChange==color_[i])&&area_[i]<minArea){
				sign=i;
				minArea=area_[i];
			}
		}	
	}
	if(sign==-1){
		cout<<"no rectangle!!!"<<endl;	
		return 0;
	}
	Point2f rect_[4];
	rectangles[sign].points(rect_);
	for(int j=0; j<4; j++)
        {
        	line(image, rect_[j], rect_[(j+1)%4], Scalar(0, 255, 0), 2, 4);  //绘制最小外接矩形每条边
        }
	circle(image,rectangles[sign].center,1,Scalar(0,255,0),-1);
	cout<<"color    : "<<color_[sign]<<endl;
	cout<<"area     : "<<area_[sign]<<endl;
	cout<<"position : "<<rectangles[sign].center<<endl;
	cout<<"angle    : "<<rectangles[sign].angle<<endl;
	return 1;
}
int main()
{

	Mat src;
	image.copyTo(src);
	scanArea area;
	
	area={Point2f(10,10),Point2f(10,300),Point2f(300,10),Point2f(300,300)};
	line(image,area.p1,area.p2,Scalar(0,255,0),1);
	line(image,area.p1,area.p3,Scalar(0,255,0),1);
	line(image,area.p4,area.p2,Scalar(0,255,0),1);
	line(image,area.p4,area.p3,Scalar(0,255,0),1);
	//rectDetect(Mat src,scanArea area, int colorChange,int minDiagonal,int maxDiagonal,int edgeThreshold,int minLenWidRatio,int maxLenWidRatio,int angleDiffer,int selectType)
	rectDetect(src,area,-1,30,600,20, 0.2,5,10,1);
	imshow("image", image);
	waitKey(0);
	return 0;
}
