#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
using namespace cv;
using namespace std;
Mat srcImage=imread("4.jpg",0);
//Mat tempImage=srcImage(Rect(srcImage.cols/3,srcImage.rows/3,100,100));
typedef struct scanArea{
	Point2f p1;
	Point2f p2;
	Point2f p3;
	Point2f p4;
}scanArea;
int tempMatch(Mat src0,Mat temp0);  //模板匹配
void rotate_rect(Mat src,float center_angle);//旋转
void findRect(Mat src);          //搜索矩形
void findCircle(Mat src);        //搜索圆形
int edgeLocation(Mat src);       //直线检测
bool isInRect(scanArea area,Point2f p); //判断点是否在矩形内
int main(){
	Mat src;
	srcImage.copyTo(src);

	
	/*判断点是否在矩形内
	scanArea area={Point2f(50,50),Point2f(50,100),Point2f(100,50),Point2f(110,90),};
	vector<Point2f> points;
	points.push_back(area.p1);
	points.push_back(area.p2);
	points.push_back(area.p3);
	points.push_back(area.p4);
	Rect rect=boundingRect(Mat(points));
	for(int i=rect.tl().x;i<=rect.br().x;i++){
		for(int j=rect.tl().y;j<=rect.br().y;j++){
			int elem=(int)src.at<uchar>(j,i);
			if(elem>0.5&&!isInRect(area,Point2f(i,j))){
				srcImage.at<uchar>(j,i)=0;
			}
		}
	}
	*/
	imshow("srcImage",srcImage);
	waitKey(0);
	
	return 0;

}

bool isInRect(scanArea area,Point2f p){	
	
	line(srcImage,area.p1,area.p2,Scalar(0,255,0),1);
	line(srcImage,area.p1,area.p3,Scalar(0,255,0),1);
	line(srcImage,area.p4,area.p2,Scalar(0,255,0),1);
	line(srcImage,area.p4,area.p3,Scalar(0,255,0),1);
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

void rotate_rect(Mat src,float center_angle){

	Mat image;
	src.copyTo(image);
	if(image.channels()>1){
		//cvtColor(image,image,COLOR_BGR2GRAY);
		cout<<image.channels()<<endl;
	}
	Point2f center=Point2f(src.rows/5*2,src.cols/5*2);
	Point2f h0;
	float angle = center_angle * CV_PI / 180; // 弧度
	for(int i=src.cols/5*2;i<src.cols/5*3;i++){
		for(int j=src.rows/5*2;j<src.rows/5*3;j++){
			
			h0.x = (i - center.x)*cos(angle) - (j - center.y)*sin(angle) + center.x;
			h0.y = (i - center.x)*sin(angle) + (j - center.y)*cos(angle) + center.y;
			for(int k=0;k<3;k++){
				src.at<Vec3b>(h0.x,h0.y)[k] = src.at<Vec3b>(i,j)[k];
				src.at<Vec3b>(Point(i,j))[k] = 0;
			}
		}
	}
}

int tempMatch(Mat src0,Mat temp0){
	
	Mat src,temp;
	src0.copyTo(src);
	temp0.copyTo(temp);
	GaussianBlur(src,src,Size(3,3),0,0);
	GaussianBlur(temp,temp,Size(3,3),0,0);
	Canny(src,src,40,80,3,false);
	Canny(temp,temp,40,80,3,false);
	
	int tempCount = countNonZero(temp);
	for(int j=0;j<src.rows-temp.rows;j++){
		for(int i=0;i<src.cols-temp.cols;i++){
			Rect rect(i,j,temp.cols,temp.rows);
			int srcCount = countNonZero(src(rect));
			if(abs(tempCount-srcCount)==0){
				int cou=0;
				for(int m=0;m<temp.cols;m++){
					for(int n=0;n<temp.rows;n++){
						if((int)temp.at<uchar>(m,n)==(int)src.at<uchar>(m+i,n+j)){
							cou++;
						}
					}
				}
				if(cou>temp.cols*temp.rows-10){
					line(srcImage,Point(i,j),Point(i,j+temp.rows),Scalar(0,0,255),2);
					line(srcImage,Point(i+temp.cols,j),Point(i+temp.cols,j+temp.rows),Scalar(0,0,255),2);
					line(srcImage,Point(i,j),Point(i+temp.cols,j),Scalar(0,0,255),2);
					line(srcImage,Point(i,j+temp.rows),Point(i+temp.cols,j+temp.rows),Scalar(0,0,255),2);
					return 1;
				}
			}
		}
	}
	return 0;
}

void findRect(Mat src){
	Mat image;
	Mat im=Mat::zeros(src.rows,src.cols,CV_8UC3);
	src.copyTo(image);
	if(image.channels()>1){
		cvtColor(image,image,COLOR_BGR2GRAY);	
	}
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	blur(image,image,Size(3,3));
	Canny(image,image,30,100,3);
	findContours(image,contours,hierarchy,RETR_CCOMP, CHAIN_APPROX_SIMPLE); //寻找轮廓
	vector<Rect> boundRect(contours.size());  //定义外接矩形集合
        vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合
        Point2f rect[4];
        for(int i=0; i<contours.size(); i++)
        {
           box[i] = minAreaRect(Mat(contours[i]));  //计算每个轮廓最小外接矩形
           //boundRect[i] = boundingRect(Mat(contours[i]));
           box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
           //rectangle(im, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x + boundRect[i].width, boundRect[i].y + boundRect[i].height), Scalar(0, 255, 0), 2, 8);
	   double area1 = contourArea(contours[i], true);
	   double area2 = sqrt(((rect[2].y-rect[1].y)*(rect[2].y-rect[1].y)+(rect[2].x-rect[1].x)*(rect[2].x-rect[1].x))*((rect[3].y-rect[2].y)*(rect[3].y-rect[2].y)+(rect[3].x-rect[2].x)*(rect[3].x-rect[2].x)));
	   if(fabs(area2-area1)<area2/20&&area2>20){
          	 for(int j=0; j<4; j++)
          	 {
          	     line(src, rect[j], rect[(j+1)%4], Scalar(0, 255, 0), 2, 4);  //绘制最小外接矩形每条边
         	  }
	   }
	   for (int i = 0; i < contours.size(); i++)
	   {
		drawContours(im, contours, i, Scalar(0,255,0), 1, 8, hierarchy, 0);//绘制轮廓
           }
	}

}

void findCircle(Mat src){
	Mat image;
	Mat im=Mat::zeros(src.rows,src.cols,CV_8UC3);
	src.copyTo(image);
	if(image.channels()>1){
		cvtColor(image,image,COLOR_BGR2GRAY);	
	}
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	blur(image,image,Size(3,3));
	Canny(image,image,30,100,3);
	findContours(image,contours,hierarchy,RETR_CCOMP, CHAIN_APPROX_SIMPLE); //寻找轮廓
	vector<Point2f> center(contours.size());  //定义外接圆圆心
        vector<float> radius(contours.size()); //定义外接圆半径
        for(int i=0; i<contours.size(); i++)
        {
           minEnclosingCircle(contours[i],center[i],radius[i]);
	   double area1 = contourArea(contours[i], true);
	   double area2 = CV_PI*radius[i]*radius[i];
	   if(fabs(area2-area1)<area2/20&&area2>10){
          	 circle(src,center[i],radius[i],Scalar(0,255,0),
1);
	   } 
	}
}

int edgeLocation(Mat src){
	double start=getTickCount();
	src=imread("4.jpg");
	Mat image;
	cvtColor(src,image,COLOR_BGR2GRAY);
	blur(image,image,Size(3,3));
	Canny(image,image,50,120,3);
	imshow("image",image);
	vector<Vec4i> lines;
	HoughLinesP(image,lines,1,CV_PI/180,30,30,10);
	for(int i=0;i<lines.size();i++){
		line(src,Point(lines[i][0],lines[i][1]),Point(lines[i][2],lines[i][3]),Scalar(0,255,0),1);
	}
	double end=getTickCount();
	float time=(end-start)/getTickFrequency()*1000;
	cout<<time<<endl;
	imshow("src",src);
	waitKey();
	return 0;
/*//HoughLinesP(image,lines,1,CV_PI/180,100,10,5);
//InputArray image：输入图像，必须是8位单通道图像。 
　　//OutputArray lines：检测到的线条参数集合。 
　　//double rho：直线搜索时的距离步长，以像素为单位。 
　　//double theta：直线搜索时的角度步长，以弧度为单位。 
　　//int threshold：累加计数值的阈值参数，当参数空间某个交点的累加计数的值超过该阈值，则认为该交点对应了图像空间的一条直线。 
　　//double minLineLength：默认值为0，表示最小线段长度阈值（像素）。 
　　//double maxLineGap：线段上最近两点之间的阈值*/
}
