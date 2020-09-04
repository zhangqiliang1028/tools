#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
using namespace cv;
using namespace std;
//【颜色变化】沿着ROI设定的扫描方向进行扫描，寻找从【黑到白】或者【白到黑】的边缘点，二选一。
//【选点方式】沿着ROI设定的扫描方向进行扫描，寻找【峰值点】或者【最先找到点】作为边缘点。【峰值点】指扫描方向上边缘最强且超过【检出阈值】的点，【最先找到点】指扫描方向上最先找到的边缘强度超过【检出阈值】的点。
//【选点数量】沿着ROI设定的扫描方向等间隔设置扫描线进行扫描，扫描线的数量。数量为0时表示自动选择，一般为逐像素设置扫描线；数量大于0时表示实际的扫描线数量。
//【边缘阈值】，图像上检出边缘点时的阈值，参数范围为0到255。默认值20。【检出阈值】也可以通过【直方图显示窗口】中调整蓝色竖直线来调节。
//【线宽度】为了抑制噪声，在扫描方向上扫面边缘点时，可以选择一定的线宽，综合扫描方向上多个像素的信息寻找边缘，边缘位置会更稳定一点。参数范围为非负整数，默认值为3。
//【滤波宽度】差分滤波器的半长度。
//【深度阈值】此参数仅用户选择寻找第一个满足上述要求的边缘点时有效。大于设定阈值的边缘点与下一个大于同样阈值的反色边缘点的距离定义为此边缘点的深度。深度小于深度阈值的边缘点被认为是噪声，此时算法将跳过继续寻找满足要求的边缘点。
//【角度阈值】找到的圆弧的角度必须大于此阈值，单位为角度；
/*
Mat  src                      ：输入图像 
int selectPointCount    :  选点数量
int blurWidth              ：滤波宽度
int edgeThreshold      ： 边缘阈值
int selectionMethod   ： 选点方式，0为峰值点，1为最先找到点
int colorChange         ： 颜色变换，0为黑到白，1为白到黑
int blurLength            ： 差分滤波器长度
int depthThreshold     :   深度阈值
float step               :   点距
*/
typedef struct scanArea{
	Point center;//检测区域圆心坐标	
	int exRadius;//外半径长度
	int inRadius;//内半径长度
	float angle1;
	float angle2;
}scanArea;
typedef struct node{
	Point point;//位置
	float value=0;//求平均值后的结果
	float result=0;//滤波后的结果
}node;
//Mat image = imread("/home/zql/图片/8.png");
Mat image = imread("testImage.bmp");
void workPoints(Mat src,scanArea area, int selectPointCount, int blurWidth,vector<vector<Point>>&results)//选出所有起作用的点
{
	if(area.angle1>area.angle2){
		area.angle2+=360;
	}
	float step;
	float arc_angle1=CV_PI/180*area.angle1;            //转换成弧度
	float arc_angle2=CV_PI/180*area.angle2;
	float angleScale=abs(arc_angle1-arc_angle2);
	if(selectPointCount!=0){
		step=angleScale/(selectPointCount+1);
	}
	else{
		step=CV_PI/180;
	}

	float aa=CV_PI/180;                               //此处画ROI区域
	vector<Point> v1,v2;                   
	for(float an=arc_angle1;an<=arc_angle2;an+=aa){
		int x0=cos(an)*area.exRadius+area.center.x;
		int y0=sin(an)*area.exRadius+area.center.y;
		int x1=cos(an)*area.inRadius+area.center.x;
		int y1=sin(an)*area.inRadius+area.center.y;
		v1.push_back(Point(x0,y0));
		v2.push_back(Point(x1,y1));
	}
	for(int i=0;i<v1.size()-1;i++){
		line(image,v1[i],v1[i+1],Scalar(0,0,255),1);
		line(image,v2[i],v2[i+1],Scalar(0,0,255),1);
	}

	float angle=arc_angle1;
	vector<Point> vex,vin;
	int x0,y0,x1,y1;
	for(int i=0;i<selectPointCount;i++){                   //获取所有扫描线的起点和终点(包含最后一条)
		x0=cos(angle)*area.exRadius+area.center.x;
		y0=sin(angle)*area.exRadius+area.center.y;
		x1=cos(angle)*area.inRadius+area.center.x;
		y1=sin(angle)*area.inRadius+area.center.y;
		vex.push_back(Point(x0,y0));
		vin.push_back(Point(x1,y1));
		angle+=step;
		
	}
	float tepm_angle=arc_angle2;
	x0=cos(tepm_angle)*area.exRadius+area.center.x;
	y0=sin(tepm_angle)*area.exRadius+area.center.y;
	x1=cos(tepm_angle)*area.inRadius+area.center.x;
	y1=sin(tepm_angle)*area.inRadius+area.center.y;
	vex.push_back(Point(x0,y0));
	vin.push_back(Point(x1,y1));
	line(image,vin[0],vex[0],Scalar(0,0,255),1);
	line(image,vin[vin.size()-1],vex[vex.size()-1],Scalar(0,0,255),1);
	int connectivity = 4;
	int count;
	for(int i=0;i<vin.size();i++){                         //根据扫描线的起点和终点找到所有扫描线所在坐标
		LineIterator iterator(src, vex[i], vin[i], connectivity, false);
		count=iterator.count;
		vector<Point> point;
		for(int j=0;j<count;j++){
			point.push_back(iterator.pos());
			if(j%8==0&&i%3==0){
				circle(image,iterator.pos(),1,Scalar(0,0,255),-1);
			}
			iterator++;
		}
		results.push_back(point);
	}
}

bool fittingCircle(std::vector<cv::Point>& pts, cv::Point2f& center, double& radius)
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

int pointsDetection(Mat src,scanArea area,vector<vector<Point>> scanPoints, int edgeThreshold, int selectionMethod, int colorChange, int blurLenth,int blurWidth,int depthThreshold,vector<Point> &points )
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

int arcDetect(Mat src,scanArea area, int selectPointCount, int blurWidth,int blurLength,int edgeThreshold, int selectionMethod,int colorChange,int depthThreshold,int angleThreshold ){
	if(src.channels()>1){
		cvtColor(src,src, COLOR_BGR2GRAY);
	}
	float score=0;
	vector<vector<Point>> results;
	vector<Point> points;
	workPoints(src,area, selectPointCount, blurWidth,results);
	pointsDetection( src,area,results,edgeThreshold,selectionMethod,colorChange,blurLength,blurWidth,depthThreshold,points );
	
	if(points.size()<3){
		cout<<"未检测到圆弧！"<<endl;
		return 0;
	}
	Point2f center_;
	double radius_;
	fittingCircle(points,center_,radius_);              //最小二乘法拟合圆，求出圆心和半径

	Point2f start=points[0];
	Point2f end=points[points.size()-1];
	float an1=atan((start.y-center_.y)/(start.x-center_.x));   //弧度转换，画圆弧
	float an2=atan((end.y-center_.y)/(end.x-center_.x));
	if(start.x<center_.x)
		an1=CV_PI+an1;
	if(end.x<center_.x)
		an2=CV_PI+an2;
	if(an2<0)
		an2=CV_PI*2+an2;
	if(an1<0)
		an1=CV_PI*2+an1;
	if(an1>an2)
		an2+=2*CV_PI;
	
	float aa=CV_PI/180;
	vector<Point> v;
	for(float i=an1;i<=an2;i+=aa){
		int x0=center_.x+radius_*cos(i);
		int y0=center_.y+radius_*sin(i);
		v.push_back(Point(x0,y0));
	}
	for(int i=0;i<v.size()-1;i++){
		line(image,v[i],v[i+1],Scalar(0,255,0),2);
	}

	return 1;

}
int main()
{
	double time0=(double)(getTickCount());
	Mat src;
	image.copyTo(src);
	scanArea area;
	area={Point(1410,690),int(120),int(40),float(210),float(270)};
	//GaussianBlur(src,src,Size(3,3),0,0);
	//Mat element = getStructuringElement(MORPH_RECT,Size(3,3),Point(-1,-1));
	//dilate(src,src,element,Point(-1,-1),1);
	//erode(src,src,element,Point(-1,-1),1);
	//circleDetect(src,area, int selectPointCount, blurWidth,blurLength,edgeThreshold, selectionMethod,colorChange,int depthThreshold,float angleThreshold)
	arcDetect(src,area,20,3,2,20, 0,1,0,90);
	double time1=(double)(getTickCount());
	double sumTime=(time1-time0)/getTickFrequency();
	cout<<"time(ms): "<<sumTime*1000<<endl;
	resize(image,image,Size(image.cols/2,image.rows/2));
	imshow("image", image);
	waitKey(0);
	return 0;
}
