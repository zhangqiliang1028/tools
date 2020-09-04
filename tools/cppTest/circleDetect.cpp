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
//【找圆得分】根据边缘点落在圆上点的比例可计算得到圆的得分，用来赛选某些虚假的圆；
//【使用忽略掩膜过滤边缘点】选中后可使用忽略掩膜过滤部分边缘点；
/*
Mat  src                      ：输入图像
scanArea scanArea       :扫描区域 
int selectPointCount    :  选点数量
int blurWidth              ：滤波宽度
int edgeThreshold      ： 边缘阈值
int selectionMethod   ： 选点方式，0为峰值点，1为最先找到点
int colorChange         ： 颜色变换，0为黑到白，1为白到黑
int blurLength            ： 差分滤波器长度
int depthThreshold     :   深度阈值
int score                     :   找圆得分
float step               :   点距
*/
typedef struct scanArea{
	Point center;//检测区域圆心坐标	
	int exRadius;//外半径长度
	int inRadius;//内半径长度
}scanArea;
typedef struct node{
	Point point;//位置
	float value=0;//求平均值后的结果
	float result=0;//滤波后的结果
}node;
Mat image = imread("7.png");
bool fittingCircle(std::vector<cv::Point>& pts, cv::Point2f& center, float& radius);
void workPoints(Mat src,scanArea area, int selectPointCount, int blurWidth,vector<vector<Point>>&results)//选出所有起作用的点
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
				circle(image,iterator.pos(),1,Scalar(0,0,255),-1);
			}
			iterator++;
		}
		results.push_back(point);
	}
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

int circleDetect(Mat src,scanArea area, int selectPointCount, int blurWidth,int blurLength,int edgeThreshold, int selectionMethod,int colorChange,int depthThreshold){
	if(src.channels()>1){
		cvtColor(src,src, COLOR_BGR2GRAY);
	}
	float score=0;
	vector<vector<Point>> results;
	vector<Point> points;
	workPoints(src,area, selectPointCount, blurWidth,results);
	pointsDetection( src,area,results,edgeThreshold,selectionMethod,colorChange,blurLength,blurWidth,depthThreshold,points );

	//排除孤立点(比较偏的点)  根据所有点到平均中心的距离排除部分最远和最近的点
	/*float averX=0,averY=0,averR=0;
	for(int i=0;i<points.size();i++){                  //计算中心平均值
		averX+=points[i].x;
		averY+=points[i].y;
	}
	averX/=points.size();
	averY/=points.size();
	vector<int> dis;
	for(int j=0;j<points.size();j++){                  //计算所有点到中心的距离
		int temp=sqrt((points[j].y-averY)*(points[j].y-averY)+(points[j].x-averX)*(points[j].x-averX));
		dis.push_back(temp);
		averR+=temp;
		
	}
	averR/=points.size();

	for(int i=points.size()-1;i>=0;i--){               //所有点距离从小到大排序
		for(int j=0;j<i;j++){
			if(dis[j]>dis[j+1]){
				int t=dis[j];
				dis[j]=dis[j+1];
				dis[j+1]=t;
				Point p=points[j];
				points[j]=points[j+1];
				points[j+1]=p;
			}
		}
	}
	
	vector<Point> points_;                              //排除1/5最远点和1/5最近点
	for(int i=points.size()/10;i<=points.size()*9/10;i++){
		points_.push_back(points[i]);
	}
	*/
	Point2f center;
	float radius;
	fittingCircle(points, center, radius);

                           //得分判断
	float x0=center.x;
	float y0=center.y;
	cout<<radius<<endl;
	int hit=0;
	for(int i=0;i<points.size();i++){  
		circle(image,points[i],2,Scalar(0,255,0),-1);            
		float x_=fabs(points[i].x-x0);
		float y_=fabs(points[i].y-y0);
		cout<<sqrt(x_*x_+y_*y_)<<endl;
		if(fabs(sqrt((x_*x_)+(y_*y_))-radius)<=2){
			hit++;
		}
	}
	score=((float)hit/selectPointCount*100);
	cout<<"+++score is : "<<score<<endl;
	if(score<60){
		cout<<"未检测到圆"<<endl;
		return 0;
	}
	circle(image,area.center,area.exRadius,Scalar(0,0,255),1);
	circle(image,area.center,area.inRadius,Scalar(0,0,255),1);
	circle(image,center,radius,Scalar(0,255,0),1);
	return 1;
}
bool fittingCircle(std::vector<cv::Point>& pts, cv::Point2f& center, float& radius)
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
int main()
{

	Mat src;
	image.copyTo(src);
	scanArea area;
	//area={Point(542,182),int(70),int(20)};
	//area={Point(290,275),int(60),int(20)};
	area={Point(577,342),int(60),int(20)};
	//area={Point(610,318),int(70),int(20)};
	GaussianBlur(src,src,Size(3,3),0,0);
	Mat element = getStructuringElement(MORPH_RECT,Size(3,3),Point(-1,-1));
	dilate(src,src,element,Point(-1,-1),1);
	erode(src,src,element,Point(-1,-1),1);
	//circleDetect(src,area, int selectPointCount, blurWidth,blurLength,edgeThreshold, selectionMethod,colorChange,int depthThreshold)；
	circleDetect(src,area,30,3,2,10, 0,0,0);
	resize(image,image,Size(image.cols*2,image.rows*2));
	imshow("image", image);
	waitKey(0);
	return 0;
}
