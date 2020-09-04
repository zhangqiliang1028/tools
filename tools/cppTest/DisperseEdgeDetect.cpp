#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<math.h>
using namespace cv;
using namespace std;
/*
【边缘数量】序列化ROI中子ROI的数量就是检出边缘点的数量，即在每个子ROI中检出一个边缘点；
【区域尺寸】表示序列化ROI中每个子ROI的长度和宽度；
【颜色变化】边缘点颜色变化参数。边缘的扫描方向由序列化ROI确定，用户可以在图像上调整。子ROI中显示了每个子矩形ROI中边缘扫描方向；
【选点方式】扫描线上选择满足颜色变化需求并且强度大于阈值的峰值点或者选择第一个点；
【边缘阈值】，图像上检出边缘点时的阈值，参数范围为0到255。默认值20。【检出阈值】也可以通过【直方图显示窗口】中调整蓝色竖直线来调节。
【点数阈值】找到的边缘点数量少于此阈值时直接报错。
【扫描线宽度】边缘扫描线的宽度，等于子矩形ROI的宽度；
【滤波宽度】差分滤波器的半长度。
【深度阈值】此参数仅用户选择寻找第一个满足上述要求的边缘点时有效。大于设定阈值的边缘点与下一个大于同样阈值的反色边缘点的距离定义为此边缘点的深度。深度小于深度阈值的边缘点被认为是噪声，此时算法将跳过继续寻找满足要求的边缘点。
【长度范围】找到的边缘点拟合得到直线段之后，此直线段长度范围。最大值为0时表示不限制。
【角度差异】找到的边缘点拟合得到直线段之后，此直线段与【边定位】ROI的角度差异最大值，参数范围为0-180度，默认值为10度。*/
/*
Mat  src                      ：输入图像 
int selectPointCount    :  选点数量
int blurWidth              ：扫描线宽度
int roiHeight                :子ROI高度
int edgeThreshold      ： 边缘阈值
int selectionMethod   ： 选点方式，0为峰值点，1为最先找到点
int colorChange         ： 颜色变换，0为黑到白，1为白到黑
int blurLength            ： 差分滤波器长度
int depthThreshold     :   深度阈值
int pointnumThreshold:  点数阈值
int minlength         :    最小长度
int maxlength         :    最大长度
int angleDiffer            :   角度差异
scanArea  detectArea   :   扫描区域
scanArea  templateArea  :   模板区域
float step                        
*/
Mat image;
typedef struct Line{
	Point2f p1;
	Point2f p2;
}Line;
typedef struct scanArea{
	Point2f p1;
	Point2f p2;
}scanArea;
typedef struct NNode{
	Point2f point;//位置
	float value;//求平均值后的结果
	float result=0;//滤波后的结果
}NNode;
void workPoints(Mat src,scanArea detectArea, int selectPointCount, int blurWidth,int roiHeight,vector<vector<Point2f>>& results)//选出所有起作用的点
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
		/*cv::line(image,p1,p2,Scalar(0,255,0),1);
		cv::line(image,p1,p3,Scalar(0,255,0),1);
		cv::line(image,p2,p4,Scalar(0,255,0),1);
		cv::line(image,p3,p4,Scalar(0,255,0),1);*/
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

int pointsDetection(Mat src,scanArea area,vector<vector<Point2f>> scanPoints, int edgeThreshold, int selectionMethod, int colorChange, int blurLenth,int blurWidth,int depthThreshold,vector<Point2f> &points )
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
	for(int i=0;i<nodes.size();i++){
		for(int j=0;j<nodes[i].size();j++){
			cout<<nodes[i][j].result<<" ";
		}
		cout<<endl;
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
int  edgeDetect(Mat src,scanArea area, int selectPointCount, int blurWidth,int blurLength,int roiHeight,int edgeThreshold,int depthThreshold,int pointnumThreshold,int selectionMethod,int colorChange,int minLength,int maxLength,int angleDiffer,Line &line){
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
	cv::line(image,begin,end,Scalar(0,255,0),1);
	return 1;
}
int main()
{
	Mat src = imread("template.bmp");
	
	src.copyTo(image);
	scanArea area;
	area={Point(1100,801),Point(1500,801)};
	//edgeDetect(src,area, int selectPointCount, blurWidth,blurLength,roiHeight,edgeThreshold,depthThreshold,pointnumThreshold,selectionMethod,colorChang,minlength,maxlength,angleDiffer,line)；
	Line line;
	edgeDetect(src,area,7,5,2,25,20,0, 2,1,1,20,0,10,line);
	imshow("image", image(Rect(700,700,1000,700)));
	waitKey(0);
	return 0;
}
       
