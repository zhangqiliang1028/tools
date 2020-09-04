#include "LineDetectTool.h"
#include "ivs_algorithm_utils.h"
#include <string>
ToolBase *CreateTool(void)
{
    return (new LineDetectTool());
}

int LineDetectTool::initTool(Json::Value json)
{
        if(json["toolRelyNums"].isNull()||json["centerX"].isNull()||json["centerY"].isNull()||json["templatePointX"].isNull()||json["templatePointY"].isNull()||json["detectRect"].isNull()||json["sampleImageWidth"].isNull()||json["sampleImageHeight"].isNull())
        {
           std::cout<<"Error:LineDetectTool tool init error:NULL!"<<std::endl;
           //throw std::exception();
           return 0;
        }
        std::cout<<"LineDetect tool init start"<<std::endl;
	//��ʼ������
        unsigned int rely_size =  json["toolRelyNums"].size();
        for (unsigned int i = 0; i < rely_size; ++i)
        {
              int num=json["toolRelyNums"][i].asInt();
              relyNums.push_back(num);
        }
        //��ʼ������
	center.x = json["centerX"].asDouble();
        center.y = json["centerY"].asDouble();
        tempDatum_point.x = json["templatePointX"].asDouble();
        tempDatum_point.y = json["templatePointY"].asDouble();
        detectRect.x = json["detectRect"]["extRectX"].asInt();
        detectRect.y = json["detectRect"]["extRectY"].asInt();
	detectRect.width = json["detectRect"]["extRectWidth"].asInt();
        detectRect.height = json["detectRect"]["extRectHeight"].asInt();
        sample_width=json["sampleImageWidth"].asInt();
        sample_height=json["sampleImageHeight"].asInt();

        std::cout<<"LineDetect tool init down"<<std::endl;
        return 1;
}
int LineDetectTool::getRelyOnParam(std::vector<int> &nums) 
{
        //ͨ���������߲�����ȡ��ǰ���ߵĽ�����nums
        if(relyNums.size()==0)
          return 0;
        nums=relyNums;
        return 1;
}
int LineDetectTool::run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result)
{
        TimeTracker time;
        time.start();
        int lineflag = 0;
        int toolIsOk = 0;
        try
	{
            if(src.channels() != 1)
                cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
	    for (int i = 0; i < relyOnResults.size(); i++)
	    {
                //std::cout<<"CenterX:"<<relyOnResults[i]["CenterX"].asInt()<<std::endl;
                //std::cout<<"CenterY:"<<relyOnResults[i]["CenterY"].asInt()<<std::endl;
		//���������ߵĽ���ж�����
                testDatum_point.x = relyOnResults[i]["CenterX"].asDouble();
                testDatum_point.y = relyOnResults[i]["CenterY"].asDouble();
		center_angle=relyOnResults[i]["Angle"].asInt();
	    }
            std::cout<<"-----angle:"<<center_angle<<std::endl;
	    //�����µļ���
	    Rotate_rect(center, center_angle, tempDatum_point, testDatum_point, detectRect, sample_width, sample_height);
            //std::cout<<detectRect.x<<":"<<detectRect.y<<std::endl;
	    lineflag=LineDetect(src, detectRect, line); 
            if(lineflag)
                toolIsOk = 1;
	}
        catch (std::exception e)
        {
            std::cout<<e.what()<<" :LineDetect fail!"<< std::endl;   //�����쳣��Ȼ��������
            return 0;
        }
        time.stop();
        int times=time.duration();
        
	//��ƥ������Json��ʽ�洢
        std::string resultStr="{\"ToolIsOk\":"+std::to_string(toolIsOk)+",\"linePoint1X\":"+std::to_string(line.p1.x)+",\"linePoint1Y\":"+std::to_string(line.p1.y)+",\"linePoint2X\":"+std::to_string(line.p2.x)+",\"linePoint2Y\":"+std::to_string(line.p2.y)+",\"RunTimes\":"+std::to_string(times)+"}";
        std::cout<<resultStr.c_str()<<std::endl; 
        Json::Reader reader;
        reader.parse(resultStr, result);
        return 1;
}

/*
��ת�������Rotate_rect����ģ��ͼ����centerΪ��ת���ģ���ת��������ٸ��ݻ�׼��������λ��
Point2f center: ������ת���ο����ת����
float center_angle�� ������ת���ο����ת�Ƕ�
Point2f tempDatum_point��ģ��ͼ���ڼ������λ�ƵĻ�׼��
Point testDatum_point������ͼ���ڼ������λ�ƵĻ�׼��
Rect &detectRect��ģ��ͼ������תǰ��⹤��������Σ��Լ���������ͼ���б߼���������Ͻ����꣬��ȡ��߶ȣ�
int t_w������ͼ��Ŀ�
int t_h������ͼ��ĸ�
*/
int  LineDetectTool::Rotate_rect(cv::Point2f center, float center_angle, cv::Point2f tempDatum_point, cv::Point testDatum_point, cv::Rect &detectRect, int t_w, int t_h)
{
        if(center_angle==0)
           return 1;
	cv::Point2f h11, h12, h13, h14;
	//��תǰ���꣬����Ϊ���ϡ����¡����ϡ�����
	h11.x = detectRect.x;
	h11.y = detectRect.y;
	h12.x = detectRect.x;
	h12.y = detectRect.y + detectRect.height;
	h13.x = detectRect.x + detectRect.width;
	h13.y = detectRect.y;
	h14.x = detectRect.x + detectRect.width;
	h14.y = detectRect.y + detectRect.height;

	//��ת�������
	cv::Point2f h1, h2, h3, h4;
	if (center_angle > 180)
	{
		center_angle = 360 - center_angle;
		double angle = center_angle * CV_PI / 180; // ����
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
		double angle = center_angle * CV_PI / 180; // ����
		h1.x = (h11.x - center.x)*cos(angle) - (h11.y - center.y)*sin(angle) + center.x;
		h1.y = (h11.y - center.y)*cos(angle) + (h11.x - center.x)*sin(angle) + center.y;
		h2.x = (h12.x - center.x)*cos(angle) - (h12.y - center.y)*sin(angle) + center.x;
		h2.y = (h12.y - center.y)*cos(angle) + (h12.x - center.x)*sin(angle) + center.y;
		h3.x = (h13.x - center.x)*cos(angle) - (h13.y - center.y)*sin(angle) + center.x;
		h3.y = (h13.y - center.y)*cos(angle) + (h13.x - center.x)*sin(angle) + center.y;
		h4.x = (h14.x - center.x)*cos(angle) - (h14.y - center.y)*sin(angle) + center.x;
		h4.y = (h14.y - center.y)*cos(angle) + (h14.x - center.x)*sin(angle) + center.y;
	}

	//�������λ��
	cv::Point pp1 = cv::Point(h1.x - tempDatum_point.x, h1.y - tempDatum_point.y);
	cv::Point pp2 = cv::Point(h2.x - tempDatum_point.x, h2.y - tempDatum_point.y);
	cv::Point pp3 = cv::Point(h3.x - tempDatum_point.x, h3.y - tempDatum_point.y);
	cv::Point pp4 = cv::Point(h4.x - tempDatum_point.x, h4.y - tempDatum_point.y);
	//�µ�����
	h1.x = testDatum_point.x + pp1.x;
	h1.y = testDatum_point.y + pp1.y;
	h2.x = testDatum_point.x + pp2.x;
	h2.y = testDatum_point.y + pp2.y;
	h3.x = testDatum_point.x + pp3.x;
	h3.y = testDatum_point.y + pp3.y;
	h4.x = testDatum_point.x + pp4.x;
	h4.y = testDatum_point.y + pp4.y;


	//waitKey(0);
	//��ת��������ϵƽ�о������Ϻ���������
	cv::Point2f p1, p4;
	p1.x = IVS_MIN(h1.x, h2.x, h3.x, h4.x);
	p1.y = IVS_MIN(h1.y, h2.y, h3.y, h4.y);
	p4.x = IVS_MAX(h1.x, h2.x, h3.x, h4.x);
	p4.y = IVS_MAX(h1.y, h2.y, h3.y, h4.y);
	//��ֹ�������ת��ͼ������
	if (p1.x >= t_w)
		p1.x = t_w - 1;
	if (p1.y >= t_h)
		p1.y = t_h - 1;
	if (p4.x >= t_w)
		p4.x = t_w - 1;
	if (p4.y >= t_h)
		p4.y = t_h - 1;

	//��detectRect��Ŵ���ͼ���ĸ��߼���������Ͻ����꼰������
	detectRect.x = p1.x;
	detectRect.y = p1.y;
	detectRect.width = p4.x - p1.x;
	detectRect.height = p4.y - p1.y;

	return 1;
}
/*
lineDetect�߼����ͨ����Ե�����ȡ��Ե�㣬�پ�����任���ֱ�߶�
Mat input:����ͼ��
Rect ROI:���ڼ��ߵ�����

�����Line���ڸ��������������
*/
int LineDetectTool::LineDetect(cv::Mat input, cv::Rect ROI, Line &l)
{
        
	cv::Mat inputROI = input(ROI);
	cv::Mat thresh_img;
	double thresh_value = threshold(inputROI, thresh_img, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);
	cv::Mat edges_img;
	std::vector<cv::Point> edges;
	cv::Canny(thresh_img, edges_img, thresh_value, 1.5*thresh_value);
	//imshow("edges_img", edges_img);
        //cv::waitKey(0);
	std::vector<cv::Vec4i> lines;//����һ��ʸ���ṹlines���ڴ�ŵõ����߶�ʸ������
	HoughLinesP(edges_img, lines, 1, CV_PI / 180, 80, 1, 100);
	//������ֱ����ϣ��������Ŀ����ж��ֱ�߶Σ����ǰ����ϳ�һ��
	std::vector<cv::Point> points;
	for (size_t i = 0; i < lines.size(); i++)
	{
		points.push_back(cv::Point(lines[i][0], lines[i][1]));
		points.push_back(cv::Point(lines[i][2], lines[i][3]));
	}
	cv::Vec4f line;
	cv::fitLine(points, line, CV_DIST_L2, 0, 0.01, 0.01);
	float vx = line[0];
	float vy = line[1];
	float k = vy / vx;
	float x0 = line[2];
	float y0 = line[3];
	//���ֱ������ ���ϲ���ȫ������ϵ
	if (k > -1 && k < 1)//ˮƽ��
	{
		int y1 = int((0 - x0)*k + y0);
		int y2 = int((inputROI.cols - x0)*k + y0);
		l.p1 = cv::Point((inputROI.cols - 1) + ROI.x, y2 + ROI.y);
		l.p2 = cv::Point(ROI.x, y1 + ROI.y);
	}
	else//��ֱ��
	{
		int x1 = x0 + (0 - y0) / k;
		int x2 = x0 + (inputROI.rows - y0) / k;
		l.p1 = cv::Point(x1 + ROI.x, +ROI.y);
		l.p2 = cv::Point(x2 + ROI.x, inputROI.rows + ROI.y);
	}
	/*cv::line(input, l.p1, l.p2, cv::Scalar(127), 2);
        cv::resize(input,input,cv::Size(800,800));
	cv::imshow("input",input);
	cv::waitKey();*/
	return 1;
}
