#ifndef IVS_LINEDETECTTOOL_H
#define IVS_LINEDETECTTOOL_H
#include "ToolBase.h"
#include <iostream>

//�߶���
struct Line{
	cv::Point p1;//��ʼ�����յ�
        cv::Point p2;
};

class LineDetectTool :public ToolBase
{
private:
        std::vector<int> relyNums={};
	cv::Point2f center;//������ת�������꣺ģ��������λ�������ĵ�
	float center_angle;//ģ����ת�Ƕ�
	cv::Point2f tempDatum_point;//���μ������λ�ƻ�׼�����꣺ģ��������λ�������ĵ�
	cv::Point testDatum_point;//���μ������λ�ƻ�׼�����꣺����ͼ��λ�����ĵ�
	cv::Rect detectRect;//ģ������Σ�Ҳ�Ǵ���ͼ������
	int sample_width;//����ͼ�Ŀ�ȸ߶�
        int sample_height;
	Line line;

	int  Rotate_rect(cv::Point2f center, float center_angle, cv::Point2f tempDatum_point, cv::Point testDatum_point, cv::Rect &detectRect, int t_w, int t_h);
	int LineDetect(cv::Mat input, cv::Rect ROI, Line &l);

public:
	LineDetectTool(){};
	virtual~LineDetectTool() {};
        virtual int getRelyOnParam(std::vector<int> &nums);
        virtual int initTool(Json::Value json);
        virtual void updateTool(Json::Value json){};
	virtual int run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result);
        virtual int run(cv::Mat src,cv::Mat &dst ){};
        virtual int emptyTool(){};

};
#endif 
