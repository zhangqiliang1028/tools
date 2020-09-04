#ifndef IVS_FEATYRELOCATIONTOOL_H
#define IVS_FEATYRELOCATIONTOOL_H
#include "ToolBase.h"
#include "IVSParameters.h"
//预留金字塔最大层数
#define MAX_NUM_PYRAMID  8
// 模板子结构，代表某一层某一个角度的模板信息
struct IVSTemplateSubStruct
{
	/* 模板的由下面的几个性质进行描述：

	* 1. 模板中所有边缘点的位置；
	* 2. 模板中所有边缘点的梯度值；
	* 3. 模板中所有边缘点在X和Y方向上的梯度方向；
	* 4. 模板中边缘点的个数；
	* 5. 模板的高宽；
	* 6. 模板的重心。
	* */
	UINT8						modelDefined;
	UINT32						noOfCordinates;		//Number of elements in coordinate array 边缘点的个数
	UINT16						modelHeight;		//Template height 模板的高度
	UINT16						modelWidth;			//Template width 模板的宽度
	cv::Point					centerOfGravity;	//Center of gravity of template 重心
	std::vector<cv::Point>		cordinates;			//Coordinates array to store mo hjel points	model points 也就是所有的边缘点
	std::vector<float>			edgeDerivativeX;	//gradient in X direction
	std::vector<float>			edgeDerivativeY;	//gradient in Y direction
};


// 模板结构，代表所有模板
struct IVSTemplateStruct {
	UINT8 pyramidLevelNum;
	std::vector<float> searchAngelStep;
	std::vector<UINT16> searchRectWidth;
	std::vector<std::vector<IVSTemplateSubStruct>>	tpls;
        bool templateIsOK;
};

struct CandidateResult {
	int level;
	int angleIndex;			// 角度下标
	int positionX;			// 质心x坐标
	int positionY;			// 质心y坐标
	float score;			// 该处位置评分

	bool operator<(const CandidateResult other) const {
		return score > other.score;
	}
};

/* 梯度、积分图、二值图*/  //每张待测图片不一样，需要在处理进程中重新计算
struct ContourUtility {
	cv::Mat srcFiltered;				// 原图
	cv::Mat gradx[MAX_NUM_PYRAMID];			// 待测图各层x方向梯度图
	cv::Mat grady[MAX_NUM_PYRAMID];			// 待测图各层y方向梯度图
	cv::Mat searchRegion[MAX_NUM_PYRAMID];		// 降采样图片金字塔
	cv::Mat imageSearchInteg[MAX_NUM_PYRAMID];	// 积分图金字塔，积分图只需要最上面两层(层数可能为4或5)，下层都是使用二值图，0为4层，1为5层

	/* 当前图片归一化的梯度向量表 */
	float **edgeX[MAX_NUM_PYRAMID];
	float **edgeY[MAX_NUM_PYRAMID];

	/* 全图分辨率 */
	UINT16 u16Width;
	UINT16 u16Height;

	/* 查找表*/
	float **lookupTableX;
	float **lookupTableY;
	float **lookupTableS;
};
class FeatureLocationTool :public ToolBase
{
private:
	cv::Mat template_image;
	int sampleImageWidth;
        int sampleImageHeight;
	IVSToolContourParameter toolParameter;
	IVSTemplateStruct templateInfo;

	ContourUtility contourUtility;
	std::vector<cv::Rect> searchRect;				// 代表搜索区域（全局和局部）
	std::vector<cv::Mat> cannyPyramid;				// 每个工具独有的canny金字塔
	//std::vector<cv::Mat> cannyDilatePyramid;			// 每个工具独有的膨胀canny金字塔
	std::vector<std::priority_queue<CandidateResult>> candidates;	// 每一层的候选位置信息（从高层往底层填充结果，0层代表高层）


	int unpackTemplate(char* buf);
	void print_tpl(const IVSTemplateSubStruct &tpl);
        int doCreateTemplate(const cv::Mat &src, const cv::Mat &bitMap);
   
	INT8 doTemplateMatch(ContourUtility &contourUtility);
	int doTopLayerMatch(ContourUtility &contourUtility);
	int doOtherLayerMatch(ContourUtility &contourUtility, int cur_level);
	int createUtility(ContourUtility &contourUtility,int width, int height);
	int computeUtility(ContourUtility &contourUtility,cv::Mat sample_image);
	int freeUtility(ContourUtility &contourUtility);
	int ivs_create_template();
	int ivs_get_contours(cv::Mat template_image, const IVSToolContourParameter &ivsToolContourParameter, UINT8 *contours);
	// 初始化任务
	int initTask(ContourUtility &contourUtility);
	// 执行任务并返回结果
	int doTask(ContourUtility &contourUtility, UINT8 *contourResultAndBitmap, size_t *bufSize);
	// 释放任务
	int freeTask();
        

public:
	FeatureLocationTool(){};
	virtual~FeatureLocationTool(){};
	virtual int getRelyOnParam(std::vector<int> &nums){};
        virtual int initTool(Json::Value json);
        virtual void updateTool(Json::Value json);
	virtual int run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result);
        virtual int run(cv::Mat src,cv::Mat &dst ){};
        virtual int emptyTool();
};
#endif 
