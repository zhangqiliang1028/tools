#ifndef IVS_FEATYRELOCATIONTOOL_H
#define IVS_FEATYRELOCATIONTOOL_H
#include "ToolBase.h"
#include "IVSParameters.h"
//Ԥ��������������
#define MAX_NUM_PYRAMID  8
// ģ���ӽṹ������ĳһ��ĳһ���Ƕȵ�ģ����Ϣ
struct IVSTemplateSubStruct
{
	/* ģ���������ļ������ʽ���������

	* 1. ģ�������б�Ե���λ�ã�
	* 2. ģ�������б�Ե����ݶ�ֵ��
	* 3. ģ�������б�Ե����X��Y�����ϵ��ݶȷ���
	* 4. ģ���б�Ե��ĸ�����
	* 5. ģ��ĸ߿�
	* 6. ģ������ġ�
	* */
	UINT8						modelDefined;
	UINT32						noOfCordinates;		//Number of elements in coordinate array ��Ե��ĸ���
	UINT16						modelHeight;		//Template height ģ��ĸ߶�
	UINT16						modelWidth;			//Template width ģ��Ŀ��
	cv::Point					centerOfGravity;	//Center of gravity of template ����
	std::vector<cv::Point>		cordinates;			//Coordinates array to store mo hjel points	model points Ҳ�������еı�Ե��
	std::vector<float>			edgeDerivativeX;	//gradient in X direction
	std::vector<float>			edgeDerivativeY;	//gradient in Y direction
};


// ģ��ṹ����������ģ��
struct IVSTemplateStruct {
	UINT8 pyramidLevelNum;
	std::vector<float> searchAngelStep;
	std::vector<UINT16> searchRectWidth;
	std::vector<std::vector<IVSTemplateSubStruct>>	tpls;
        bool templateIsOK;
};

struct CandidateResult {
	int level;
	int angleIndex;			// �Ƕ��±�
	int positionX;			// ����x����
	int positionY;			// ����y����
	float score;			// �ô�λ������

	bool operator<(const CandidateResult other) const {
		return score > other.score;
	}
};

/* �ݶȡ�����ͼ����ֵͼ*/  //ÿ�Ŵ���ͼƬ��һ������Ҫ�ڴ�����������¼���
struct ContourUtility {
	cv::Mat srcFiltered;				// ԭͼ
	cv::Mat gradx[MAX_NUM_PYRAMID];			// ����ͼ����x�����ݶ�ͼ
	cv::Mat grady[MAX_NUM_PYRAMID];			// ����ͼ����y�����ݶ�ͼ
	cv::Mat searchRegion[MAX_NUM_PYRAMID];		// ������ͼƬ������
	cv::Mat imageSearchInteg[MAX_NUM_PYRAMID];	// ����ͼ������������ͼֻ��Ҫ����������(��������Ϊ4��5)���²㶼��ʹ�ö�ֵͼ��0Ϊ4�㣬1Ϊ5��

	/* ��ǰͼƬ��һ�����ݶ������� */
	float **edgeX[MAX_NUM_PYRAMID];
	float **edgeY[MAX_NUM_PYRAMID];

	/* ȫͼ�ֱ��� */
	UINT16 u16Width;
	UINT16 u16Height;

	/* ���ұ�*/
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
	std::vector<cv::Rect> searchRect;				// ������������ȫ�ֺ;ֲ���
	std::vector<cv::Mat> cannyPyramid;				// ÿ�����߶��е�canny������
	//std::vector<cv::Mat> cannyDilatePyramid;			// ÿ�����߶��е�����canny������
	std::vector<std::priority_queue<CandidateResult>> candidates;	// ÿһ��ĺ�ѡλ����Ϣ���Ӹ߲����ײ��������0�����߲㣩


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
	// ��ʼ������
	int initTask(ContourUtility &contourUtility);
	// ִ�����񲢷��ؽ��
	int doTask(ContourUtility &contourUtility, UINT8 *contourResultAndBitmap, size_t *bufSize);
	// �ͷ�����
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
