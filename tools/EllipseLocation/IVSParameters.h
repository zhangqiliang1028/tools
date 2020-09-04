#ifndef IVSPARAMETER_H
#define IVSPARAMETER_H

#include "IVSVarTypes.h"

//ԭʼͼ��ֱ���
#define IMAGE_WIDTH                3072
#define IMAGE_HEIGHT               2048
//ǰ̨��ʾͼ��ֱ���
#define IMAGE_SHOW_WIDTH           1360
//#define IMAGE_SHOW_HEIGHT          1020
#define imageScale ((double)IMAGE_WIDTH/IMAGE_SHOW_WIDTH)
//ģ��ͼƬ�ṹ��
typedef struct ModelFile {
	UINT32                  width;                //ͼƬ���
	UINT32                  height;               //ͼƬ�߶�
	ENUM_RAW_DATA_FORMAT    rawDataFormat;        //ԭʼͼ��洢��ʽ
	ENUM_RGB_DATA_FORMAT   rgbDataFormat;        //RGBͼ��洢��ʽ
	ADDR32                  pRawDataAddr;	      //ԭʼͼ��洢��ַ
	ADDR32                  pRgbDataAddr;         //RGBͼ��洢��ַ
}ModelFile;
\
// �ýṹ��ֻ�����㷨ͼƬ�ṹ�������̵߳����㷨ʱ����Ҫ�Ըýṹ�崦��
typedef struct AlgoPic {
	UINT32                  width;                //ͼƬ���
	UINT32                  height;               //ͼƬ�߶�
	ENUM_RAW_DATA_FORMAT    rawDataFormat;        //ԭʼͼ��洢��ʽ
	ADDR32                  pRawDataAddr;	      //ԭʼͼ��洢��ַ
}AlgoPic;

struct IVSToolContourParameter
{
	//INT8	toolName[32];			// ��������
	UINT8	regionShape;			// ���������״������Ϊ1��Բ��Ϊ0��
	UINT8	toolType;				// ��������

	/* ���μ������
	*
	*		0-----------3
	*		|			|
	*		|			|
	*		1-----------2
	*/
	
	INT16	detectRectX0;			// ������Ͻ�Բ���������ϵ������
	INT16	detectRectY0;			// ������Ͻ�Բ���������ϵ�������
	INT16	detectRectX1;			// ������Ͻ�Բ���������µ������
	INT16	detectRectY1;			// ������Ͻ�Բ���������µ�������
	INT16	detectRectX2;			// ������Ͻ�Բ���������µ������
	INT16	detectRectY2;			// ������Ͻ�Բ���������µ�������
	INT16	detectRectX3;			// ������Ͻ�Բ���������ϵ������
	INT16	detectRectY3;			// ������Ͻ�Բ���������ϵ�������
	
	/* ���Բ���� */
	INT16	detectCircleX;			// ���Բ�ĺ�����
	INT16	detectCircleY;			// ���Բ��������
	UINT16	detectCircleRadius;		// ���Բ�뾶
	


	/* ��Ӿ���������� */
	INT16	extRectX;			// �����Ӿ�����ʼ�������
	INT16	extRectY;			// �����Ӿ�����ʼ��������
	UINT16	extRectWidth;			// �����Ӿ��ο��
	UINT16	extRectHeight;			// �����Ӿ��θ߶�

	/* ����������� */
	INT16	searchRectX0;			// ������Ͻ�Բ���������ϵ������
	INT16	searchRectY0;			// ������Ͻ�Բ���������ϵ�������
	INT16	searchRectX1;			// ������Ͻ�Բ���������µ������
	INT16	searchRectY1;			// ������Ͻ�Բ���������µ�������
	INT16	searchRectX2;			// ������Ͻ�Բ���������µ������
	INT16	searchRectY2;			// ������Ͻ�Բ���������µ�������
	INT16	searchRectX3;			// ������Ͻ�Բ���������ϵ������
	INT16	searchRectY3;			// ������Ͻ�Բ���������ϵ�������

	/* �㷨������� */
	UINT8	angleRange;			// �����Ƕȷ�Χ
	UINT8	algoStrategy;			// ���ٶȻ��߸߾����㷨���ԣ��߾���Ϊ0�����ٶ�Ϊ1��
	UINT8	sensiTopThreshold;		// canny������ֵ����
	UINT8	sensiLowThreshold;		// canny������ֵ����
        UINT8   contourCandidateNum;            // ����ɸѡ��ѡ��Ϣ�ĸ���,������Ϊ30�����²����
        UINT16  contourNumThreshold;            // ������������С��ֵ 
        UINT16  contourScoreThreshold;          // �������ƶ���ֵ����С���ƶ� 
        UINT8   contourNaborSize;               // ����߲��ѡ�����ƶ���Χ
        UINT8   contourNaborAngle;              // ����߲��ѡ�Ƕ�ƫ�Ʒ�Χ
        UINT8   minContourPyra;                 //���Ƹ߲����ٵ���������
        UINT8   maxNumPyramid;                  //�߲㷶Χmin~max:3~4,���ٶ�ʱ�ã�����������������
        UINT8   minNumPyramid;              
        UINT8   highPrecisionNumPyramid;        //�߾���ʱ����������߲��� 

	/* �㷨����������� */
	UINT16	scoreTopThreshold;		// ��������ֵ����
	UINT16	scoreLowThreshold;		// ��������ֵ����
	//UINT32 templateBitmapSize;		// ����ģ��λͼ�Ĵ�С
	//INT8	templatePath[128];		// λͼ·��
};


struct IVSToolColorParameter
{
	INT8	toolName[32];			// ��������
	UINT8	regionShape;			// ����������״
	UINT8	toolType;				// ��������

	/* ���μ������
	*
	*		0-----------3
	*		|			|
	*		|			|
	*		1-----------2
	*/

	INT16	detectRectX0;			// ������Ͻ�Բ���������ϵ������
	INT16	detectRectY0;			// ������Ͻ�Բ���������ϵ�������
	INT16	detectRectX1;			// ������Ͻ�Բ���������µ������
	INT16	detectRectY1;			// ������Ͻ�Բ���������µ�������
	INT16	detectRectX2;			// ������Ͻ�Բ���������µ������
	INT16	detectRectY2;			// ������Ͻ�Բ���������µ�������
	INT16	detectRectX3;			// ������Ͻ�Բ���������ϵ������
	INT16	detectRectY3;			// ������Ͻ�Բ���������ϵ�������

	/* ���Բ���� */
	INT16	detectCircleX;			// ���Բ�ĺ�����
	INT16	detectCircleY;			// ���Բ��������
	UINT16	detectCircleRadius;		// ���Բ�뾶

	/* ��Ӿ���������� */
	INT16	extRectX;				// �����Ӿ�����ʼ�������
	INT16	extRectY;				// �����Ӿ�����ʼ��������
	UINT16	extRectWidth;			// �����Ӿ��ο��
	UINT16	extRectHeight;			// �����Ӿ��θ߶�

	/* �㷨����������� */
	UINT32	pixelCount;				// ���ظ���
	UINT16	HSVMin[3];				// hsv����
	UINT16	HSVMax[3];				// hsv����
	UINT16	scoreTopThreshold;		// ��������ֵ����
	UINT16	scoreLowThreshold;		// ��������ֵ����
};

struct IVSToolWidthParameter
{
	INT8	toolName[32];			// ��������
	UINT8	toolType;				// ��������
	UINT8	aglin;					// �������

	/* ���������� */
	INT16	rectCenterX;			// ���μ������ĺ�����
	INT16	rectCenterY;			// ���μ�������������
	UINT16	rectWidth;				// ���μ�����
	UINT16	rectHeight;				// ���μ���߶�
	INT16	rectAngle;				// ���μ�����ת�Ƕȣ���ʱ��Ϊ����ˮƽ����Ϊ0�ȣ�

	/* �㷨����������� */
	UINT16	detectWidth;			// ��ȼ��Ŀ����
	UINT8	detectDirect;			// ��ȼ�ⷽ��
	UINT8	sensiWidthDetect;		// ��ȼ��������
	UINT16	detectLeftLine;			// ��ȼ������λ��
	UINT16	detectRightLine;		// ��ȼ������λ��
	UINT16	scoreTopThreshold;		// ��������ֵ����
	UINT16	scoreLowThreshold;		// ��������ֵ����
};


struct IVSToolCircleParameter
{
	INT8	toolName[32];			// ��������
	UINT8	toolType;				// ��������
	UINT8	sensiCircleDetect;		// Բ���������

	/* ���������� */
	INT16	searchCircleX;			// ����Բ��Բ�ĺ�����
	INT16	searchCircleY;			// ����Բ��Բ��������
	UINT16	searchCircleRadius;		// ����Բ��Բ�뾶

	/* �㷨����������� */
	INT16	detectCircleX;			// ѡȡĳһ��⵽��Բ��Բ�ĺ�����
	INT16	detectCircleY;			// ѡȡĳһ��⵽��Բ��Բ��������
	INT16	detectCircleRadius;		// ѡȡĳһ��⵽��Բ��Բ�뾶

	UINT16	scoreTopThreshold;		// ��������ֵ����
	UINT16	scoreLowThreshold;		// ��������ֵ����
};


/* �������ߴ����� */
struct IVSContourResult
{
	INT32	toolId;					// ID���ʹ�ȷ�ϡ�
	INT8	toolName[32];			// ��������
	INT32	toolType;				// ��������
	UINT8	regionShape;			// ����������״
	INT8	toolIsOk;				// ���߽���Ƿ�OK
	INT16	toolScore;				// ���߽������

	/* ���μ������ */
	INT16	detectRectX0;			// ������Ͻ�Բ���������ϵ������
	INT16	detectRectY0;			// ������Ͻ�Բ���������ϵ�������
	INT16	detectRectX1;			// ������Ͻ�Բ���������µ������
	INT16	detectRectY1;			// ������Ͻ�Բ���������µ�������
	INT16	detectRectX2;			// ������Ͻ�Բ���������µ������
	INT16	detectRectY2;			// ������Ͻ�Բ���������µ�������
	INT16	detectRectX3;			// ������Ͻ�Բ���������ϵ������
	INT16	detectRectY3;			// ������Ͻ�Բ���������ϵ�������

	/* ���Բ���� */
	INT16	detectCircleX;			// ���Բ�ĺ�����
	INT16	detectCircleY;			// ���Բ��������
	UINT16	detectCircleRadius;		// ���Բ�뾶

	/* ģ����Ӿ���������� */
	INT16	extRectX;				// �����Ӿ�����ʼ�������
	INT16	extRectY;				// �����Ӿ�����ʼ��������
	UINT16	extRectWidth;			// �����Ӿ��ο��
	UINT16	extRectHeight;			// �����Ӿ��θ߶�

	/* ����������� */
	INT16	searchRectX0;			// ������Ͻ�Բ���������ϵ������
	INT16	searchRectY0;			// ������Ͻ�Բ���������ϵ�������
	INT16	searchRectX1;			// ������Ͻ�Բ���������µ������
	INT16	searchRectY1;			// ������Ͻ�Բ���������µ�������
	INT16	searchRectX2;			// ������Ͻ�Բ���������µ������
	INT16	searchRectY2;			// ������Ͻ�Բ���������µ�������
	INT16	searchRectX3;			// ������Ͻ�Բ���������ϵ������
	INT16	searchRectY3;			// ������Ͻ�Բ���������ϵ�������

	INT16	algin;					// �������
	UINT32	bitmapSize;				// ʵ��λͼ��С(ģ����Ӿ���λͼ,����8�ı�������8λ)��λ��bit
};

/* ��ɫ������ߴ����� */
struct IVSColorResult
{
	INT32	toolId;					// ID���ʹ�ȷ�ϡ�
	INT8	toolName[32];			// ��������
	INT16	toolScore;				// ���߽������
	INT16	toolIsOk;				// ���߽���Ƿ�OK
	INT32	toolType;				// ��������

	UINT8	regionShape;			// ����������״
	UINT8	aglin;					// �������

	/* ���μ������ */
	INT16	detectRectX0;			// ������Ͻ�Բ���������ϵ������
	INT16	detectRectY0;			// ������Ͻ�Բ���������ϵ�������
	INT16	detectRectX1;			// ������Ͻ�Բ���������µ������
	INT16	detectRectY1;			// ������Ͻ�Բ���������µ�������
	INT16	detectRectX2;			// ������Ͻ�Բ���������µ������
	INT16	detectRectY2;			// ������Ͻ�Բ���������µ�������
	INT16	detectRectX3;			// ������Ͻ�Բ���������ϵ������
	INT16	detectRectY3;			// ������Ͻ�Բ���������ϵ�������

	/* ���Բ���� */
	INT16	detectCircleX;			// ���Բ�ĺ�����
	INT16	detectCircleY;			// ���Բ��������
	UINT16	detectCircleRadius;		// ���Բ�뾶

	/* ģ����Ӿ���������� */
	INT16	extRectX;				// �����Ӿ�����ʼ�������
	INT16	extRectY;				// �����Ӿ�����ʼ��������
	UINT16	extRectWidth;			// �����Ӿ��ο��
	UINT16	extRectHeight;			// �����Ӿ��θ߶�

	/* �㷨����������� */
	UINT16	HSVMin[3];				// hsv����
	UINT16	HSVMax[3];				// hsv����
	UINT32	pixelCount;				// ���ظ���

	UINT32	bitmapSize;				// ʵ��λͼ��С(ģ����Ӿ���λͼ,����8�ı�������8λ)��λ��bit
};

/* ��ȹ��ߴ����� */
struct IVSWidthResult
{
	INT32	toolId;					// ID���ʹ�ȷ�ϡ�
	INT8	toolName[32];			// ��������
	INT16	toolScore;				// ���߽������
	INT16	toolIsOk;				// ���߽���Ƿ�OK
	INT32	toolType;				// ��������

	/* ���������� */
	INT16	rectCenterX;			// ���μ������ĺ�����
	INT16	rectCenterY;			// ���μ�������������
	UINT16	rectWidth;				// ���μ�����
	UINT16	rectHeight;				// ���μ���߶�
	INT32	rectAngle;				// ���μ�����ת�Ƕȣ���ʱ��Ϊ����ˮƽ����Ϊ0�ȣ�

	/* �㷨����������� */
	UINT16	detectWidth;			// ��ȼ��Ŀ����
	UINT16	lineNum;				// �ߵ�����
	UINT8	detectDirect;			// ��ȼ�ⷽ��
	UINT8	sensiWidthDetect;		// ��ȼ��������
	UINT16	detectLeftLine;			// ��ȼ������λ��
	UINT16	detectRightLine;		// ��ȼ������λ��

	UINT16	widthEdge[2];			// �����ߵĺ�����

	UINT8	ifDetetctSuccess;		// �Ƿ��⵽�Ϸ��Ŀ��
	UINT8	ifToolSuccess;			// ����������⵽�Ŀ���Ƿ��ڲ��������ȹ涨�ķ�Χ�ڡ�100*(1-abs(dp-dd)/dp) > sensitivity��
	INT32	resultLen;				// ��⵽�Ŀ�ȴ�С
};

/* ֱ�����߽���ṹ�� */
struct IVSCircleResult
{
	INT32	toolId;					// ID���ʹ�ȷ�ϡ�
	INT8	toolName[32];			// ��������
	INT16	toolScore;				// ���߽������
	INT16	toolIsOk;				// ���߽���Ƿ�OK
	INT32	toolType;				// ��������

	INT16	searchCircleX;			// ����Բ��Բ�ĺ�����
	INT16	searchCircleY;			// ����Բ��Բ��������
	UINT16	searchCircleRadius;		// ����Բ��Բ�뾶
	UINT16  circleNum;				// ��������Բ������
	INT16	circleX;				// ��⵽��Բ��Բ�ĺ�����
	INT16	circleY;				// ��⵽��Բ��Բ��������
	UINT16	circleRadius;			// ��⵽��Բ��Բ�뾶
	INT16	align;					// �������
};

//���νṹ�壬�������Ϊ���Ͻǵ㣨iX0,iY0������ʱ��ת����Ϊ��iX1,iY1����iX2,iY2���ͣ�iX3,iY3��
struct IVSRectangle {
	INT16 x0;
	INT16 y0;
	INT16 x1;
	INT16 y1;
	INT16 x2;
	INT16 y2;
	INT16 x3;
	INT16 y3;
};

//Բ�ṹ��
struct IVSCircle {
	//Բ�����꣨iX��iY��
	INT16 x;
	INT16 y;
	UINT16 radius;//Բ�İ뾶
};

/***
 @brief: ö�ٹ�������
 ***/
enum TOOL_TYPE {
	CONTOUR_TOOL = 0,
	COLOR_TOOL = 1,
	WIDTH_TOOL = 2,
	CIRCLE_TOOL = 3
};

// ���������״
#define REGION_SHAPE_CIRC 0
#define REGION_SHAPE_RECT 1







#endif // IVSPARAMETER_H
