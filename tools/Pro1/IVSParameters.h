#ifndef IVSPARAMETER_H
#define IVSPARAMETER_H

#include "IVSVarTypes.h"

//原始图像分辨率
#define IMAGE_WIDTH                3072
#define IMAGE_HEIGHT               2048
//前台显示图像分辨率
#define IMAGE_SHOW_WIDTH           1360
//#define IMAGE_SHOW_HEIGHT          1020
#define imageScale ((double)IMAGE_WIDTH/IMAGE_SHOW_WIDTH)
//模板图片结构体
typedef struct ModelFile {
	UINT32                  width;                //图片宽度
	UINT32                  height;               //图片高度
	ENUM_RAW_DATA_FORMAT    rawDataFormat;        //原始图像存储格式
	ENUM_RGB_DATA_FORMAT   rgbDataFormat;        //RGB图像存储格式
	ADDR32                  pRawDataAddr;	      //原始图像存储地址
	ADDR32                  pRgbDataAddr;         //RGB图像存储地址
}ModelFile;
\
// 该结构体只用于算法图片结构，处理线程调用算法时，需要对该结构体处理
typedef struct AlgoPic {
	UINT32                  width;                //图片宽度
	UINT32                  height;               //图片高度
	ENUM_RAW_DATA_FORMAT    rawDataFormat;        //原始图像存储格式
	ADDR32                  pRawDataAddr;	      //原始图像存储地址
}AlgoPic;

struct IVSToolContourParameter
{
	//INT8	toolName[32];			// 工具名称
	UINT8	regionShape;			// 检测区域形状（矩形为1，圆形为0）
	UINT8	toolType;				// 工具类型

	/* 矩形检测框参数
	*
	*		0-----------3
	*		|			|
	*		|			|
	*		1-----------2
	*/
	
	INT16	detectRectX0;			// 相对左上角圆点坐标左上点横坐标
	INT16	detectRectY0;			// 相对左上角圆点坐标左上点纵坐标
	INT16	detectRectX1;			// 相对左上角圆点坐标左下点横坐标
	INT16	detectRectY1;			// 相对左上角圆点坐标左下点纵坐标
	INT16	detectRectX2;			// 相对左上角圆点坐标右下点横坐标
	INT16	detectRectY2;			// 相对左上角圆点坐标右下点纵坐标
	INT16	detectRectX3;			// 相对左上角圆点坐标右上点横坐标
	INT16	detectRectY3;			// 相对左上角圆点坐标右上点纵坐标
	
	/* 检测圆参数 */
	INT16	detectCircleX;			// 检测圆心横坐标
	INT16	detectCircleY;			// 检测圆心纵坐标
	UINT16	detectCircleRadius;		// 检测圆半径
	


	/* 外接矩形区域参数 */
	INT16	extRectX;			// 检测外接矩形起始点横坐标
	INT16	extRectY;			// 检测外接矩形起始点纵坐标
	UINT16	extRectWidth;			// 检测外接矩形宽度
	UINT16	extRectHeight;			// 检测外接矩形高度

	/* 搜索区域参数 */
	INT16	searchRectX0;			// 相对左上角圆点坐标左上点横坐标
	INT16	searchRectY0;			// 相对左上角圆点坐标左上点纵坐标
	INT16	searchRectX1;			// 相对左上角圆点坐标左下点横坐标
	INT16	searchRectY1;			// 相对左上角圆点坐标左下点纵坐标
	INT16	searchRectX2;			// 相对左上角圆点坐标右下点横坐标
	INT16	searchRectY2;			// 相对左上角圆点坐标右下点纵坐标
	INT16	searchRectX3;			// 相对左上角圆点坐标右上点横坐标
	INT16	searchRectY3;			// 相对左上角圆点坐标右上点纵坐标

	/* 算法策略相关 */
	UINT8	angleRange;			// 搜索角度范围
	UINT8	algoStrategy;			// 高速度或者高精度算法策略（高精度为0，高速度为1）
	UINT8	sensiTopThreshold;		// canny算子阈值上限
	UINT8	sensiLowThreshold;		// canny算子阈值下限
        UINT8   contourCandidateNum;            // 限制筛选候选信息的个数,顶层设为30，往下层减半
        UINT16  contourNumThreshold;            // 轮廓点数的最小阈值 
        UINT16  contourScoreThreshold;          // 计算相似度阈值的最小相似度 
        UINT8   contourNaborSize;               // 非最高层候选坐标移动范围
        UINT8   contourNaborAngle;              // 非最高层候选角度偏移范围
        UINT8   minContourPyra;                 //限制高层最少的轮廓点数
        UINT8   maxNumPyramid;                  //高层范围min~max:3~4,高速度时用，有轮廓点数来限制
        UINT8   minNumPyramid;              
        UINT8   highPrecisionNumPyramid;        //高精度时用来限制最高层数 

	/* 算法处理及评分相关 */
	UINT16	scoreTopThreshold;		// 处理结果阈值上限
	UINT16	scoreLowThreshold;		// 处理结果阈值下限
	//UINT32 templateBitmapSize;		// 工具模板位图的大小
	//INT8	templatePath[128];		// 位图路径
};


struct IVSToolColorParameter
{
	INT8	toolName[32];			// 工具名称
	UINT8	regionShape;			// 搜索区域形状
	UINT8	toolType;				// 工具类型

	/* 矩形检测框参数
	*
	*		0-----------3
	*		|			|
	*		|			|
	*		1-----------2
	*/

	INT16	detectRectX0;			// 相对左上角圆点坐标左上点横坐标
	INT16	detectRectY0;			// 相对左上角圆点坐标左上点纵坐标
	INT16	detectRectX1;			// 相对左上角圆点坐标左下点横坐标
	INT16	detectRectY1;			// 相对左上角圆点坐标左下点纵坐标
	INT16	detectRectX2;			// 相对左上角圆点坐标右下点横坐标
	INT16	detectRectY2;			// 相对左上角圆点坐标右下点纵坐标
	INT16	detectRectX3;			// 相对左上角圆点坐标右上点横坐标
	INT16	detectRectY3;			// 相对左上角圆点坐标右上点纵坐标

	/* 检测圆参数 */
	INT16	detectCircleX;			// 检测圆心横坐标
	INT16	detectCircleY;			// 检测圆心纵坐标
	UINT16	detectCircleRadius;		// 检测圆半径

	/* 外接矩形区域参数 */
	INT16	extRectX;				// 检测外接矩形起始点横坐标
	INT16	extRectY;				// 检测外接矩形起始点纵坐标
	UINT16	extRectWidth;			// 检测外接矩形宽度
	UINT16	extRectHeight;			// 检测外接矩形高度

	/* 算法处理及评分相关 */
	UINT32	pixelCount;				// 像素个数
	UINT16	HSVMin[3];				// hsv下限
	UINT16	HSVMax[3];				// hsv上限
	UINT16	scoreTopThreshold;		// 处理结果阈值上限
	UINT16	scoreLowThreshold;		// 处理结果阈值下限
};

struct IVSToolWidthParameter
{
	INT8	toolName[32];			// 工具名称
	UINT8	toolType;				// 工具类型
	UINT8	aglin;					// 对齐填充

	/* 检测区域相关 */
	INT16	rectCenterX;			// 矩形检测框中心横坐标
	INT16	rectCenterY;			// 矩形检测框中心纵坐标
	UINT16	rectWidth;				// 矩形检测框宽度
	UINT16	rectHeight;				// 矩形检测框高度
	INT16	rectAngle;				// 矩形检测框旋转角度（逆时针为正，水平方向为0度）

	/* 算法处理及评分相关 */
	UINT16	detectWidth;			// 宽度检测目标宽度
	UINT8	detectDirect;			// 宽度检测方向
	UINT8	sensiWidthDetect;		// 宽度检测灵敏度
	UINT16	detectLeftLine;			// 宽度检测左线位置
	UINT16	detectRightLine;		// 宽度检测右线位置
	UINT16	scoreTopThreshold;		// 处理结果阈值上限
	UINT16	scoreLowThreshold;		// 处理结果阈值下限
};


struct IVSToolCircleParameter
{
	INT8	toolName[32];			// 工具名称
	UINT8	toolType;				// 工具类型
	UINT8	sensiCircleDetect;		// 圆检测灵敏度

	/* 检测区域相关 */
	INT16	searchCircleX;			// 搜索圆的圆心横坐标
	INT16	searchCircleY;			// 搜索圆的圆心纵坐标
	UINT16	searchCircleRadius;		// 搜索圆的圆半径

	/* 算法处理及评分相关 */
	INT16	detectCircleX;			// 选取某一检测到的圆的圆心横坐标
	INT16	detectCircleY;			// 选取某一检测到的圆的圆心纵坐标
	INT16	detectCircleRadius;		// 选取某一检测到的圆的圆半径

	UINT16	scoreTopThreshold;		// 处理结果阈值上限
	UINT16	scoreLowThreshold;		// 处理结果阈值下限
};


/* 轮廓工具处理结果 */
struct IVSContourResult
{
	INT32	toolId;					// ID类型待确认。
	INT8	toolName[32];			// 工具名称
	INT32	toolType;				// 工具类型
	UINT8	regionShape;			// 搜索区域形状
	INT8	toolIsOk;				// 工具结果是否OK
	INT16	toolScore;				// 工具结果评分

	/* 矩形检测框参数 */
	INT16	detectRectX0;			// 相对左上角圆点坐标左上点横坐标
	INT16	detectRectY0;			// 相对左上角圆点坐标左上点纵坐标
	INT16	detectRectX1;			// 相对左上角圆点坐标左下点横坐标
	INT16	detectRectY1;			// 相对左上角圆点坐标左下点纵坐标
	INT16	detectRectX2;			// 相对左上角圆点坐标右下点横坐标
	INT16	detectRectY2;			// 相对左上角圆点坐标右下点纵坐标
	INT16	detectRectX3;			// 相对左上角圆点坐标右上点横坐标
	INT16	detectRectY3;			// 相对左上角圆点坐标右上点纵坐标

	/* 检测圆参数 */
	INT16	detectCircleX;			// 检测圆心横坐标
	INT16	detectCircleY;			// 检测圆心纵坐标
	UINT16	detectCircleRadius;		// 检测圆半径

	/* 模板外接矩形区域参数 */
	INT16	extRectX;				// 检测外接矩形起始点横坐标
	INT16	extRectY;				// 检测外接矩形起始点纵坐标
	UINT16	extRectWidth;			// 检测外接矩形宽度
	UINT16	extRectHeight;			// 检测外接矩形高度

	/* 搜索区域参数 */
	INT16	searchRectX0;			// 相对左上角圆点坐标左上点横坐标
	INT16	searchRectY0;			// 相对左上角圆点坐标左上点纵坐标
	INT16	searchRectX1;			// 相对左上角圆点坐标左下点横坐标
	INT16	searchRectY1;			// 相对左上角圆点坐标左下点纵坐标
	INT16	searchRectX2;			// 相对左上角圆点坐标右下点横坐标
	INT16	searchRectY2;			// 相对左上角圆点坐标右下点纵坐标
	INT16	searchRectX3;			// 相对左上角圆点坐标右上点横坐标
	INT16	searchRectY3;			// 相对左上角圆点坐标右上点纵坐标

	INT16	algin;					// 对齐填充
	UINT32	bitmapSize;				// 实际位图大小(模板外接矩形位图,不足8的倍数补足8位)单位是bit
};

/* 颜色面积工具处理结果 */
struct IVSColorResult
{
	INT32	toolId;					// ID类型待确认。
	INT8	toolName[32];			// 工具名称
	INT16	toolScore;				// 工具结果评分
	INT16	toolIsOk;				// 工具结果是否OK
	INT32	toolType;				// 工具类型

	UINT8	regionShape;			// 搜索区域形状
	UINT8	aglin;					// 对齐填充

	/* 矩形检测框参数 */
	INT16	detectRectX0;			// 相对左上角圆点坐标左上点横坐标
	INT16	detectRectY0;			// 相对左上角圆点坐标左上点纵坐标
	INT16	detectRectX1;			// 相对左上角圆点坐标左下点横坐标
	INT16	detectRectY1;			// 相对左上角圆点坐标左下点纵坐标
	INT16	detectRectX2;			// 相对左上角圆点坐标右下点横坐标
	INT16	detectRectY2;			// 相对左上角圆点坐标右下点纵坐标
	INT16	detectRectX3;			// 相对左上角圆点坐标右上点横坐标
	INT16	detectRectY3;			// 相对左上角圆点坐标右上点纵坐标

	/* 检测圆参数 */
	INT16	detectCircleX;			// 检测圆心横坐标
	INT16	detectCircleY;			// 检测圆心纵坐标
	UINT16	detectCircleRadius;		// 检测圆半径

	/* 模板外接矩形区域参数 */
	INT16	extRectX;				// 检测外接矩形起始点横坐标
	INT16	extRectY;				// 检测外接矩形起始点纵坐标
	UINT16	extRectWidth;			// 检测外接矩形宽度
	UINT16	extRectHeight;			// 检测外接矩形高度

	/* 算法及处理结果相关 */
	UINT16	HSVMin[3];				// hsv下限
	UINT16	HSVMax[3];				// hsv上限
	UINT32	pixelCount;				// 像素个数

	UINT32	bitmapSize;				// 实际位图大小(模板外接矩形位图,不足8的倍数补足8位)单位是bit
};

/* 宽度工具处理结果 */
struct IVSWidthResult
{
	INT32	toolId;					// ID类型待确认。
	INT8	toolName[32];			// 工具名称
	INT16	toolScore;				// 工具结果评分
	INT16	toolIsOk;				// 工具结果是否OK
	INT32	toolType;				// 工具类型

	/* 检测区域相关 */
	INT16	rectCenterX;			// 矩形检测框中心横坐标
	INT16	rectCenterY;			// 矩形检测框中心纵坐标
	UINT16	rectWidth;				// 矩形检测框宽度
	UINT16	rectHeight;				// 矩形检测框高度
	INT32	rectAngle;				// 矩形检测框旋转角度（逆时针为正，水平方向为0度）

	/* 算法处理及评分相关 */
	UINT16	detectWidth;			// 宽度检测目标宽度
	UINT16	lineNum;				// 线的数量
	UINT8	detectDirect;			// 宽度检测方向
	UINT8	sensiWidthDetect;		// 宽度检测灵敏度
	UINT16	detectLeftLine;			// 宽度检测左线位置
	UINT16	detectRightLine;		// 宽度检测右线位置

	UINT16	widthEdge[2];			// 两条边的横坐标

	UINT8	ifDetetctSuccess;		// 是否检测到合法的宽度
	UINT8	ifToolSuccess;			// 参数宽度与检测到的宽度是否在参数灵敏度规定的范围内。100*(1-abs(dp-dd)/dp) > sensitivity。
	INT32	resultLen;				// 检测到的宽度大小
};

/* 直径工具结果结构体 */
struct IVSCircleResult
{
	INT32	toolId;					// ID类型待确认。
	INT8	toolName[32];			// 工具名称
	INT16	toolScore;				// 工具结果评分
	INT16	toolIsOk;				// 工具结果是否OK
	INT32	toolType;				// 工具类型

	INT16	searchCircleX;			// 搜索圆的圆心横坐标
	INT16	searchCircleY;			// 搜索圆的圆心纵坐标
	UINT16	searchCircleRadius;		// 搜索圆的圆半径
	UINT16  circleNum;				// 检测出来的圆的数量
	INT16	circleX;				// 检测到的圆的圆心横坐标
	INT16	circleY;				// 检测到的圆的圆心纵坐标
	UINT16	circleRadius;			// 检测到的圆的圆半径
	INT16	align;					// 对齐填充
};

//矩形结构体，坐标起点为左上角点（iX0,iY0），逆时针转依次为（iX1,iY1）（iX2,iY2）和（iX3,iY3）
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

//圆结构体
struct IVSCircle {
	//圆心坐标（iX，iY）
	INT16 x;
	INT16 y;
	UINT16 radius;//圆的半径
};

/***
 @brief: 枚举工具类型
 ***/
enum TOOL_TYPE {
	CONTOUR_TOOL = 0,
	COLOR_TOOL = 1,
	WIDTH_TOOL = 2,
	CIRCLE_TOOL = 3
};

// 检测区域形状
#define REGION_SHAPE_CIRC 0
#define REGION_SHAPE_RECT 1







#endif // IVSPARAMETER_H
