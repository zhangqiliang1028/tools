#ifndef IVSVARTYPES_H
#define IVSVARTYPES_H

//用于填写toolType的类型
#define UNDIFINE        0X00
#define CONTOURTYPE     0X01
#define COLORTYPE       0X02
#define WIDTHTYPE       0X03
#define CIRCLETYPE      0X04
//新宏定义用于区分正常轮廓预处理结果上报，和轮廓金字塔精度位图上报
#define PYRBITMAP       0X06

//gCurrentID的赋值宏，当gCurrentID被赋值为0~16的时候，代表处理线程只处理对应id的工具，或者gCurrentID被赋值为以下两个值
#define ALLTOOL         0x7FFFFFFF  //代表int类型可表示的最大值，处理线程处理所有工具
#define NOTOOL          0xFFFFFFFF  //代表-1，处理线程不处理任何工具

#define MEMORY_SIZE_PER_CELL    (18*1024*1024)  //每个内存单元的大小
#define MEMORY_CELL_NUM         10              //初始化时一次性创建的内存单元数量

using UINT8 = unsigned char;
using INT8 = signed char;
using UINT16 = unsigned short;
using INT16 = short;
using UINT32 = unsigned int;
using INT32 = int;
using ADDR32 = void*;

/***
 @brief: 工具间输出逻辑类型
 ***/
enum LOGIC_TYPE {
	WITH = 0,
	OR = 1,
	NOT = 2
};

/***
 @brief: 触发类型
 ***/
enum TRIGGER_MODE {
	INTERNAL_TRIGGER = 1,       //主动采集
	OUT_TRIGGER = 2             //被动采集
};
/***
 @brief: 组当前状态
 ***/
enum GROUP_STATE {
	RUNNING = 0,
	SETTING = 1,
	STARTING = 2
};
//预处理消息负载，位置修正工具
typedef struct CORRECTIONTOOLMSG {

}CorrectionToolMsg;
//预处理消息负载，轮廓工具
typedef struct CONTOURTOOLMSG {
	//预处理的时候，用于传递坐标点
	int x;
	int y;
}ContourToolMsg;
//预处理消息负载，颜色工具
typedef struct COLORTOOLMSG {

}ColorToolMsg;
//预处理消息负载，宽度工具
typedef struct WIDTHTOOLMSG {

}WidthToolMsg;
//预处理消息负载，直径工具
typedef struct CIRCLETOOLMSG {

}CircleToolMsg;

//原始图片存储格式，用于消息结构体
enum ENUM_RAW_DATA_FORMAT {
	YUV420SP = 0,
	YUV422SP = 1
};
//编码图片存储格式，用于消息结构体
enum ENUM_RGB_DATA_FORMAT {

};

//模板创建的返回状态码，用于处理线程到主线程的消息结构体
enum ENUM_RETURN_CODE {
	UNDEFINE = 0,           //未定义
	CREATE_SUCCEED = 1,     //模板创建成功
	CREATE_FAILED = 2,      //模板创建失败
	OUT_OF_RANGE = 3
};

//用于填充预处理的步骤
enum ENUM_STAGE {
	STEP1 = 0,          //预处理的第一阶段，用于四个工具的预处理
	STEP2 = 1,           //预处理的第二阶段，特指用轮廓工具创建模板
	STEP3 = 2             //特指轮廓工具生成金字塔位图
};

//保存每个工具的指针和型别，用于处理线程到主线程的消息结构体
typedef struct IVSToolResult {
	ADDR32              pToolResultAddr;        //存储工具处理结果的指针
	UINT32      		toolType;            	//地址指向的工具类型
}IVSToolResult;

#endif // IVSVARTYPES_H
