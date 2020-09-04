#ifndef IVSVARTYPES_H
#define IVSVARTYPES_H

//������дtoolType������
#define UNDIFINE        0X00
#define CONTOURTYPE     0X01
#define COLORTYPE       0X02
#define WIDTHTYPE       0X03
#define CIRCLETYPE      0X04
//�º궨������������������Ԥ�������ϱ�������������������λͼ�ϱ�
#define PYRBITMAP       0X06

//gCurrentID�ĸ�ֵ�꣬��gCurrentID����ֵΪ0~16��ʱ�򣬴������߳�ֻ�����Ӧid�Ĺ��ߣ�����gCurrentID����ֵΪ��������ֵ
#define ALLTOOL         0x7FFFFFFF  //����int���Ϳɱ�ʾ�����ֵ�������̴߳������й���
#define NOTOOL          0xFFFFFFFF  //����-1�������̲߳������κι���

#define MEMORY_SIZE_PER_CELL    (18*1024*1024)  //ÿ���ڴ浥Ԫ�Ĵ�С
#define MEMORY_CELL_NUM         10              //��ʼ��ʱһ���Դ������ڴ浥Ԫ����

using UINT8 = unsigned char;
using INT8 = signed char;
using UINT16 = unsigned short;
using INT16 = short;
using UINT32 = unsigned int;
using INT32 = int;
using ADDR32 = void*;

/***
 @brief: ���߼�����߼�����
 ***/
enum LOGIC_TYPE {
	WITH = 0,
	OR = 1,
	NOT = 2
};

/***
 @brief: ��������
 ***/
enum TRIGGER_MODE {
	INTERNAL_TRIGGER = 1,       //�����ɼ�
	OUT_TRIGGER = 2             //�����ɼ�
};
/***
 @brief: �鵱ǰ״̬
 ***/
enum GROUP_STATE {
	RUNNING = 0,
	SETTING = 1,
	STARTING = 2
};
//Ԥ������Ϣ���أ�λ����������
typedef struct CORRECTIONTOOLMSG {

}CorrectionToolMsg;
//Ԥ������Ϣ���أ���������
typedef struct CONTOURTOOLMSG {
	//Ԥ�����ʱ�����ڴ��������
	int x;
	int y;
}ContourToolMsg;
//Ԥ������Ϣ���أ���ɫ����
typedef struct COLORTOOLMSG {

}ColorToolMsg;
//Ԥ������Ϣ���أ���ȹ���
typedef struct WIDTHTOOLMSG {

}WidthToolMsg;
//Ԥ������Ϣ���أ�ֱ������
typedef struct CIRCLETOOLMSG {

}CircleToolMsg;

//ԭʼͼƬ�洢��ʽ��������Ϣ�ṹ��
enum ENUM_RAW_DATA_FORMAT {
	YUV420SP = 0,
	YUV422SP = 1
};
//����ͼƬ�洢��ʽ��������Ϣ�ṹ��
enum ENUM_RGB_DATA_FORMAT {

};

//ģ�崴���ķ���״̬�룬���ڴ����̵߳����̵߳���Ϣ�ṹ��
enum ENUM_RETURN_CODE {
	UNDEFINE = 0,           //δ����
	CREATE_SUCCEED = 1,     //ģ�崴���ɹ�
	CREATE_FAILED = 2,      //ģ�崴��ʧ��
	OUT_OF_RANGE = 3
};

//�������Ԥ����Ĳ���
enum ENUM_STAGE {
	STEP1 = 0,          //Ԥ����ĵ�һ�׶Σ������ĸ����ߵ�Ԥ����
	STEP2 = 1,           //Ԥ����ĵڶ��׶Σ���ָ���������ߴ���ģ��
	STEP3 = 2             //��ָ�����������ɽ�����λͼ
};

//����ÿ�����ߵ�ָ����ͱ����ڴ����̵߳����̵߳���Ϣ�ṹ��
typedef struct IVSToolResult {
	ADDR32              pToolResultAddr;        //�洢���ߴ�������ָ��
	UINT32      		toolType;            	//��ַָ��Ĺ�������
}IVSToolResult;

#endif // IVSVARTYPES_H
