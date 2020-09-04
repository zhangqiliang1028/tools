#include <stdio.h>
#include <string>
#include <fstream>
#include <unistd.h>
#include <dlfcn.h>
#include "ToolBase.h"

/*
ͨ����̬��������̬��ȡ���߶���
*/
int getToolBylib(std::string libName,ToolBase* &preTool,void* &lib_tool)
{
      ToolBase *(*pToolObj)(void);
      //dlerror();
     std::string file="./lib"+libName+".so";
     // std::cout<<file.c_str()<<std::endl;
      lib_tool = dlopen(file.c_str(),RTLD_LAZY);
      if(NULL == lib_tool)
      {
            //std::cout<<"load library"<<libName.c_str()<<" error"<<std::endl<<"lErrmsg:"<<dlerror()<<std::endl;
            return -1;
      }
      pToolObj = (ToolBase *(*)(void))dlsym(lib_tool,"CreateTool");
      const char *dlmsg = dlerror();
      if(NULL != dlmsg)
      {
           //std::cout<<"get class"<<libName.c_str()<<"error"<<std::endl<<"Errmsg:"<<dlmsg<<std::endl;
           dlclose(lib_tool);
           return -1;
      }
      preTool = (*pToolObj)();
      //std::cout<<"create tool down"<<std::endl;
      return 0;
}
int main()
{
        Json::Value preJson; 
        Json::Value json;
        //Json::Value json1;
        std::vector<Json::Value> jsons;

        Json::Reader reader;
        std::string inputsStr,onelineStr;
        std::ifstream preifs,ifs1,ifs2,ifs3,ifs4,ifs5;
        preifs.open("preTool1.txt");
        while(getline(preifs,onelineStr))
        {
            inputsStr+=onelineStr;
        }
        reader.parse(inputsStr, preJson);
        //std::cout<<preJson["libName"].asString()<<std::endl;
        inputsStr="";
        //onelineStr="";
        ifs1.open("bottleTool1.txt");
        while(getline(ifs1,onelineStr))
        {
            inputsStr+=onelineStr;
        }
        reader.parse(inputsStr, json);
        std::cout<<json["libName"].asString()<<std::endl;
        jsons.push_back(json);
         
        inputsStr="";
        ifs2.open("bottleTool9.txt");
        while(getline(ifs2,onelineStr))
        {
            inputsStr+=onelineStr;
        }
        reader.parse(inputsStr, json);
        std::cout<<json["libName"].asString()<<std::endl;
        jsons.push_back(json);

        /*inputsStr="";
        ifs3.open("bottleTool6.txt");
        while(getline(ifs3,onelineStr))
        {
            inputsStr+=onelineStr;
        }
        reader.parse(inputsStr, json);
        std::cout<<json["libName"].asString()<<std::endl;
        jsons.push_back(json);

        inputsStr="";
        ifs4.open("bottleTool4.txt");
        while(getline(ifs4,onelineStr))
        {
            inputsStr+=onelineStr;
        }
        reader.parse(inputsStr, json);
        std::string stdjson=json.toStyledString();
        std::cout<<stdjson.c_str()<<std::endl;
        Json::Value jsont;
        reader.parse(stdjson, jsont);
        jsons.push_back(jsont);*/

        //���嶯̬��ָ��
        void *lib_tool=NULL;
        
        std::vector<Json::Value> preToolsResults = {};
        std::vector<Json::Value> toolsResults = {};
        
        //Ԥ�������б�
        //��etcd��ȡ��Json�ļ���һ������Ԫjson�ڴ˽����ɶ�����߲���json
        std::vector<std::string> preToolsList;
        preToolsList.push_back("preTool1");

        std::vector<ToolBase*> preTools;
        for(int i = 0;i < preToolsList.size();i++)
        {
            ToolBase* preTool=NULL;
            getToolBylib(preJson["libName"].asString(),preTool,lib_tool);
            preTool->initTool(preJson);
	    preTools.push_back(preTool);
        }
          
        //Ԥ����
        std::cout << "start preprocess" << std::endl;
        cv::Mat sample_image = cv::imread("template.bmp", 0);
        //cv::imshow("qq1",sample_image);
        //cv::waitKey(0);
        cv::Mat preMat=sample_image.clone(); 
        std::vector<cv::Mat> preImages; 
        preImages.push_back(preMat);
	for (int i = 0; i < preTools.size(); i++)
	{ 
		//����Ԥ������                                                      
		int runflag=preTools[i]->run(preImages[i],preMat);
                preImages.push_back(preMat);
                //cv::imshow("qq",preMat);
                //cv::waitKey(0);
	}
        std::cout << "preprocess down" << std::endl;
       
        //��Ԥ�������б�         
	//��etcd��ȡ��Json�ļ���һ������Ԫjson�ڴ˽����ɶ�����߲���json
	//�������߲���json������Ĺ��캯���н���������ʼ�������
	//���ݹ����б�json��������������
	std::vector<ToolBase*> tools;
        for(int i=0;i<jsons.size();i++)
        {
            ToolBase* tool=NULL;
            getToolBylib(jsons[i]["libName"].asString(),tool,lib_tool);
            tool->initTool(jsons[i]);
	    tools.push_back(tool);

        }
	
	//˳��ִ�й���run����
	std::cout << "start run" << std::endl;

        std::vector<int> toolsID;
	for (int i = 0; i < tools.size(); i++)
	{ 
                toolsID.push_back(i+1);
                Json::Value toolResult;
                std::vector<Json::Value> relyOnResults;
                std::vector<int> nums;
                int getflag=tools[i]->getRelyOnParam(nums);
                if(getflag)
                {   
                    for(int j = 0;j < nums.size(); j++)
                    {
                       std::cout<<"rely:"<<nums[j]<<std::endl;
                       //��ʼ�������Ĳ������
                       for(int k = 0;k < i; k++)
                       {   
                           Json::Value rResult;
                           if(nums[j]==toolsID[k])
                           {
                              relyOnResults.push_back(toolsResults[k]);                                    
                              break;
                           }  
                       } 
                       //std::cout<<relyOnResults[j]["linePoint1Y"].asString()<<std::endl;
                    }
                    
                }
		//���й���
		std::cout<<i<<" tool is running"<<std::endl;
		int runflag=tools[i]->run(preMat,relyOnResults,toolResult);
		std::cout<<i<<" tool is running"<<std::endl;
                std::cout<<i<<":ToolOk:"<<toolResult.toStyledString().c_str()<<std::endl;
                if(runflag)
                {
		   //����ǰ���߽������ȫ�ֽ��
		   toolsResults.push_back(toolResult);
                }
                else
                   return -1;
	}
        
        //�����й��ߵ����н�������ͻ��ˣ�Ȼ����� 
        preImages={};
        toolsResults={}; 
	//etcd������ģ������仯(status=0)���ص����²�������
        //tools[i]->updateToolParam(json);
	//etcd������״̬�����仯(status=1)���ص����²���������������
         //�ɼ��̷߳���һ֡ͼ�񴥷���ѭ���������ļ���ͼ������        
	//1
	std::cout<<"-----------------------------------------------"<<std::endl;      
	sample_image = cv::imread("ok1.bmp", 0);
        std::cout << "start preprocess" << std::endl;
        for (int i = 0; i < preTools.size(); i++) { //����Ԥ������
		 int runflag=preTools[i]->run(sample_image,preMat);
                //cv::imshow("qq2",preMat); }
	        std::cout << "preprocess down" << std::endl;
	}
	for (int i = 0; i < tools.size(); i++) { 
		std::vector<int> nums;
                int getflag=tools[i]->getRelyOnParam(nums);
                std::vector<Json::Value> relyOnResults;
                for(int j = 0;j < nums.size(); j++)
                {
                   //��ʼ�������Ĳ������
                   for(int k = 0;k < i; k++)
                   {
                       Json::Value rResult;
                       if(nums[j]==toolsID[k])                    			
                       {
				relyOnResults.push_back(toolsResults[k]);
				break;
                       }
                    }
                }
		//���й���
		Json::Value toolResult;
		 int runflag=tools[i]->run(preMat,relyOnResults,toolResult);
		std::cout<<toolResult["RunTimes"].asInt()<<std::endl;
		std::cout<<"*****************"<<toolResult<<std::endl;
                if(runflag)
                {
		    //����ǰ���߽������ȫ�ֽ��
		    toolsResults.push_back(toolResult);
                }
	} 

        //�ͷ�ָ��       
        for (int i = 0; i < preTools.size(); i++)
	{ 
		delete preTools[i];
	}
        for (int i = 0; i < tools.size(); i++)
	{ 
                //�ͷŵ�ǰ�����ڴ�
		tools[i]->emptyTool();
		delete tools[i];

	}
        dlclose(lib_tool);                                                                                  
        
	cv::waitKey(0);
	return 0;
}
