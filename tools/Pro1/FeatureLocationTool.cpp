#include "FeatureLocationTool.h"
#include "ivs_algorithm_utils.h"
#include <string>
#include <exception>

ToolBase *CreateTool(void)
{
    return (new FeatureLocationTool());
}

int FeatureLocationTool::initTool(Json::Value json)
{
        std::cout<<"FeatureLocation tool init start"<<std::endl;
	//初始化参数
	//template_image = cv::imread("newvenc1.jpg", 0);
        if(json["TemplateRect"].isNull()||json["searchRect"].isNull()||json["algoStrategy"].isNull()||json["angleRange"].isNull()||json["sensiLowThreshold"].isNull()||json["sensiTopThreshold"].isNull()||json["scoreLowThreshold"].isNull()||json["scoreLowThreshold"].isNull()||json["scoreTopThreshold"].isNull()||json["sampleImageWidth"].isNull()||json["sampleImageHeight"].isNull())
        {
           std::cout<<"Error:FeatureLocation tool init error:NULL!"<<std::endl;
           //throw std::exception();
           return 0;
        }
        else
        {
	   toolParameter.regionShape = (UINT8)1;
	   toolParameter.detectRectX0 = (INT16)json["TemplateRect"]["TemplatePosition"]["detectRectX0"].asInt();
	   toolParameter.detectRectY0 = (INT16)json["TemplateRect"]["TemplatePosition"]["detectRectY0"].asInt();
	   toolParameter.detectRectX1 = (INT16)json["TemplateRect"]["TemplatePosition"]["detectRectX1"].asInt();
	   toolParameter.detectRectY1 = (INT16)json["TemplateRect"]["TemplatePosition"]["detectRectY1"].asInt();
	   toolParameter.detectRectX2 = (INT16)json["TemplateRect"]["TemplatePosition"]["detectRectX2"].asInt();
	   toolParameter.detectRectY2 = (INT16)json["TemplateRect"]["TemplatePosition"]["detectRectY2"].asInt();
	   toolParameter.detectRectX3 = (INT16)json["TemplateRect"]["TemplatePosition"]["detectRectX3"].asInt();
	   toolParameter.detectRectY3 = (INT16)json["TemplateRect"]["TemplatePosition"]["detectRectY3"].asInt();
	   toolParameter.detectCircleX = (INT16)1;
	   toolParameter.detectCircleY = (INT16)1;
	   toolParameter.detectCircleRadius = (UINT16)1;
	   toolParameter.searchRectX0= (INT16)json["searchRect"]["searchRectX0"].asInt();
	   toolParameter.searchRectY0 = (INT16)json["searchRect"]["searchRectY0"].asInt();
	   toolParameter.searchRectX1 = (INT16)json["searchRect"]["searchRectX1"].asInt();
	   toolParameter.searchRectY1= (INT16)json["searchRect"]["searchRectY1"].asInt();
	   toolParameter.searchRectX2 = (INT16)json["searchRect"]["searchRectX2"].asInt();
	   toolParameter.searchRectY2 = (INT16)json["searchRect"]["searchRectY2"].asInt();
	   toolParameter.searchRectX3 = (INT16)json["searchRect"]["searchRectX3"].asInt();
	   toolParameter.searchRectY3 = (INT16)json["searchRect"]["searchRectY3"].asInt();
	   toolParameter.extRectX = (INT16)json["TemplateRect"]["extRectX"].asInt();
	   toolParameter.extRectY = (INT16)json["TemplateRect"]["extRectY"].asInt();
	   toolParameter.extRectWidth= (UINT16)json["TemplateRect"]["extRectWidth"].asInt();
	   toolParameter.extRectHeight = (UINT16)json["TemplateRect"]["extRectHeight"].asInt();
	   toolParameter.algoStrategy = (UINT8)json["algoStrategy"].asInt();
	   toolParameter.angleRange = (UINT8)json["angleRange"].asInt();
	   toolParameter.sensiLowThreshold =(UINT8)json["sensiLowThreshold"].asInt();
	   toolParameter.sensiTopThreshold = (UINT8)json["sensiTopThreshold"].asInt();
	   toolParameter.scoreLowThreshold = (UINT16)json["scoreLowThreshold"].asDouble();
	   toolParameter.scoreTopThreshold = (UINT16)json["scoreTopThreshold"].asInt();
           toolParameter.contourCandidateNum =(UINT8)30;
	   toolParameter.contourScoreThreshold = (UINT16)0.7;
           toolParameter.contourNumThreshold = (UINT16)0.6;
           toolParameter.contourNaborSize =(UINT8)1;
	   toolParameter.contourNaborAngle = (UINT8)2;
           toolParameter.minContourPyra =(UINT8)30;
	   toolParameter.maxNumPyramid = (UINT8)7;
           toolParameter.minNumPyramid =(UINT8)6;
	   toolParameter.highPrecisionNumPyramid = (UINT8)6;
           //给待测金字塔申请内存空间
           sampleImageWidth=json["sampleImageWidth"].asInt();
           sampleImageHeight=json["sampleImageHeight"].asInt();
           createUtility(contourUtility,sampleImageWidth, sampleImageHeight);
           //
           templateInfo.templateIsOK=0;
           std::cout<<"FeatureLocation tool init down"<<std::endl;
           return 1;
        }

        
}
void FeatureLocationTool::updateTool(Json::Value json) 
{

}


int FeatureLocationTool::run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result)
{
        TimeTracker time;
        time.start();
        try
        {
       
             if(src.channels() != 1)
                cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
             //清空之前测试图像的工具结果
             cannyPyramid = {};
	     candidates = {};
             //std::cout << (int)toolParameter.angleRange << std::endl;
	     //如果模板信息为空
	     if (!templateInfo.templateIsOK)
	     {
                  template_image=src.clone();
	          std::cout << "start create temple" << std::endl;
	          //计算单角度轮廓信息
                  //UINT8 *contours=NULL;
	          //ivs_get_contours(template_image, toolParameter, contours);
	          //计算模板信息,填充templateParam
	          ivs_create_template();
	          //print_tpl(templateInfo.tpls[3][0]);
                  //print_tpl(templateInfo.tpls[0][10]);
                  std::cout << templateInfo.tpls[0].size() << std::endl;
	          std::cout << "create temple stop" << std::endl;      
  
	     }
	     //std::cout << templateInfo.tpls.size()<<":"<< status << std::endl;
	
	     //匹配待测图像
	     std::cout << "start match" << std::endl;
        
	     //imshow("sample", src);
	     computeUtility(contourUtility,src);
	     initTask(contourUtility);
	     doTemplateMatch(contourUtility);
	}
        catch (std::exception e)
        {
            std::cout<<e.what()<<" :feature location fail!"<< std::endl;   //捕获异常，然后程序结束
            return 0;
        }
        time.stop();
        int times=time.duration();
	CandidateResult finalResult;
	if (!candidates[templateInfo.pyramidLevelNum - 1].empty())
	    finalResult = candidates[templateInfo.pyramidLevelNum - 1].top();
	//std::cout << candidates[templateInfo.pyramidLevelNum - 1].size() << std::endl;
	UINT16 Theta_for_cal = (finalResult.angleIndex - toolParameter.angleRange + 360) % 360;
	
	std::cout << "match stop" << std::endl;
        int toolIsOk = 0; 
        int score= finalResult.score*100;
        if(score>=toolParameter.scoreLowThreshold)
           toolIsOk = 1;
        
	//将匹配结果以Json形式存储
        std::string resultStr="{\"ToolIsOk\":"+std::to_string(toolIsOk)+",\"ResultShape\":"+std::to_string((int)toolParameter.regionShape)+",\"Score\":"+std::to_string(score)+",\"Angle\":"+std::to_string((int)Theta_for_cal)+",\"CenterX\":"+std::to_string(finalResult.positionX)+",\"CenterY\":"+std::to_string(finalResult.positionY)+",\"Width\":"+std::to_string(templateInfo.searchRectWidth[0])+",\"RunTimes\":"+std::to_string(times)+"}";
        std::cout<<resultStr.c_str()<<std::endl; 
        Json::Reader reader;
        reader.parse(resultStr, result);
        //result["test"]="ok";
	return 1;
}
int FeatureLocationTool::emptyTool()
{
        //释放待测金字塔内存
	freeUtility(contourUtility);
        return 1;
};
int FeatureLocationTool::unpackTemplate(char* buf)
{

	std::size_t index = 0;

	templateInfo.pyramidLevelNum = *((UINT8 *)&buf[index]);
	index += sizeof(templateInfo.pyramidLevelNum);

	for (int i = 0; i < templateInfo.pyramidLevelNum; ++i) {
		float angle = *((float*)&buf[index]);
		index += sizeof(float);
		templateInfo.searchAngelStep.push_back(angle);
	}

	for (int i = 0; i < templateInfo.pyramidLevelNum; ++i) {
		UINT16 width = *((UINT16*)&buf[index]);
		index += sizeof(UINT16);
		templateInfo.searchRectWidth.push_back(width);
	}

	// 拷贝模板数据
	for (int i = 0; i < templateInfo.pyramidLevelNum; ++i) {
		std::vector<IVSTemplateSubStruct> tpl_arr;

		UINT16 tpl_size = *((UINT16*)&buf[index]);
		index += sizeof(UINT16);

		for (int j = 0; j < tpl_size; ++j) {
			IVSTemplateSubStruct tpl;
			tpl.modelDefined = *((UINT8*)&buf[index]);
			index += sizeof(tpl.modelDefined);

			tpl.noOfCordinates = *((UINT32*)&buf[index]);
			index += sizeof(tpl.noOfCordinates);

			tpl.modelHeight = *((UINT16*)&buf[index]);
			index += sizeof(tpl.modelHeight);

			tpl.modelWidth = *((UINT16*)&buf[index]);
			index += sizeof(tpl.modelWidth);

			tpl.centerOfGravity.x = *((INT16*)&buf[index]);
			index += sizeof(short);

			tpl.centerOfGravity.y = *((INT16*)&buf[index]);
			index += sizeof(short);

			// 拷贝特征数据
			for (std::size_t t = 0; t < tpl.noOfCordinates; ++t) {
				cv::Point point;
				point = *((cv::Point*)&buf[index]);
				index += sizeof(cv::Point);
				tpl.cordinates.push_back(point);
			}

			for (std::size_t t = 0; t < tpl.noOfCordinates; ++t) {
				float edgeX = *((float *)&buf[index]);
				index += sizeof(float);
				tpl.edgeDerivativeX.push_back(edgeX);
			}

			for (std::size_t t = 0; t < tpl.noOfCordinates; ++t) {
				float edgeY = *((float*)&buf[index]);
				index += sizeof(float);
				tpl.edgeDerivativeY.push_back(edgeY);
			}
			tpl_arr.push_back(tpl);
		}
		templateInfo.tpls.push_back(tpl_arr);
	}

	return 0;
}

void FeatureLocationTool::print_tpl(const IVSTemplateSubStruct &tpl) {
	cv::Mat tmp(tpl.modelHeight, tpl.modelWidth, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < tpl.noOfCordinates; ++i) {
		tmp.at<uchar>(tpl.cordinates[i].y + tpl.centerOfGravity.y, tpl.cordinates[i].x + tpl.centerOfGravity.x) = 255;
	}
	cv::imshow("tmp", tmp);
	//cv::waitKey(0);
}

int FeatureLocationTool::initTask(ContourUtility &contourUtility) {
	//TimeTracker time;
	//time.start();
    /*
	// 根据模板文件路径，解析模板文件
	FILE *fp = fopen((const char*)toolParameter.templatePath, "rb");
	fseek(fp, 0L, SEEK_END);
	size_t templateSize = ftell(fp);
	std::cout << "templateSize is" << templateSize << std::endl;
	rewind(fp);
	char *buf = (char *)malloc(sizeof(char)*templateSize);
	fread(buf, sizeof(UINT8), templateSize, fp);

	time.stop();
	double spendtime;

	spendtime = time.duration();
	std::cout << spendtime * 1000 << "ms" << std::endl;

	// 解析数据
	unpackTemplate(buf);

	// 释放资源
	fclose(fp);
	free(buf);
	*/
	/* 设置每层金字塔的搜索区域 */
	//此部分只用到顶层的搜索区域，在加入搜索框，在4层搜索时需要用到，以加快搜索速度

	// 考虑到之后局部轮廓可能也存在矫正的问题，这里的搜索区域需要为参数中搜索范围的外接框
	INT16 leftTopX = IVS_MIN<INT16>(toolParameter.searchRectX0, toolParameter.searchRectX1,
		toolParameter.searchRectX2, toolParameter.searchRectX3);
	INT16 leftTopY = IVS_MIN<INT16>(toolParameter.searchRectY0, toolParameter.searchRectY1,
		toolParameter.searchRectY2, toolParameter.searchRectY3);

	INT16 rightBottomX = IVS_MAX<INT16>(toolParameter.searchRectX0, toolParameter.searchRectX1,
		toolParameter.searchRectX2, toolParameter.searchRectX3);

	INT16 rightBottomY = IVS_MAX<INT16>(toolParameter.searchRectY0, toolParameter.searchRectY1,
		toolParameter.searchRectY2, toolParameter.searchRectY3);

	// 填充第一层搜索范围
	cv::Rect rectTmp;
	rectTmp.x = leftTopX;
	rectTmp.y = leftTopY;
	rectTmp.width = rightBottomX - leftTopX;
	rectTmp.height = rightBottomY - leftTopY;
	searchRect.push_back(rectTmp);

	for (int i = 1; i < templateInfo.pyramidLevelNum; ++i) {

		// 填充每一层的搜索范围
		rectTmp.x = searchRect[i - 1].x >> 1;
		rectTmp.y = searchRect[i - 1].y >> 1;
		rectTmp.width = searchRect[i - 1].width >> 1;
		rectTmp.height = searchRect[i - 1].height >> 1;
		searchRect.push_back(rectTmp);
	}

	// 打印查看输出的模板文件是否正确
	//for (int i = 0; i < templateInfo.tpls.size(); ++i) {
	//	for (int j = 0; j < templateInfo.tpls[i].size(); ++j) {
	//		print_tpl(templateInfo.tpls[i][j]);
	//	}
	//}

	// 处理待测图像的图像金字塔(其他部分都在contourUtility进行了处理，这里只处理canny图像)
	int lowThreshold = toolParameter.sensiLowThreshold;
	int topThreshold = toolParameter.sensiTopThreshold;

	lowThreshold = lowThreshold > 255 ? 255 : lowThreshold;
	topThreshold = topThreshold > 255 ? 255 : topThreshold;
        std::cout<<searchRect[0].x<<":"<<searchRect[0].y<<std::endl;
	for (int i = 0; i < templateInfo.pyramidLevelNum; ++i) {
		// 创建canny图像，以及相应的膨胀图像，积分图图像(只处理搜索范围的图像)
		cv::Mat cannyMat, cannyDilateMat, integralMat;
		cv::Canny(contourUtility.searchRegion[i](searchRect[i]), cannyMat, lowThreshold, topThreshold);
		Dilation(cannyMat, cannyDilateMat, 2);
		cannyPyramid.push_back(cannyMat);
		//cannyDilatePyramid.push_back(cannyDilateMat);

		//cv::imshow("cannyMat", cannyMat);
		//cv::imshow("cannyDilateMat", cannyDilateMat);
		//cv::waitKey(0);
		// todo 积分图待续
	}

	// todo 处理一下次层膨胀然后降采样到顶层的操作


	return 0;
}

cv::Point cal_coor_by_ratio(cv::Point start_point, cv::Point end_point, double ratio) {
	cv::Point tmp_point;
	tmp_point.x = (end_point.x - start_point.x)*ratio + start_point.x;
	tmp_point.y = (end_point.y - start_point.y)*ratio + start_point.y;
	return tmp_point;
}

int FeatureLocationTool::doTask(ContourUtility &contourUtility, UINT8 *contourResultAndBitmap, size_t *bufSize) {
	// 判读参数是否正确
	if (contourResultAndBitmap == NULL) {
		printf("error: no allocate\n");
		return -1;
	}

	// 由于确定高速度策略也做到最底层，为了改动较少，只修改这三处，其一
	//INT8 com_level = palg_handle->contour_param->algo_strategy;
	INT8 com_level = 0;
	INT8 com_score = 20;

	IVSContourResult ivsContourResult;
	// 初始化结果结构体
	//ivsContourResult.toolId;
	//ivsContourResult.toolName;
	//ivsContourResult.toolType;
	ivsContourResult.toolIsOk = 0;
	ivsContourResult.toolScore = 0;

	ivsContourResult.regionShape = toolParameter.regionShape;

	ivsContourResult.detectCircleX = toolParameter.detectCircleX;
	ivsContourResult.detectCircleY = toolParameter.detectCircleY;
	ivsContourResult.detectCircleRadius = toolParameter.detectCircleRadius;

	ivsContourResult.detectRectX0 = toolParameter.detectRectX0;
	ivsContourResult.detectRectY0 = toolParameter.detectRectY0;
	ivsContourResult.detectRectX1 = toolParameter.detectRectX1;
	ivsContourResult.detectRectY1 = toolParameter.detectRectY1;
	ivsContourResult.detectRectX2 = toolParameter.detectRectX2;
	ivsContourResult.detectRectY2 = toolParameter.detectRectY2;
	ivsContourResult.detectRectX3 = toolParameter.detectRectX3;
	ivsContourResult.detectRectY3 = toolParameter.detectRectY3;

	ivsContourResult.searchRectX0 = toolParameter.searchRectX0;
	ivsContourResult.searchRectY0 = toolParameter.searchRectY0;
	ivsContourResult.searchRectX1 = toolParameter.searchRectX1;
	ivsContourResult.searchRectY1 = toolParameter.searchRectY1;
	ivsContourResult.searchRectX2 = toolParameter.searchRectX2;
	ivsContourResult.searchRectY2 = toolParameter.searchRectY2;
	ivsContourResult.searchRectX3 = toolParameter.searchRectX3;
	ivsContourResult.searchRectY3 = toolParameter.searchRectY3;

	ivsContourResult.bitmapSize = 0;
	ivsContourResult.extRectX = toolParameter.extRectX;
	ivsContourResult.extRectY = toolParameter.extRectY;
	ivsContourResult.extRectHeight = toolParameter.extRectHeight;
	ivsContourResult.extRectWidth = toolParameter.extRectWidth;

	//    qDebug() << "toolParameter.extRectX" << toolParameter.extRectX;
	//    qDebug() << "toolParameter.extRectY" << toolParameter.extRectY;
	//    qDebug() << "toolParameter.extRectHeight" << toolParameter.extRectHeight;
	//    qDebug() << "toolParameter.extRectWidth" << toolParameter.extRectWidth;

		// 进行匹配
	doTemplateMatch(contourUtility);

	CandidateResult finalResult;
	if (!candidates[templateInfo.pyramidLevelNum - 1].empty()) {
		finalResult = candidates[templateInfo.pyramidLevelNum - 1].top();

		// 评分策略：（计算得出的评分*（搜索框内的点数/模板总点数））
		cv::Point left_top_point, right_bottom_point;
		left_top_point.x = searchRect[0].x;
		left_top_point.y = searchRect[0].y;
		right_bottom_point.x = searchRect[0].x + searchRect[0].width;
		right_bottom_point.y = searchRect[0].y + searchRect[0].height;

		UINT32 point_sum = 0;
		for (UINT32 i = 0; i < templateInfo.tpls[0][finalResult.angleIndex].noOfCordinates; ++i) {
			cv::Point p = templateInfo.tpls[0][finalResult.angleIndex].cordinates[i];
			// 判断该点是否在搜索范围内
			p.x += finalResult.positionX << com_level;
			p.y += finalResult.positionY << com_level;
			if (p.x >= left_top_point.x && p.x <= right_bottom_point.x && p.y >= left_top_point.y && p.y <= right_bottom_point.y) {
				point_sum++;
			}
		}
		double percent = (double)point_sum / templateInfo.tpls[0][finalResult.angleIndex].noOfCordinates;
		printf("the num of points in detection area is %d, the num of model points is %d, the percent is %lf\n", point_sum,
			templateInfo.tpls[0][finalResult.angleIndex].noOfCordinates, percent);

		// 将更新的评分结果填充到结构体中(百分制)
		ivsContourResult.toolScore = roundf(finalResult.score * percent * 100);
		ivsContourResult.toolIsOk = (ivsContourResult.toolScore >= toolParameter.scoreLowThreshold);
	}
	else {
		// 将更新的评分结果填充到结构体中(百分制)
		ivsContourResult.toolScore = 0;
		ivsContourResult.toolIsOk = 0;
	}


	if (ivsContourResult.toolScore >= com_score) {
		// extRect外接框结果填充
		UINT16 extWidth = templateInfo.tpls[0][finalResult.angleIndex].modelWidth;
		UINT16 extHeight = templateInfo.tpls[0][finalResult.angleIndex].modelHeight;
		INT16 extX = finalResult.positionX - templateInfo.tpls[0][finalResult.angleIndex].centerOfGravity.x;
		INT16 extY = finalResult.positionY - templateInfo.tpls[0][finalResult.angleIndex].centerOfGravity.y;

		ivsContourResult.extRectX = extX;
		ivsContourResult.extRectY = extY;
		ivsContourResult.extRectWidth = extWidth;
		ivsContourResult.extRectHeight = extHeight;

		// detect检测框结果填充
		if (ivsContourResult.regionShape == REGION_SHAPE_RECT) {
			UINT16 minx = ivsContourResult.extRectX;
			UINT16 miny = ivsContourResult.extRectY;
			UINT16 maxx = minx + ivsContourResult.extRectWidth;
			UINT16 maxy = miny + ivsContourResult.extRectHeight;

			UINT16 height = templateInfo.tpls[0][toolParameter.angleRange].modelHeight;
			UINT16 width = templateInfo.tpls[0][toolParameter.angleRange].modelWidth;

			UINT16 Theta_for_cal = (finalResult.angleIndex - toolParameter.angleRange + 360) % 360;
			double sin_theta = sin(Theta_for_cal*3.1415926535 / 180);

			// 处理在配置阶段非水平检测框显示外接框的问题

			cv::Point coor_point[4];
			cv::Point left_point, top_point;
			double ratiox = 0.0, ratioy = 0.0;
			coor_point[0].x = ivsContourResult.detectRectX0;
			coor_point[0].y = ivsContourResult.detectRectY0;
			coor_point[1].x = ivsContourResult.detectRectX1;
			coor_point[1].y = ivsContourResult.detectRectY1;
			coor_point[2].x = ivsContourResult.detectRectX2;
			coor_point[2].y = ivsContourResult.detectRectY2;
			coor_point[3].x = ivsContourResult.detectRectX3;
			coor_point[3].y = ivsContourResult.detectRectY3;

			int y_min_tmp = 10000;
			int x_min_tmp = 10000;
			int y_max_tmp = -10000;
			int x_max_tmp = -10000;

			for (int i = 0; i < 4; ++i) {
				if (y_min_tmp > coor_point[i].y) {
					top_point = coor_point[i];
					y_min_tmp = coor_point[i].y;
				}
				if (x_min_tmp > coor_point[i].x) {
					left_point = coor_point[i];
					x_min_tmp = coor_point[i].x;
				}
				if (y_max_tmp < coor_point[i].y) {
					y_max_tmp = coor_point[i].y;
				}
				if (x_max_tmp < coor_point[i].x) {
					x_max_tmp = coor_point[i].x;
				}
			}

			printf("the top_point is (%d, %d)\n", top_point.x, top_point.y);
			printf("the left_point is (%d, %d)\n", left_point.x, left_point.y);

			ratiox = (double)(top_point.x - x_min_tmp) / (x_max_tmp - x_min_tmp);
			ratioy = (double)(left_point.y - y_min_tmp) / (y_max_tmp - y_min_tmp);

			printf("the ratiox is %lf, the ratioy is %lf\n", ratiox, ratioy);

			if (Theta_for_cal >= 0 && Theta_for_cal < 90) {
				coor_point[0].x = minx;
				coor_point[0].y = miny + sin_theta * width;
				coor_point[1].x = minx + sin_theta * height;
				coor_point[1].y = maxy;
				coor_point[2].x = maxx;
				coor_point[2].y = maxy - sin_theta * width;
				coor_point[3].x = maxx - sin_theta * height;
				coor_point[3].y = miny;
			}
			else if (Theta_for_cal >= 90 && Theta_for_cal < 180) {
				coor_point[0].x = maxx - sin_theta * height;
				coor_point[0].y = maxy;
				coor_point[1].x = maxx;
				coor_point[1].y = miny + sin_theta * width;
				coor_point[2].x = minx + sin_theta * height;
				coor_point[2].y = miny;
				coor_point[3].x = minx;
				coor_point[3].y = maxy - sin_theta * width;
			}
			else if (Theta_for_cal >= 180 && Theta_for_cal < 270) {
				coor_point[0].x = maxx;
				coor_point[0].y = maxy + sin_theta * width;
				coor_point[1].x = maxx + sin_theta * height;
				coor_point[1].y = miny;
				coor_point[2].x = minx;
				coor_point[2].y = miny - sin_theta * width;
				coor_point[3].x = minx - sin_theta * height;
				coor_point[3].y = maxy;
			}
			else {
				coor_point[0].x = minx - sin_theta * height;
				coor_point[0].y = miny;
				coor_point[1].x = minx;
				coor_point[1].y = maxy + sin_theta * width;
				coor_point[2].x = maxx + sin_theta * height;
				coor_point[2].y = maxy;
				coor_point[3].x = maxx;
				coor_point[3].y = miny - sin_theta * width;
			}

			if (ratiox < 0.000001 || ratioy < 0.000001) {
				ivsContourResult.detectRectX0 = coor_point[0].x;
				ivsContourResult.detectRectY0 = coor_point[0].y;
				ivsContourResult.detectRectX1 = coor_point[1].x;
				ivsContourResult.detectRectY1 = coor_point[1].y;
				ivsContourResult.detectRectX2 = coor_point[2].x;
				ivsContourResult.detectRectY2 = coor_point[2].y;
				ivsContourResult.detectRectX3 = coor_point[3].x;
				ivsContourResult.detectRectY3 = coor_point[3].y;
			}
			else {
				cv::Point tmp_point;
				tmp_point = cal_coor_by_ratio(coor_point[0], coor_point[1], ratioy);
				ivsContourResult.detectRectX0 = tmp_point.x;
				ivsContourResult.detectRectY0 = tmp_point.y;
				tmp_point = cal_coor_by_ratio(coor_point[2], coor_point[1], ratiox);
				ivsContourResult.detectRectX1 = tmp_point.x;
				ivsContourResult.detectRectY1 = tmp_point.y;
				tmp_point = cal_coor_by_ratio(coor_point[2], coor_point[3], ratioy);
				ivsContourResult.detectRectX2 = tmp_point.x;
				ivsContourResult.detectRectY2 = tmp_point.y;
				tmp_point = cal_coor_by_ratio(coor_point[0], coor_point[3], ratiox);
				ivsContourResult.detectRectX3 = tmp_point.x;
				ivsContourResult.detectRectY3 = tmp_point.y;
			}
		}
		else {
			ivsContourResult.detectCircleX = finalResult.positionX;
			ivsContourResult.detectCircleY = finalResult.positionY;
		}

	}


	// 填充结果位图
	IVSTemplateSubStruct *tpl;
	if (ivsContourResult.toolScore >= com_score) {
		tpl = &templateInfo.tpls[0][finalResult.angleIndex];
	}
	else {
		tpl = &templateInfo.tpls[0][toolParameter.angleRange];
	}

	int bitmap_height = tpl->modelHeight;
	int bitmap_width = tpl->modelWidth;
	memset(contourResultAndBitmap, 0, sizeof(IVSContourResult) + sizeof(UINT8)* bitmap_height * bitmap_width);

	// 将IVSContourResult数据填充，之后直接填充位图
	memcpy(contourResultAndBitmap, &ivsContourResult, sizeof(IVSContourResult));
	UINT8 *bitmapAddr = contourResultAndBitmap + sizeof(IVSContourResult);
	INT16 cenx = tpl->centerOfGravity.x;
	INT16 ceny = tpl->centerOfGravity.y;
	for (UINT32 i = 0; i < tpl->noOfCordinates; i++) {
		*(bitmapAddr + bitmap_width * (tpl->cordinates[i].y + ceny) + (tpl->cordinates[i].x + cenx)) = 1;
	}

	// 由于客户端显示轮廓比较细，不易观察，所以在这里进行一下膨胀操作
	cv::Mat contourDilation;
	contourDilation.create(bitmap_height, bitmap_width, CV_8UC1);
	bitmap2Mat(contourDilation, bitmapAddr, bitmap_width, bitmap_height);
	Dilation(contourDilation, contourDilation, 3);
	//    cv::imshow("contourDilation", contourDilation);
	//    cv::waitKey(0);
	mat2Bitmap(contourDilation, bitmapAddr, bitmap_width, bitmap_height);

	*bufSize = sizeof(IVSContourResult) + bitmap_height * bitmap_width;


	return 0;
}

int FeatureLocationTool::freeTask() {
	///* 由内至外释放内存, 只释放工具的内容, 查找表也得释放了，目前查找表是在各个工具中的，可以考虑移出来 */
	//INT32 i = 0, j = 0;
	//Koyo_Template_Match_Info *pstTemplate_match_info = (Koyo_Template_Match_Info*)palg_handle->run_tool_param;

	//KOYO_LOG_INFO("here.\n");
	///* 释放所有的模板 */
	//UINT16 tplsize;
	//for (INT32 tpl_i = 0; tpl_i<pstTemplate_match_info->run_time_npyramid; ++tpl_i)
	//{
	//	/* 释放所有的模板 */
	//	free(pstTemplate_match_info->tpls[tpl_i]);
	//	//        KOYO_LOG_INFO("here, %d %d.\n", tpl_i, run_tool_param->run_time_npyramid);
	//}
	//free(pstTemplate_match_info->tpls);
	//KOYO_LOG_INFO("here.\n");

	////    INT16 nangs_temp = (INT16)ceil((double)MAX_DEGREE/pstTemplate_match_info->search_angel_nstep[1]);
	////    int width = tool_utility->u16Width >> 1;
	////    int height = tool_utility->u16Height >> 1;
	////    for (INT16 wtemp = 0; wtemp < width; ++wtemp) {
	////        for (INT16 htemp = 0; htemp < height; ++htemp) {
	////            free(pstTemplate_match_info->iscompute[wtemp][htemp]);
	////        }
	////        free(pstTemplate_match_info->iscompute[wtemp]);
	////    }
	////    free(pstTemplate_match_info->iscompute);

	//KOYO_LOG_INFO("here.\n");
	return 0;
}

// 根据待测图长宽，对应到待测图的坐标，以及传递过来的模板的搜索框边长, 求出相交的矩形
cv::Rect getRuningRect(cv::Rect searchRegion, cv::Point mapCenter, UINT16 top_search_rect_width) {
	int a = (float)top_search_rect_width / 2;
	int x0 = (std::max)(searchRegion.x, mapCenter.x - a);
	int x2 = (std::min)(searchRegion.x + searchRegion.width, mapCenter.x + a);
	int y0 = (std::max)(searchRegion.y, mapCenter.y - a);
	int y2 = (std::min)(searchRegion.y + searchRegion.height, mapCenter.y + a);

	cv::Rect result(x0, y0, x2 - x0, y2 - y0);
	return result;
}


// 根据矩形，求出此时待测图中的轮廓点数
int get_num_of_rect(cv::Rect searchRegion, cv::Rect rect, cv::Mat cannyBinary) {
	int x0 = rect.x - searchRegion.x;
	int y0 = rect.y - searchRegion.y;
	int x2 = x0 + rect.width;
	int y2 = y0 + rect.height;

	int noOfPoint = 0;
	for (int i = x0; i < x2; ++i) {
		for (int j = y0; j < y2; ++j) {
			if (cannyBinary.at<uchar>(j, i) == 255) {
				++noOfPoint;
			}
		}
	}
	return noOfPoint;
}

// 计算待测图中相对应的点的个数
int num_of_mapping_point(cv::Rect rect, std::vector<cv::Point> &tpl_point, const cv::Mat binary, const cv::Point &mapCenter) {
	int noOfPoint = 0;
	for (auto point : tpl_point) {
		point.x += mapCenter.x;
		point.y += mapCenter.y;
		if (rect.contains(point) && binary.at<uchar>(point.y, point.x) == 255) {
			++noOfPoint;
		}
	}
	return noOfPoint;
}

// 进行其他层的匹配，从最顶层往下继续匹配（最底层评分计算有所不同）
int FeatureLocationTool::doOtherLayerMatch(ContourUtility &contourUtility, int cur_level) {

	std::vector<IVSTemplateSubStruct> &cur_tpls = templateInfo.tpls[cur_level];
	float cur_angle_step = templateInfo.searchAngelStep[cur_level];
	UINT16 cur_search_rect_width = templateInfo.searchRectWidth[cur_level];

	std::priority_queue<CandidateResult> curLevelCandidates;
	std::priority_queue<CandidateResult> upperLevelCandidates = candidates[templateInfo.pyramidLevelNum - 2 - cur_level];
	int upperLevelCandidatesNum = upperLevelCandidates.size();

	// 非顶层的时候需要重新计算从上一层到该层的候选点
	// 优先队列从0开始

	// 为避免重复计算，每次新的位置都需要查看该位置是否已经计算过
	std::vector<CandidateResult> calculatedResult;
	for (int i = 0; i < upperLevelCandidatesNum; ++i) {
		CandidateResult pos = upperLevelCandidates.top();
		upperLevelCandidates.pop();
		pos.positionX <<= 1;
		pos.positionY <<= 1;
		pos.angleIndex = (pos.angleIndex * templateInfo.searchAngelStep[cur_level + 1] /
			templateInfo.searchAngelStep[cur_level]) + 0.5;


		// 备选的质心坐标周围的点以及以当前角度左右几个该层角度步长的角度
		std::vector<cv::Point> point_bak;
		for (int i = pos.positionX - toolParameter.contourNaborSize; i <= pos.positionX + toolParameter.contourNaborSize && i < searchRect[cur_level].x + searchRect[cur_level].width; ++i) {
			for (int j = pos.positionY - toolParameter.contourNaborSize; j <= pos.positionY + toolParameter.contourNaborSize && j < searchRect[cur_level].y + searchRect[cur_level].height; ++j) {
				point_bak.push_back(cv::Point(i, j));
			}
		}


		// 进行相似度的计算

		for (auto centerPoint : point_bak) {
			cv::Rect rectRuning = getRuningRect(searchRect[cur_level], centerPoint, cur_search_rect_width);
			bool isCalculated = false;

			// (当前默认角度范围为正负2)
			for (int i = -toolParameter.contourNaborAngle; i <= toolParameter.contourNaborAngle; ++i) {
				int angleIndex = (pos.angleIndex + i + cur_tpls.size()) % cur_tpls.size();
				float posAngle = pos.angleIndex * cur_angle_step - toolParameter.angleRange;
				float tmpAngle = angleIndex * cur_angle_step - toolParameter.angleRange;
				// 这里做下修改+1，提高了算法稳定性
				if (fabs(posAngle - tmpAngle) > (toolParameter.contourNaborAngle + 1) * cur_angle_step) {
					//printf("当前角度不适合\n");
					continue;
				}
				// 当前角度是否计算过
				for (auto res : calculatedResult) {
					float resAngle = res.angleIndex * cur_angle_step - toolParameter.angleRange;
					if (abs(centerPoint.x - res.positionX) <= toolParameter.contourNaborSize
						&& abs(centerPoint.y - res.positionY) <= toolParameter.contourNaborSize
						&& fabs(resAngle - tmpAngle) <= toolParameter.contourNaborAngle  * cur_angle_step) {
						//printf("该位置及角度已经被计算过");
						isCalculated = true;
						break;
					}
				}
				if (isCalculated) {
					continue;
				}

				// 进行计算
				IVSTemplateSubStruct &tpl = cur_tpls[angleIndex];
				int K1 = tpl.cordinates.size();
				int num_T = 0;
				double sum_of_s = 0.0;
				int order = 0;
				for (auto point : tpl.cordinates) {
					point.x += centerPoint.x;
					point.y += centerPoint.y;
					if (rectRuning.contains(point)) {
						++num_T;
						sum_of_s += fabs(tpl.edgeDerivativeX[order] * contourUtility.edgeX[cur_level][point.y][point.x]+ tpl.edgeDerivativeY[order] * contourUtility.edgeY[cur_level][point.y][point.x]);
					}
					++order;
					/*if (sum_of_s < (toolParameter.contourNumThreshold - 1)*K1 + num_T) {
						break;
					}*/
				}

				//int zeroDegreeIndex = templateInfo.tpls[cur_level].size() / 2;
				double s = sum_of_s / K1;

				// 匹配相似度高的质心和角度记录下来
				if (s >toolParameter.contourScoreThreshold) {
					CandidateResult result;
					result.level = cur_level;
					result.positionX = centerPoint.x;
					result.positionY = centerPoint.y;
					result.angleIndex = angleIndex;
					result.score = s;

					

					// 读入该层的候选点小顶堆，默认最小候选点个数为1
					int curCandidateNum = (toolParameter.contourCandidateNum >> (templateInfo.pyramidLevelNum - 1 - cur_level)) > 1 ?
						( toolParameter.contourCandidateNum>> (templateInfo.pyramidLevelNum - 1 - cur_level)) : 1;
					//std::cout << "level " << cur_level << " have " << curCandidateNum << " candidate point." << std::endl;         
                                        //最底层只保留一个候选结果
                                        if(cur_level == 0)
                                             curCandidateNum = 1;

					if (curLevelCandidates.size() < curCandidateNum) {
						curLevelCandidates.push(result);
//std::cout << "000now center is (" << result.positionX << "," << result.positionY<< "), and the angle is " << result.angleIndex*cur_angle_step - toolParameter.angleRange<< "the similarity is " << s << std::endl;
					}
					else {
						if (curLevelCandidates.top().score < result.score) {
							curLevelCandidates.pop();
							curLevelCandidates.push(result);
//std::cout << "now center is (" << result.positionX << "," << result.positionY<< "), and the angle is " << result.angleIndex*cur_angle_step - toolParameter.angleRange<< "the similarity is " << s << std::endl;
						}
					}
				}
			}
		}

		// 将已经计算的上层候选位置记录下来
		calculatedResult.push_back(pos);

	}

	// 将该层候选点位置小顶堆读入到candidates中
	candidates.push_back(curLevelCandidates);
        std::cout<<"ONtop result:"<<curLevelCandidates.size()<<std::endl;
	return 0;
}


// 模板顶层对应的待测图金字塔的相应层数进行匹配
int FeatureLocationTool::doTopLayerMatch(ContourUtility &contourUtility) {
	// 准备必要的参数(0度模板时的轮廓点总数)
	int top_level = templateInfo.pyramidLevelNum - 1;
	std::vector<IVSTemplateSubStruct> &top_tpls = templateInfo.tpls[top_level];
	int K = top_tpls[top_tpls.size() / 2].noOfCordinates;
	float top_search_angel_nstep = templateInfo.searchAngelStep[top_level];
	int top_search_rect_width = templateInfo.searchRectWidth[top_level];

	// 创建一个候选信息的小顶堆
	std::priority_queue<CandidateResult> minHeapCandidate;

	//SampleStruct &spl = sampleStructs[level];
	//    std::cout << spl.height << " " << spl.width << std::endl;


	for (int i = searchRect[top_level].y; i < searchRect[top_level].y + searchRect[top_level].height; ++i) {
		for (int j = searchRect[top_level].x; j < searchRect[top_level].x + searchRect[top_level].width; ++j) {
			// 对应到待测图的坐标
			cv::Point mapCenter;
			mapCenter.x = j;
			mapCenter.y = i;

			cv::Rect rectRuning = getRuningRect(searchRect[top_level], mapCenter, top_search_rect_width);
			// 判断搜索框的运行情况
			//cv::Mat rectPic = contourUtility.searchRegion[top_level].clone();
			//cv::rectangle(rectPic,rectRuning,cv::Scalar(255, 255, 255),1,8);

			int num = get_num_of_rect(searchRect[top_level], rectRuning, cannyPyramid[top_level]);
			//std::cout << "this rect has num is " << num << std::endl;
			//cv::imshow("rectangle", rectPic);
			//cv::waitKey(0);
			if (num > K * toolParameter.contourNumThreshold) {
				// std::cout << "OK the num of Point more than K*S and now center is ("
				// << j << "," << i << ")" << std::endl;

				for (int k = 0; k < templateInfo.tpls[top_level].size(); ++k) {
					IVSTemplateSubStruct &tpl = top_tpls[k];
					int K1 = tpl.cordinates.size();
					//cv::imshow("cannyPyramid[top_level]", cannyPyramid[top_level]);
					//cv::waitKey(0);
					//if(num_of_mapping_point(rectRuning, tpl.cordinates, cannyPyramid[top_level], mapCenter) > K1 * toolParameter.contourNumThreshold){
					//std::cout << "OK the num of Point more than K*S and now center is ("
					// << j << "," << i << "), and the angle is " << k << std::endl;
					// 相似度计算公式
					int num_T = 0;
					double sum_of_s = 0.0;
					int order = 0;
					for (auto point : tpl.cordinates) {
						point.x = point.x + mapCenter.x;
						point.y = point.y + mapCenter.y;
						if (rectRuning.contains(point)) {
							++num_T;
							sum_of_s += fabs(tpl.edgeDerivativeX[order] * contourUtility.edgeX[top_level][point.y][point.x]+ tpl.edgeDerivativeY[order] * contourUtility.edgeY[top_level][point.y][point.x]);
						}
						++order;
						/*if (sum_of_s < (toolParameter.contourNumThreshold - 1)*K1 + num_T) {
							break;
						}*/
					}
					double s = sum_of_s / num_T;
					//std::cout << "OK the num of Point more than K*S and now center is ("<< j << "," << i << "), and the angle is " << k << "the similarity is " << s << std::endl;

					// 匹配相似度最高的质心和角度记录下来
					if (s > toolParameter.contourScoreThreshold) {
						CandidateResult result;
						result.angleIndex = k;
						result.level = top_level;
						result.positionX = j;
						result.positionY = i;
						result.score = s;
						if (minHeapCandidate.size() < toolParameter.contourCandidateNum) {
							minHeapCandidate.push(result);
//std::cout << "000now center is (" << j << "," << i << "), and the angle is " << k*top_search_angel_nstep<< "the similarity is " << s << std::endl;
						}
						else {
							if (minHeapCandidate.top().score < result.score) {
								minHeapCandidate.pop();
								minHeapCandidate.push(result);
                                                                //std::cout << "now center is (" << j << "," << i << "), and the angle is " << k*top_search_angel_nstep<< "the similarity is " << s << std::endl;
							}
						}
						
					}
					//}
				}
			} 
		}
	}
        
	candidates.push_back(minHeapCandidate);
        //for(int i=0;i<minHeapCandidate.size();i++)
        //{
           //cv::Point p1.x=minHeapCandidate.x-
           //cv::rectangle(cannyPyramid[3], p1, p2, Scalar(0,255,0),1,8,0);
        //}
        std::cout<<"top result:"<<minHeapCandidate.size()<<std::endl;
	return 0;
}

static void draw_template(cv::Mat &src, const IVSTemplateSubStruct &tpl, cv::Point centerPoint)
{
	for (UINT32 i = 0; i < tpl.noOfCordinates; ++i) {
		cv::circle(src, cv::Point(tpl.cordinates[i].x + centerPoint.x, tpl.cordinates[i].y + centerPoint.y),
			1, cv::Scalar(255, 255, 255));
	}
}

//src 待测图像
INT8 FeatureLocationTool::doTemplateMatch(ContourUtility &contourUtility)
{

	INT8 ret;
	INT8 run_time_npyramid = templateInfo.pyramidLevelNum;

	// 由于高速度策略也检测到最底层，同时保证改动不大，只修改这三处，其二
	//INT8 com_level = p_template_matchInfo->contour_parameter.algo_strategy;   ///fixme:now is zero
	INT8 com_level = 0;
	//    KOYO_LOG_INFO("begin match.\n");

	// 顶层匹配分离出来
	doTopLayerMatch(contourUtility);

	// 其他层的匹配
	for (int l = run_time_npyramid - 2; l >= com_level; --l) {

		printf("begin match level %d.\n", l);
		doOtherLayerMatch(contourUtility, l);


		//if (l == com_level)
		//{
		//	p_template_matchInfo->res_coor.y = result_out->data[0]->position.y;
		//	p_template_matchInfo->res_coor.x = result_out->data[0]->position.x;
		//	p_template_matchInfo->res_angel = result_out->data[0]->angel_idx;
		//	p_template_matchInfo->res_score = result_out->data[0]->score + 0.5;
		//}
		//if (ret < 0) {
		//	printf("Error at match_template_one_level %d.\n", l);
		//	return ret;
		//}
		//printf("level %d match %d targets.\n", l, result_out->n_scalar);
		//tmp = result_in;
		//result_in = result_out;
		//result_out = tmp;
		//printf("--------------------------------------------.\n");

	}

	// 将结果绘制出来
	if (!candidates[templateInfo.pyramidLevelNum - 1].empty()) {
		CandidateResult finalResult = candidates[templateInfo.pyramidLevelNum - 1].top();
		std::cout << "the final result is x: " << finalResult.positionX << " y: " << finalResult.positionY
			<< " the angleindex is: " << finalResult.angleIndex << " score: " << finalResult.score << std::endl;
	}


	//cv::Mat tmpresult = contourUtility.searchRegion[0].clone();
	//draw_template(tmpresult, templateInfo.tpls[0][finalResult.angleIndex],
	//	cv::Point(finalResult.positionX, finalResult.positionY));
	//cv::imshow("result", tmpresult);
	//cv::waitKey(0);
	return 0;
}


int packTemplate(const IVSTemplateStruct &ivsTemplateStruct, UINT8 *buf, int buf_free, size_t *bufsize)
{
	// 先计算所有要使用的内存大小，然后分配空间，最后一点点将数据拷贝过去
	size_t buf_size = 0;

	buf_size += sizeof(ivsTemplateStruct.pyramidLevelNum);	// 保存金字塔层数
	buf_size += sizeof(float)* ivsTemplateStruct.pyramidLevelNum;	// 保存金字塔每层的角度步长
	buf_size += sizeof(UINT16)* ivsTemplateStruct.pyramidLevelNum;	// 保存金字塔每层的搜索框框长

	// 要记录每层金字塔上的模板个数
	for (const auto &tpl_arr : ivsTemplateStruct.tpls) {
		// 每层金字塔上的模板个数
		buf_size += sizeof(UINT16);
		for (const auto &tpl : tpl_arr) {
			buf_size += sizeof(tpl.modelDefined);
			buf_size += sizeof(tpl.noOfCordinates);
			buf_size += sizeof(tpl.modelHeight);
			buf_size += sizeof(tpl.modelWidth);

			buf_size += sizeof(short) * 2;	// 保存重心坐标

			buf_size += sizeof(cv::Point) * tpl.noOfCordinates;
			buf_size += sizeof(float)* tpl.noOfCordinates;
			buf_size += sizeof(float)* tpl.noOfCordinates;
		}
	}

	if (bufsize && buf_size > buf_free) {
		*bufsize = buf_size;
		return -1;
	}
	if (buf_size > buf_free) {
		return -1;
	}

	std::size_t index = 0;
	memcpy(&buf[index], &ivsTemplateStruct.pyramidLevelNum, sizeof(ivsTemplateStruct.pyramidLevelNum));
	index += sizeof(ivsTemplateStruct.pyramidLevelNum);

	for (int i = 0; i < ivsTemplateStruct.pyramidLevelNum; ++i) {
		memcpy(&buf[index], &ivsTemplateStruct.searchAngelStep[i], sizeof(float));
		index += sizeof(float);
	}


	for (int i = 0; i < ivsTemplateStruct.pyramidLevelNum; ++i) {
		memcpy(&buf[index], &ivsTemplateStruct.searchRectWidth[i], sizeof(UINT16));
		index += sizeof(UINT16);
	}


	// 拷贝模板数据
	for (const auto &tpl_arr : ivsTemplateStruct.tpls) {
		UINT16 tpl_size = static_cast<UINT16>(tpl_arr.size()); //肯定不会超的
		memcpy(&buf[index], &tpl_size, sizeof(UINT16));
		index += sizeof(UINT16);

		for (const auto &tpl : tpl_arr) {
			memcpy(&buf[index], &tpl.modelDefined, sizeof(tpl.modelDefined));
			index += sizeof(tpl.modelDefined);

			memcpy(&buf[index], &tpl.noOfCordinates, sizeof(tpl.noOfCordinates));

			index += sizeof(tpl.noOfCordinates);

			memcpy(&buf[index], &tpl.modelHeight, sizeof(tpl.modelHeight));
			index += sizeof(tpl.modelHeight);

			memcpy(&buf[index], &tpl.modelWidth, sizeof(tpl.modelWidth));
			index += sizeof(tpl.modelWidth);

			memcpy(&buf[index], &tpl.centerOfGravity.x, sizeof(short));
			index += sizeof(short);

			memcpy(&buf[index], &tpl.centerOfGravity.y, sizeof(short));
			index += sizeof(short);

			for (auto const &coord : tpl.cordinates) {
				cv::Point point;
				point.x = static_cast<INT16>(coord.x);
				point.y = static_cast<INT16>(coord.y);
				memcpy(&buf[index], &point, sizeof(cv::Point));
				index += sizeof(cv::Point);

			}
			for (auto const & edgeX : tpl.edgeDerivativeX) {
				memcpy(&buf[index], &edgeX, sizeof(float));
				index += sizeof(float);
			}

			for (auto const & edgeY : tpl.edgeDerivativeY) {
				memcpy(&buf[index], &edgeY, sizeof(float));
				index += sizeof(float);
			}
		}
	}
	std::cout << "buf_size: " << buf_size << ", index: " << index << std::endl;
	std::cout << buf_size << ", in MB: " << 1.0 * buf_size / 1024 / 1024 << "MB" << std::endl;

	std::cout << "after pack" << std::endl;
	std::cout << buf_size << ", in MB: " << 1.0 * buf_size / 1024 / 1024 << "MB" << std::endl;

	return 0;
}

void printTPLByImage(const IVSTemplateSubStruct &tpl) {
	cv::Mat tmp(tpl.modelHeight, tpl.modelWidth, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < tpl.noOfCordinates; ++i) {
		tmp.at<uchar>(tpl.cordinates[i].y + tpl.centerOfGravity.y, tpl.cordinates[i].x + tpl.centerOfGravity.x) = 255;
	}
	cv::imshow("tmp", tmp);
	cv::waitKey(0);
}

static std::ostream &printTPLByInfo(std::ostream &os, const IVSTemplateStruct & rhl)
{
	os << "runtime npyramid: " << static_cast<int>(rhl.pyramidLevelNum) << std::endl;
	os << "angle steps each level: " << std::endl;
	int k = 0;
	for (auto angle : rhl.searchAngelStep) {
		os << "level " << k++ << ": " << angle << std::endl;
	}

	k = 0;
	os << "templates each level: " << std::endl;
	for (auto tpl : rhl.tpls) {
		os << "level " << k++ << ": " << tpl.size() << ", Num of coordinate this level: " << tpl[0].noOfCordinates << std::endl;
	}
	return os;
}

static std::vector<float> rotateImage(const cv::Mat &src, cv::Mat &dst, cv::Point centerP, float degree)
{
	int width = src.cols;
	int height = src.rows;
	double angle = degree * CV_PI / 180.; // 弧度
	double a = sin(angle), b = cos(angle);

	// 适当增大一点宽高，防止像素不在图像内
	int width_rotate = int(height * fabs(a) + width * fabs(b));
	int height_rotate = int(width * fabs(a) + height * fabs(b));
	float map[6];
	cv::Mat map_matrix = cv::Mat(2, 3, CV_32F, map);

	// 旋转中心, 以原始图片中心作为旋转中心而不是质心
	CvPoint2D32f center = cvPoint2D32f(width / 2.0, height / 2.0);
	CvMat map_matrix2 = map_matrix;
	cv2DRotationMatrix(center, degree, 1.0, &map_matrix2);

	// 这里不能改
	map[2] += (width_rotate - width) / 2.0;
	map[5] += (height_rotate - height) / 2.0;
	cv::warpAffine(src, dst, map_matrix, cv::Size(width_rotate, height_rotate));
	std::vector<float> rotate_matrix;
	rotate_matrix.push_back(map[0]);
	rotate_matrix.push_back(map[1]);
	rotate_matrix.push_back(map[2]);
	rotate_matrix.push_back(map[3]);
	rotate_matrix.push_back(map[4]);
	rotate_matrix.push_back(map[5]);
	return rotate_matrix;
}

static int rotateRect(std::vector<cv::Point> &rect, const std::vector<float> rotate_matrix)
{
	int plx = rect[0].x, ply = rect[0].y, prx = rect[3].x, pry = rect[3].y;
	int plxb = rect[1].x, plyb = rect[1].y, prxb = rect[2].x, pryb = rect[2].y;

	rect[0].x = rotate_matrix[0] * plx + rotate_matrix[1] * ply + rotate_matrix[2];
	rect[0].y = rotate_matrix[3] * plx + rotate_matrix[4] * ply + rotate_matrix[5];

	rect[1].x = rotate_matrix[0] * plxb + rotate_matrix[1] * plyb + rotate_matrix[2];
	rect[1].y = rotate_matrix[3] * plxb + rotate_matrix[4] * plyb + rotate_matrix[5];

	rect[2].x = rotate_matrix[0] * prxb + rotate_matrix[1] * pryb + rotate_matrix[2];
	rect[2].y = rotate_matrix[3] * prxb + rotate_matrix[4] * pryb + rotate_matrix[5];

	rect[3].x = rotate_matrix[0] * prx + rotate_matrix[1] * pry + rotate_matrix[2];
	rect[3].y = rotate_matrix[3] * prx + rotate_matrix[4] * pry + rotate_matrix[5];
	return 0;
}

static unsigned short calAlign(unsigned short len, unsigned char align)
{
	return (len + (align - len % align) % align);
}


/*
*  输入图像是二值化图像
* */
static int getCenterNumOfContour(const cv::Mat src, cv::Point &center, unsigned int &numofcontour, int height, int width)
{
	double m00, m10, m01;
	auto moments = cv::moments(src, true);
	m10 = moments.m10;
	m01 = moments.m01;
	m00 = moments.m00;
	if (m00 == 0) {
		return -1;
	}
	else {
		// 求旋转角度步长的中心应该选为模板的中心，而不是质心
		center.x = width / 2;
		center.y = height / 2;
	}
	numofcontour = m00;
	return 0;
}


// 为了保证最顶层的点数足够多，将二层图像先膨胀后再向上求最顶层的二值图（最新没有使用膨胀操作，保留以后使用）
void getTopLevelBinaryPicByDilationLevel(const IVSTemplateSubStruct &tpl, cv::Mat &ret_mat, int height, int width) {
	cv::Mat tmp(tpl.modelHeight, tpl.modelWidth, CV_8UC1, cv::Scalar(0));
	cv::Mat tmp_sameSize(height, width, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < tpl.noOfCordinates; ++i) {
		tmp.at<uchar>(tpl.cordinates[i].y + tpl.centerOfGravity.y, tpl.cordinates[i].x + tpl.centerOfGravity.x) = 255;
	}
	// 膨胀
	cv::Mat tmp_dilation, tmp_notSize;
	Dilation(tmp, tmp_dilation, 2);

	// downSample 只取偶数行列
	for (int i = 0; i < tmp_sameSize.rows; ++i) {
		for (int j = 0; j < tmp_sameSize.cols; ++j) {
			if (2 * i < tmp_dilation.rows && 2 * j < tmp_dilation.cols && tmp_dilation.at<uchar>(2 * i, 2 * j) == 255) {
				tmp_sameSize.at<uchar>(i, j) = 255;
			}
		}
	}
	ret_mat = tmp_sameSize.clone();
}

/*
* rect是相对640*480图片的坐标
* */
static int doCreateTemplateIn(IVSTemplateSubStruct &tpl, const cv::Mat &src, const cv::Mat &bitmap, bool do_bitwise_and, double low_threshold, \
	double high_threshold, const cv::Mat &rmWhite, int level, int max_level, const IVSTemplateSubStruct &pre_tpl)
{
	int s32Ret = 0;
	cv::Mat gx;                //Matrix to store X derivative
	cv::Mat gy;                //Matrix to store Y derivative
							   // set width and height
	tpl.modelHeight = static_cast<UINT16>(src.rows);    //Save Template height
	tpl.modelWidth = static_cast<UINT16>(src.cols);    //Save Template width

	tpl.noOfCordinates = 0;    //initialize

	cv::Sobel(src, gx, CV_16S, 1, 0);        //gradient in X direction
	cv::Sobel(src, gy, CV_16S, 0, 1);        //gradient in Y direction

											 //    cv::Mat binaryContour;
											 //    cv::Canny(src, binaryContour, low_threshold, high_threshold);

	cv::Mat binaryContour, before_filter;

	// 在这里进行最顶层的膨胀操作（为了保留更多的点，所以二层先做膨胀操作，之后再向上建立二值化的顶层）
	if (level == max_level - 1) { // 顶层
								  //get_topLevel_binaryPic_by_dilationLevel(pre_tpl, before_filter, src.rows, src.cols);
		cv::Canny(src, before_filter, low_threshold, high_threshold);
	}
	else {
		cv::Canny(src, before_filter, low_threshold, high_threshold);
	}

	//canny的结果和bitmap相与
	//cv::imshow("before", before_filter);
	//cv::waitKey(0);
	// 把bitmap膨胀一下
	cv::Mat dialBitmap;
	Dilation(bitmap, dialBitmap, 3);
	cv::threshold(dialBitmap, bitmap, 10, 255, CV_THRESH_BINARY);

	//    std::cout << before_filter.cols << " " << before_filter.rows << std::endl;
	//    std::cout << bitmap.cols << " " << bitmap.rows << std::endl;
	//cv::imshow("bitmat", bitmap);
	//cv::waitKey(0);
	if (do_bitwise_and) {
		cv::bitwise_and(before_filter, rmWhite, before_filter);
		cv::bitwise_and(before_filter, bitmap, binaryContour);
	}
	else {
		if (level != max_level - 1) { // 顶层
			cv::bitwise_and(before_filter, rmWhite, before_filter);
		}
		binaryContour = before_filter;
	}
	//cv::imshow("binary", binaryContour);
	//cv::waitKey(0);

	int RSum = 0, CSum = 0;

	for (int i = 0; i < tpl.modelHeight; i++) {
		for (int j = 0; j < tpl.modelWidth; j++) {
			short fdx = gx.at<short>(i, j);
			short fdy = gy.at<short>(i, j);
			unsigned char U8 = binaryContour.at<uchar>(i, j);
			// 考虑到如果擦除点数太多，导致匹配过程中一级匹配出现问题，因此点数应该统计为未擦除前的
			//unsigned char U8_before = before_filter.at<uchar>(i, j);
			cv::Point p;
			p.x = j;
			p.y = i;
			// todo 这里由于使用了位与操作，所以不用再判断距离了
			// 最小距离是多少还需要斟酌，因为在最小分辨率情况下看到边框还是没有去除掉，在最小分辨情况下这个dist太小了。
			//double min_dist = MIN_DIST;
			// todo 这里由于使用了位与操作，所以不用再判断距离了
			if (U8) {
				/* 如果梯度都为零，那么不需要计算，因为分数不会有贡献 */
				if (fdx != 0 || fdy != 0) {
					/* 坐标变换到外接矩形左上角为(0, 0) */
					RSum = RSum + j;
					CSum = CSum + i;    // Row sum and column sum for center of gravity
					tpl.cordinates.push_back(p);
					//                    tpl.cordinates[tpl.noOfCordinates].x = j;
					//                    tpl.cordinates[tpl.noOfCordinates].y = i;

					/* TODO 可以修改成使用查找表的形式 */
					double vector_length = sqrt(fdx * fdx + fdy * fdy);
					if (fabs(vector_length - 0.) < 0.00001) {
						//                        printf(".............................................\n");
					}
					tpl.edgeDerivativeX.push_back(static_cast<float>(fdx / vector_length));
					tpl.edgeDerivativeY.push_back(static_cast<float>(fdy / vector_length));
					tpl.noOfCordinates++;
				}
			}
		}
	}

	if (tpl.noOfCordinates == 0) {
		//        printf(".........................");
		tpl.centerOfGravity.x = tpl.modelWidth / 2;
		tpl.centerOfGravity.y = tpl.modelHeight / 2;
	}
	else {
		tpl.centerOfGravity.x = tpl.modelWidth / 2;
		tpl.centerOfGravity.y = tpl.modelHeight / 2;
		//tpl.centerOfGravity.x = RSum / tpl.noOfCordinates;    // center of gravity
		//tpl.centerOfGravity.y = CSum / tpl.noOfCordinates;    // center of gravity
	}

	// change coordinates to reflect center of gravity
	/* 将重心变换到坐标原点 */
	UINT32 m;
	for (m = 0; m < tpl.noOfCordinates; m++) {
		/*int temp;

		temp = tpl.cordinates[m].x;
		tpl.cordinates[m].x = temp - tpl.centerOfGravity.x;
		temp = tpl.cordinates[m].y;
		tpl.cordinates[m].y = temp - tpl.centerOfGravity.y;*/
		tpl.cordinates[m].x -= tpl.centerOfGravity.x;
		tpl.cordinates[m].y -= tpl.centerOfGravity.y;
	}

	tpl.modelDefined = true;

	return 1;
}



/*
*  @param src必须是二值化轮廓图
*  @return 返回当前层上最佳的旋转步长
* */
static float getAngleStep(const cv::Mat &src, cv::Point center)
{
	// 重新修改，因为现在使用搜索框的中心做质心，所以搜索框的框长需要重新计算

	// 保留几个K，然后求平均值，用来排除外点的影响
	int K = 10;
	std::priority_queue<float> max_dist(K, -1);
	std::vector<cv::Point> points;
	//    double max_distance = -1;
	// 在目前没有旋转的图片中不会出现因为旋转导致的白边，所以直接在全图搜索就行了，不用考虑别的
	for (int i = 0; i < src.rows; ++i) {
		for (int j = 0; j < src.cols; ++j) {
			if (src.at<uchar>(i, j)) {
				double dist = -1 * sqrt(pow((i - center.y), 2) + pow((j - center.x), 2));
				//                if(-dist > max_distance) max_distance = -dist;
				if (dist < max_dist.top()) {
					max_dist.pop();
					max_dist.push(dist);
					points.push_back({ i, j });
				}
			}
		}
	}
	// 搜索框是最远点的位置，这样的话搜索框就不会超出了
	//    search_rect_width.push_back(static_cast<UINT16>(1 + max_distance));


	float average_max_dist = 0;
	int i = 0;
	while (!max_dist.empty()) {
		average_max_dist += -max_dist.top();
		max_dist.pop();
	}
	average_max_dist /= K;
	//    std::cout << "average_max_dist: " << average_max_dist << std::endl;

	//// 此处乘以2的原因是因为之前都是求的距离，而搜索框的框长必须为距离的2倍
	//search_rect_width.push_back(static_cast<UINT16>(average_max_dist)*2);
	auto range_low = acos(1 - 1 / (2 * average_max_dist * average_max_dist)) / CV_PI * 180;
	auto range_high = acos(1 - 2 / (average_max_dist * average_max_dist)) / CV_PI * 180;
	//    std::cout <<"optimal angle step: " << range_low << " ~ " << range_high << std::endl;

#ifdef NDEBUG
	cv::Mat tmp = src;
	for (unsigned int k = points.size() - K; k < points.size(); ++k) {
		cv::circle(tmp, points[k], 20, cv::Scalar(255, 255, 255));
	}
	//    cv::imshow("max dist", tmp);
	//    cvWaitKey(0);
	std::cout << range_low << " " << range_high << std::endl;
#endif

	float result = (std::max)((range_low + range_high) / 2, 1.0);
	return result < 6.0 ? result : 6.0;
}


/*
* src是截取出来的模板图片
* bitmapCleaned是位图，大小和src一致
*/
int FeatureLocationTool::doCreateTemplate(const cv::Mat &src, const cv::Mat &bitMap)
{

	//  TimeTracker tt1;
	//   tt1.start();
	std::vector<cv::Mat> pyramid_bitmaps;
	std::vector<cv::Mat> pyramid_templates;
        cv::GaussianBlur(src, src, cv::Size(5,5),3,3);
	pyramid_templates.push_back(src);
	pyramid_bitmaps.push_back(bitMap);

	UINT8 sensitity_threshold_low, sensitity_threshold_high;

	sensitity_threshold_low = toolParameter.sensiLowThreshold;
	sensitity_threshold_high = toolParameter.sensiLowThreshold * 3;
	printf("the koyo_tool_contour_parameter->sensitivity_Low_Threshold is %d, the koyo_tool_contour_parameter->sensitivity_Low_Threshold*3 is %d", sensitity_threshold_low, sensitity_threshold_high);

	std::cout << "create_template begin" << std::endl;

	// 建立各层金字塔, 并确定最佳金字塔层数
	int optimal_pyr_level = 0;

	for (int i = 0; i < toolParameter.maxNumPyramid - 1; ++i) {
		cv::Mat next_level;
		cv::Mat next_level_bmap;
		//        cv::GaussianBlur(pyramid_templates[i], pyramid_templates[i], cv::Size(5,5),3);
		//        cv::GaussianBlur(pyramid_bitmaps[i], pyramid_bitmaps[i], cv::Size(5,5),3);
		cv::pyrDown(pyramid_bitmaps[i], next_level_bmap);
		cv::pyrDown(pyramid_templates[i], next_level);
		pyramid_templates.push_back(next_level);
		pyramid_bitmaps.push_back(next_level_bmap);
		//cv::imshow("next_level", next_level);
		//cv::waitKey(0);
		//cv::imshow("next_level_bmap", next_level_bmap);
		//cv::waitKey(0);
	}
	
#if 0
	// 做过高斯滤波就不做滤波
	for (auto iter = std::begin(pyramid_templates); iter != std::end(pyramid_templates); ++iter) {
		cv::Mat after_gaus;
		cv::GaussianBlur(*iter, after_gaus, cv::Size(5, 5), 5, 5);
		*iter = after_gaus;
	}
#endif
	// 只对最底层进行滤波
	//    cv::Mat after_gaus;
	//    cv::GaussianBlur(pyramid_templates[0], after_gaus, cv::Size(5,5),3);
	//    pyramid_templates[0] = after_gaus;

	// 图像的质心
	std::vector<cv::Point> centers;
	std::vector<float> angle_steps;
	std::vector<UINT16> search_rect_width;
	for (auto iter = std::begin(pyramid_templates); iter != std::end(pyramid_templates); ++iter) {
		std::cout << "1 rows: " << iter->rows << " cols: " << iter->cols << std::endl;
	}
	std::cout << "error1" << std::endl;

	for (auto &pyr : pyramid_templates) {
#ifndef  NDEBUG
		//        saveMat(pyr, (std::string("data//") + std::to_string(pyr.rows) + std::to_string(pyr.cols)).c_str());
#endif
		cv::Mat cannyResult;
		cv::Canny(pyr, cannyResult, sensitity_threshold_low, sensitity_threshold_high);
		//cv::imshow("hahahaha", cannyResult);
		//cv::waitKey(0);
		std::cout << "2 rows: " << pyr.rows << " cols: " << pyr.cols << std::endl;
		cv::Point center;
		unsigned int num_of_contour = 0;
		getCenterNumOfContour(cannyResult, center, num_of_contour, pyr.rows, pyr.cols);
		centers.push_back(center);

		// 重新确定运行搜索框框长
		// 矩形
		if (toolParameter.regionShape == REGION_SHAPE_RECT) {
			cv::Point p0, p2;
			p0.x = toolParameter.detectRectX0;
			p0.y = toolParameter.detectRectY0;
			p2.x = toolParameter.detectRectX2;
			p2.y = toolParameter.detectRectY2;
			float dist = sqrt(pow(p0.x - p2.x, 2) + pow(p0.y - p2.y, 2));
			dist /= pow(2, optimal_pyr_level);
			search_rect_width.push_back(dist);
		}
		else {
			// 圆形
			float dist = (toolParameter.detectCircleRadius) * 2;
			dist /= pow(2, optimal_pyr_level);
			search_rect_width.push_back(dist);
		}

		// 确定角度步长
		auto step = getAngleStep(cannyResult, center);
		if (optimal_pyr_level == 0 || optimal_pyr_level == 1) {
			step = 1.0;
		}
		angle_steps.push_back(step);

		std::cout << "num of coordinate this level: " << num_of_contour << std::endl;

		if (optimal_pyr_level >= toolParameter.maxNumPyramid || (num_of_contour < toolParameter.minContourPyra && optimal_pyr_level >= toolParameter.minNumPyramid)) {
			break;
		}

		// 当策略为高精度时，保证模板层数更低，提高匹配精度
		if (toolParameter.algoStrategy == 0 && optimal_pyr_level ==toolParameter.highPrecisionNumPyramid) {
			break;
		}
		++optimal_pyr_level;
	}
	std::cout << "optimal level: " << optimal_pyr_level << std::endl;
	//   tt1.stop();
	//    std::cout << "first half: " << tt1.duration() << std::endl;

	// 对每层每个角度建立模板
	// tpls中的内存是动态分配的, 在建立完模板后需要释放所有的内存
	// todo 重构成vector版本的
	//    TemplateStruct tpls[optimal_pyr_level][MAX_DEGREE];
	std::vector<std::vector<IVSTemplateSubStruct>> tpls;
	////   TimeTracker tt;
	//    tt.start();
	// optimal_pyr_level肯定小于pyramid_templates的size
	std::cout << "angle_range is : " << (int)toolParameter.angleRange << std::endl;
	for (int i = 0; i < optimal_pyr_level; ++i) {
		std::vector<IVSTemplateSubStruct> cur_level_tpl;
		int k = 0;
		std::vector<cv::Point> cur_rect = { { 0, 0 },{ 0, pyramid_templates[i].rows - 1 },{ pyramid_templates[i].cols - 1, pyramid_templates[i].rows - 1 },{ pyramid_templates[i].cols - 1, 0 } };
		// todo angle_step是否考虑做成整数
		for (double j = -(toolParameter.angleRange); (int)j < toolParameter.angleRange; j += angle_steps[i]) {
			//            std::cout << j << " " << (int)j << std::endl;
			IVSTemplateSubStruct tpl;
			auto rect = cur_rect;
			cv::Mat rotated_image;
			cv::Mat rotated_image_bmap;

			// 在这里做与的操作，便于之后不再进行去除白边的繁琐操作，因此需要进行下列的操作
			cv::Mat rmWhite_image_canny;
			cv::Mat rotated_rmWhite_image(pyramid_templates[i].rows, pyramid_templates[i].cols, CV_8UC1);
			cv::Canny(pyramid_templates[i], rmWhite_image_canny, sensitity_threshold_low, sensitity_threshold_high);
			// 还是无法保证完全在图片框内
			// todo 客户端下发的bitmap也要旋转
			auto rotate_bitmap = rotateImage(pyramid_bitmaps[i], rotated_image_bmap, centers[i], j);
			auto rotate_matrix = rotateImage(pyramid_templates[i], rotated_image, centers[i], j);
			rotateImage(rmWhite_image_canny, rotated_rmWhite_image, centers[i], j);
			cv::threshold(rotated_rmWhite_image, rotated_rmWhite_image, 10, 255, CV_THRESH_BINARY);
			//Dilation(rotated_rmWhite_image, rotated_rmWhite_image, 3);

			//cv::imshow("rmWhite_image_canny", rotated_rmWhite_image);
			//cv::waitKey(0);

			//rotate_rect(rect, rotate_matrix);

			// 求一下该层当前角度对应下层角度的模板图(只针对顶层，一般情况下不考虑)
			IVSTemplateSubStruct pre_tpl;
			if (i == optimal_pyr_level - 1) {
				int angle_idx = (j + toolParameter.angleRange) / angle_steps[i - 1];
				if (angle_idx > tpls[i - 1].size() - 1) {
					angle_idx = tpls[i - 1].size() - 1;
				}
				pre_tpl = tpls[i - 1][angle_idx];
			}


			// todo 多传一个参数，旋转后的bitmap, 以及dobitwise_and的flag，只在高分辨率上做bitwiseand
			if (i <= 1) {
				doCreateTemplateIn(tpl, rotated_image, rotated_image_bmap, 1, sensitity_threshold_low,sensitity_threshold_high, rotated_rmWhite_image, i, optimal_pyr_level, pre_tpl);
			}
			else {
				doCreateTemplateIn(tpl, rotated_image, rotated_image_bmap, 0, sensitity_threshold_low,sensitity_threshold_high, rotated_rmWhite_image, i, optimal_pyr_level, pre_tpl);
			}
			//std::cout << "level: " << i << " num: " << tpl.noOfCordinates << std::endl;
			
			// 打印tpl图片信息
			//printTPLByImage(tpl);
			cur_level_tpl.push_back(tpl);
			//            draw_template(rotated_image, tpl);
			//cv::imshow(std::string("pyr") + std::string(1, i - '0'), rotated_image);
			//cvWaitKey(0);
		}
		tpls.push_back(cur_level_tpl);
	}
	//   tt.stop();
	//  std::cout << tt.duration() << "ms" << std::endl;
	std::cout << "create_template end" << std::endl;
#ifndef NDEBUG
	//    for (auto iter = tpls.cbegin(); iter != tpls.end(); ++iter) {
	//        std::cout << iter->at(0).noOfCordinates << std::endl;
	//    }
#endif
	//建立完模板需要将模板发送给客户端，需要发送的就是tpls这个数据结构

	templateInfo.pyramidLevelNum = optimal_pyr_level;
	templateInfo.searchAngelStep = angle_steps;
	// todo 换成move操作会好一些吧
	templateInfo.tpls = tpls;
	templateInfo.searchRectWidth = search_rect_width;
        templateInfo.templateIsOK=1;
	return 0;
}


/*
*  提供给客户端的接口
* */
int FeatureLocationTool::ivs_create_template()//, IVSTemplateStruct &ivSTemplateStruct
{

	// 获取灰度模板图
	cv::Mat template_roi;
	
	std::cout << "create_template" << std::endl;
	/* const AlgoPic pic
	cv::Mat yuvImg;
	cv::Mat template_image;

	switch (pic.rawDataFormat)
	{
	case YUV420SP:

		break;
	case YUV422SP:
		yuvImg.create(pic.height, pic.width, CV_8UC2);
		memcpy(yuvImg.data, pic.pRawDataAddr, pic.height*pic.width * 2);
		cv::cvtColor(yuvImg, template_image, CV_YUV2GRAY_YUYV);
		break;

	default:
		break;
	}

	
	cv::Mat template_roi_ext;
	

	// 不管圆形还是矩形，都是一样的操作
	// 将位图取出来，将原图外接矩形截取下来
	bitmapCleaned.create(ivsToolContourParameter.extRectHeight, ivsToolContourParameter.extRectWidth, CV_8UC1);



	// 从bitmap中恢复被擦除的位图
	FILE *fp = fopen((const char*)ivsToolContourParameter.templatePath, "rb");
	fseek(fp, 0L, SEEK_END);
	size_t eraseBitmapSize = ftell(fp);
	std::cout << "eraseBitmapSize is" << eraseBitmapSize << std::endl;
	rewind(fp);
	UINT8 *eraseBitmap = (UINT8 *)malloc(sizeof(UINT8)*eraseBitmapSize);
	fread(eraseBitmap, sizeof(UINT8), eraseBitmapSize, fp);
	fclose(fp);


	bitmap2Mat(bitmapCleaned, eraseBitmap, ivsToolContourParameter.extRectWidth, ivsToolContourParameter.extRectHeight);
	free(eraseBitmap);
	*/
	//cv::imshow("bitmapCleaned", bitmapCleaned);
	//cv::waitKey(0);

	// 从外接矩形位图中获取模板部分的位图
	template_roi = template_image(cv::Rect(toolParameter.extRectX, toolParameter.extRectY,
		toolParameter.extRectWidth, toolParameter.extRectHeight));
	cv::Mat bitmapCleaned(template_roi.size(), template_roi.type(), cv:: Scalar(255));
       
	// 保证两次截取出来的图大小一样
	//assert(template_roi.size == bitmapCleaned.size);

	// 使用截取出来的图片进行轮廓建立
	// 这之后擦除的代码不用改，保证这里传入的bitmap是对着的就行了
	doCreateTemplate(template_roi, bitmapCleaned);

	//// 打包后的template_data是unique_ptr上的指针，调用release来获取原始指针，但是要记得delete []这个内存
	//std::cout << "test pack template" << std::endl;

	//size_t buf_size = 0;
	//// 首先获取buf_size
	//packTemplate(ivSTemplateStruct, NULL, 0, &buf_size);
	//UINT8 *buf = (UINT8 *)malloc(sizeof(UINT8)*buf_size);
	//packTemplate(ivSTemplateStruct, buf, buf_size, NULL);

	//std::cout << "*****************" << buf_size << std::endl;

	//// 将模板保存成文件
	//fp = fopen((const char*)ivsToolContourParameter.templatePath, "wb");
	//if (!fp) {
	//	printf("file not found!\n");
	//	return -1;
	//}
	//fwrite(buf, sizeof(UINT8), buf_size, fp);
	//fclose(fp);
	//free(buf);

	std::cout << "now all done" << std::endl;


	return 0;
}


int FeatureLocationTool::ivs_get_contours(cv::Mat template_image, const IVSToolContourParameter& ivsToolContourParameter, UINT8 *contours) {

	/*const AlgoPic pic
	if (!pic.pRawDataAddr) {
		printf("图片不存在\n");
		return -1;
	}

	cv::Mat yuvImg;
	cv::Mat template_image;

	switch (pic.rawDataFormat)
	{
	case YUV420SP:

		break;
	case YUV422SP:
		yuvImg.create(pic.height, pic.width, CV_8UC2);
		memcpy(yuvImg.data, pic.pRawDataAddr, pic.height*pic.width * 2);
		cv::cvtColor(yuvImg, template_image, CV_YUV2GRAY_YUYV);
		break;
	default:
		break;
	}
	*/

	printf("ivs_get_contours执行");
	int low = ivsToolContourParameter.sensiLowThreshold;
	int top = ivsToolContourParameter.sensiTopThreshold;
	cv::GaussianBlur(template_image, template_image, cv::Size(3, 3), 0);
	cv::Mat contour;
	cv::Canny(template_image, contour, low, top);
	Dilation(contour, contour, 3);

	cv::imshow("contour", contour);
	//cv::waitKey(0);

	for (int i = 0; i < template_image.rows; ++i) {
		for (int j = 0; j < template_image.cols; ++j) {
			if (contour.at<uchar>(i, j)) {
				contours[i * template_image.cols + j] = 1;
			}
			else {
				contours[i * template_image.cols + j] = 0;
			}
		}
	}

	return 0;
}

void CreateFloatMatrix(float ***matrix, int width, int height) {

	(*matrix) = (float **)malloc(height * sizeof(float *));
	if (!*matrix) {
		printf("ERROR: out of memory...\n");
	}
	int i, j;
	for (i = 0; i < height; ++i) {
		(*matrix)[i] = (float *)malloc(width * sizeof(float));
		if (!(*matrix)[i]) {
			printf("ERROR: out of memory, trying to malloc, i: %d, %d * %zd...\n", i, width, sizeof(float));
		}
		for (j = 0; j < width; ++j) {
			(*matrix)[i][j] = -10;
		}
	}
}

// release memory
void ReleaseFloatMatrix(float ***matrix, int height) {
	//    for(int iInd = 0; iInd < size; iInd++)
	//        delete[] matrix[iInd];
	int i = 0;
	for (i = 0; i < height; ++i) {
		free((*matrix)[i]);
	}
	free(*matrix);
	*matrix = nullptr;
}

int FeatureLocationTool::createUtility(ContourUtility &contourUtility,int width, int height) {


	/* 分配各层金字塔图片梯度所需空间 */
	int s32Ret = 0;

	contourUtility.u16Height = height;
	contourUtility.u16Width = width;

	//ISize size;
	//size.width = LOOKUPTABLE_SIZE;
	//size.height = LOOKUPTABLE_SIZE;

	//CreateFloatMatrix(&(tool_utility->lookupTableX), size);
	//CreateFloatMatrix(&(tool_utility->lookupTableY), size);
	//CreateFloatMatrix(&(tool_utility->lookupTableS), size);

	//INT32 i, j;
	///* 初始化查找表 */
	//for (i = 0; i < LOOKUPTABLE_SIZE; i++) {
	//	for (j = 0; j < LOOKUPTABLE_SIZE; ++j) {
	//		float length;
	//		length = sqrtf(i * i + j * j);
	//		tool_utility->lookupTableS[i][j] = length;                  //强度查找表，用于计算二值化图
	//		tool_utility->lookupTableX[i][j] = 1.0f * (float)i / length;
	//		tool_utility->lookupTableY[i][j] = 1.0f * (float)j / length;
	//	}
	//}
	//tool_utility->lookupTableX[0][0] = 0;
	//tool_utility->lookupTableY[0][0] = 0;
	//tool_utility->lookupTableS[0][0] = 0;


	/* 各层金字塔运行时归一化梯度方向向量的存储表 */
	for (int i = 0; i < toolParameter.maxNumPyramid; i++) {
		int width = cvCeil((double)contourUtility.u16Width / (1 << i));
		int height = cvCeil((double)contourUtility.u16Height / (1 << i));    //金字塔建全局而不

		CreateFloatMatrix(&(contourUtility.edgeX[i]), width, height);	// create image to save gradient magnitude  values
		CreateFloatMatrix(&(contourUtility.edgeY[i]), width, height);	// create image to save gradient magnitude  values

	}
	return s32Ret;
}

//是否需要先将传入的src转换为U8C1
int FeatureLocationTool::computeUtility(ContourUtility &contourUtility,cv::Mat sample_image) {

        if(contourUtility.u16Width != sample_image.cols ||contourUtility.u16Height != sample_image.rows)
        {
            std::cout<<"Error:sample size error!"<<std::endl;
            throw std::exception();
        }
	// 将待测图片转换为opencv中Mat数据结构
	// 目前直接读入数据，之后再处理转换的问题
	// 读入灰度图
	/*AlgoPic ivsOriPic
	cv::Mat srcPic;
	cv::Mat yuvImg;
	
	switch (ivsOriPic.rawDataFormat)
	{
	case YUV420SP:

		break;
	case YUV422SP:
		yuvImg.create(ivsOriPic.height, ivsOriPic.width, CV_8UC2);
		memcpy(yuvImg.data, ivsOriPic.pRawDataAddr, ivsOriPic.height*ivsOriPic.width * 2);
		cv::cvtColor(yuvImg, srcPic, CV_YUV2GRAY_YUYV);
		break;

	default:
		break;
	}
	*/
	cv::Mat srcPic = sample_image.clone();
	contourUtility.u16Width = srcPic.cols;
	contourUtility.u16Height = srcPic.rows;

	int u16width = srcPic.cols;
	int u16height = srcPic.rows;

	// 滤波
        //contourUtility.srcFiltered=srcPic.clone();
	cv::GaussianBlur(srcPic, contourUtility.srcFiltered, cv::Size(5, 5), 3, 3);

	// 处理第0层的数据
	contourUtility.searchRegion[0] = contourUtility.srcFiltered.clone();
	cv::Sobel(contourUtility.searchRegion[0], contourUtility.gradx[0], CV_16S, 1, 0);
	cv::Sobel(contourUtility.searchRegion[0], contourUtility.grady[0], CV_16S, 0, 1);



	// 创建降采样的图像金字塔
	for (int l = 0; l < toolParameter.maxNumPyramid - 1; ++l) {

		cv::pyrDown(contourUtility.searchRegion[l], contourUtility.searchRegion[l + 1]);


		//计算sobel图片，并且归一化
		cv::Sobel(contourUtility.searchRegion[l + 1], contourUtility.gradx[l + 1], CV_16S, 1, 0);
		cv::Sobel(contourUtility.searchRegion[l + 1], contourUtility.grady[l + 1], CV_16S, 0, 1);


		/*cv::Mat tmpdx, tmpdy;
		convertScaleAbs(contourUtility.gradx[l + 1], tmpdx);
		convertScaleAbs(contourUtility.grady[l + 1], tmpdy);
		cv::imshow("dx", tmpdx);
		cv::imshow("dy", tmpdy);
		cv::waitKey(0);*/

	}

	// 计算edgeX和edgeY
	// 计算归一化的梯度edgeX,edgeY
	for (int i = 0; i < contourUtility.gradx[0].rows; i++) {

		for (int j = 0; j < contourUtility.gradx[0].cols; j++) {
			short fdx = contourUtility.gradx[0].at<short>(i, j);
			short fdy = contourUtility.grady[0].at<short>(i, j);

			double vector_length = sqrt(fdx * fdx + fdy * fdy);
			if (fdx != 0 || fdy != 0) {
				contourUtility.edgeX[0][i][j] = (double)fdx / vector_length;
				contourUtility.edgeY[0][i][j] = (double)fdy / vector_length;
			}
			else {
				contourUtility.edgeX[0][i][j] = 0.0f;
				contourUtility.edgeY[0][i][j] = 0.0f;
			}
		}
	}

	for (int l = 0; l < toolParameter.maxNumPyramid - 1; ++l) {
		// 计算归一化的梯度edgeX,edgeY
		for (int i = 0; i < contourUtility.gradx[l + 1].rows; i++) {

			for (int j = 0; j < contourUtility.gradx[l + 1].cols; j++) {
				short fdx = contourUtility.gradx[l + 1].at<short>(i, j);
				short fdy = contourUtility.grady[l + 1].at<short>(i, j);

				double vector_length = sqrt(fdx * fdx + fdy * fdy);
				if (fdx != 0 || fdy != 0) {
					contourUtility.edgeX[l + 1][i][j] = (double)fdx / vector_length;
					contourUtility.edgeY[l + 1][i][j] = (double)fdy / vector_length;
				}
				else {
					contourUtility.edgeX[l + 1][i][j] = 0.0f;
					contourUtility.edgeY[l + 1][i][j] = 0.0f;
				}
			}
		}
	}

	return 0;
}

int FeatureLocationTool::freeUtility(ContourUtility & contourUtility) {


	//ISize size;
	//size.width = LOOKUPTABLE_SIZE;
	//size.height = LOOKUPTABLE_SIZE;
	////SAMPLE_PRT("here.\n");
	///* 释放查找表 */
	//ReleaseFloatMatrix(&(tool_utility->lookupTableX), size.height);
	//ReleaseFloatMatrix(&(tool_utility->lookupTableY), size.height);
	//ReleaseFloatMatrix(&(tool_utility->lookupTableS), size.height);

	////SAMPLE_PRT("here.\n");
	/* 释放edgeX */
	/* 释放edgeY */
	/* 释放is_calculated */
	for (INT32 i = 0; i < toolParameter.maxNumPyramid; i++) {
		int width = cvCeil((double)contourUtility.u16Width / (1 << i));
		int height = cvCeil((double)contourUtility.u16Height / (1 << i));


		ReleaseFloatMatrix(&(contourUtility.edgeX[i]), height);
		ReleaseFloatMatrix(&(contourUtility.edgeY[i]), height);
	}

	return 0;
}

