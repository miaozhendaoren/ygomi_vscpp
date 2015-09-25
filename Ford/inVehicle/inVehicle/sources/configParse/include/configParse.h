/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file configParse.h
* @brief  config parameter parse header file
*
*
* Change Log:
*      Date                Who             What
*	   2015/09/07		  qiuheng   	  create
*******************************************************************************
*/
#pragma once
#include "XMLParser.h"
#include <string>
#include "typeDefine.h"
#include "roadScan.h"
#include "VisualizeControl.h"
#include "AppInitCommon.h"

using namespace std;
using ns_roadScan::Parameters;

template <class Type>  
Type stringToNum(const string& str); 


class configParse:public XMLParser
{
	public:
		configParse(string cfgfilename);
		~configParse();
		//bool isFileLoad() const;		
		
		bool GetNetWorkCfg(OUT netWorkConfig &);
		bool getAlgorithmCfg(OUT Parameters &);
		bool getOverViewPoint(OUT eyeLookAt_t &);
		bool getCameraPort(OUT int &);
		#if(RD_USE_CAMERA == OFF) 
		bool configParse::getAviGpsFileList(OUT AviAndGpsFiles &);		
		#endif			

	private:
		//XMLParser *xmlparser;
		//bool isload;
};




