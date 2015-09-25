/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  configParse.cpp
* @brief the parameter config parser source file
*
* Change Log:
*      Date                Who             What
*      2015/9/7           QiuHeng		  Create
*******************************************************************************
*/
#include <sstream>
#include "configParse.h"

template <class Type>  
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}  

configParse::configParse(string cfgfilename):XMLParser(cfgfilename)
{
	//isload = false;
	//xmlparser = new XMLParser(cfgfilename);
	
	//isload = xmlparser->isLoad();
}


configParse::~configParse()
{
	
	//delete xmlparser;
}

/*bool configParse::isFileLoad() const
{
	return isload;
}*/

bool configParse::GetNetWorkCfg(OUT netWorkConfig &networkParamCfg)
{
	vector<string> values;
	int result = 0;

	//if(isload)
	if(isLoad())
	{
		//get server IP address
		result = getValues("/config/network/serverIP", &values);
		if(result!=1)
		return false;
		networkParamCfg.serverAddr.sin_addr.S_un.S_addr = inet_addr(values.at(0).c_str());
		
		//get server port
		int sPort = 0;
		result = getValues("/config/network/serverPort", &values);
		if(result!=1)
		return false;
		sPort = stringToNum<int>(values.at(1));
		networkParamCfg.serverAddr.sin_port = htons(sPort);
		
		//set server protocol family
		networkParamCfg.serverAddr.sin_family = AF_INET;

		//get client port
		int cPort = 0;
		result = getValues("/config/network/clientPort", &values);
		if(result!=1)
		return false;
		cPort = stringToNum<int>(values.at(2));
		networkParamCfg.clientPort = htons(cPort);
		
		//get vehicle ID
		result = getValues("/config/network/vehicleId", &values);
		if(result!=1)
		return false;
		networkParamCfg.VehicleID = stringToNum<int>(values.at(3));
		
		//get emulator IP
		result = getValues("/config/network/emulatorIP", &values);
		if(result!=1)
		return false;
		networkParamCfg.EmulatorIP = inet_addr(values.at(4).c_str());
		
		//get emulator port
		int ePort = 0;
		result = getValues("/config/network/emulatorPort", &values);
		if(result!=1)
		return false;
		ePort = stringToNum<int>(values.at(5));
		networkParamCfg.EmulatorPort = htons(ePort);
		
		//get gpsPort port
		int gPort = 0;
		result = getValues("/config/network/gpsPort", &values);
		if(result!=1)
		return false;
		gPort = stringToNum<int>(values.at(6));
		networkParamCfg.GpsPort = htons(gPort);
	}
	else
	{
		cout<<"the file wasn't loaded"<<endl;
		return false;
	}
	
	return true;
}

bool configParse::getAlgorithmCfg(OUT Parameters &algorithmCfg)
{
	vector<string> values;
	int result = 0;

	//if(isload)
	if(isLoad())
	{
		//get gps reference point
		result = getValues("/config/common/GPSref/value", &values);
		if(result!=1)
		return false;
		algorithmCfg.GPSref.x = stringToNum<double>(values.at(0));
		algorithmCfg.GPSref.y = stringToNum<double>(values.at(1));
		
		//get centerPoint point
		result = getValues("/config/algorithm/centerPoint/value", &values);
		if(result!=1)
		return false;
		algorithmCfg.centerPoint.x = stringToNum<int>(values.at(2));
		algorithmCfg.centerPoint.y = stringToNum<int>(values.at(3));
		
		//get lengthRate point
		result = getValues("/config/algorithm/lengthRate", &values);
		if(result!=1)
		return false;
		algorithmCfg.lengthRate = stringToNum<double>(values.at(4));
		
		//get distanceOfSlantLeft
		result = getValues("/config/algorithm/distanceOfSlantLeft", &values);
		if(result!=1)
		return false;
		algorithmCfg.distanceOfSlantLeft = stringToNum<int>(values.at(5));
		
		//get distanceOfSlantRight
		result = getValues("/config/algorithm/distanceOfSlantRight", &values);
		if(result!=1)
		return false;
		algorithmCfg.distanceOfSlantRight = stringToNum<int>(values.at(6));
		
		//get distanceOfUpMove
		result = getValues("/config/algorithm/distanceOfUpMove", &values);
		if(result!=1)
		return false;
		algorithmCfg.distanceOfUpMove = stringToNum<int>(values.at(7));
		
		//get distanceOfLeft
		result = getValues("/config/algorithm/distanceOfLeft", &values);
		if(result!=1)
		return false;
		algorithmCfg.distanceOfLeft = stringToNum<int>(values.at(8));
		
		//get distanceOfRight
		result = getValues("/config/algorithm/distanceOfRight", &values);
		if(result!=1)
		return false;
		algorithmCfg.distanceOfRight = stringToNum<int>(values.at(9));
		
		//get ridgeThreshold
		result = getValues("/config/algorithm/ridgeThreshold", &values);
		if(result!=1)
		return false;
		algorithmCfg.ridgeThreshold = stringToNum<int>(values.at(10));
		
		//get stretchRate
		result = getValues("/config/algorithm/stretchRate", &values);
		if(result!=1)
		return false;
		algorithmCfg.stretchRate = stringToNum<int>(values.at(11));
		
		//get downSampling
		result = getValues("/config/algorithm/downSampling", &values);
		if(result!=1)
		return false;
		algorithmCfg.downSampling = stringToNum<int>(values.at(12));
		
		//get distancePerPixel
		result = getValues("/config/algorithm/distancePerPixel", &values);
		if(result!=1)
		return false;
		algorithmCfg.distancePerPixel = stringToNum<double>(values.at(13));
		
		//get imageScaleHeight
		result = getValues("/config/algorithm/imageScaleHeight", &values);
		if(result!=1)
		return false;
		algorithmCfg.imageScaleHeight = stringToNum<double>(values.at(14));
		
		//get imageScaleWidth
		result = getValues("/config/algorithm/imageScaleWidth", &values);
		if(result!=1)
		return false;
		algorithmCfg.imageScaleWidth = stringToNum<double>(values.at(15));
		
		//get imageRows
		result = getValues("/config/algorithm/imageRows", &values);
		if(result!=1)
		return false;
		algorithmCfg.imageRows = stringToNum<int>(values.at(16));
		
		//get imageCols
		result = getValues("/config/algorithm/imageCols", &values);
		if(result!=1)
		return false;
		algorithmCfg.imageCols = stringToNum<int>(values.at(17));
		
		//get discardRoadDataAfterLaneChange
		result = getValues("/config/algorithm/discardRoadDataAfterLaneChange", &values);
		if(result!=1)
		return false;
		algorithmCfg.discardRoadDataAfterLaneChange = stringToNum<bool>(values.at(18));
	}
	else
	{
		cout<<"the file wasn't loaded"<<endl;
		return false;
	}

	return true;
}

bool configParse::getOverViewPoint(OUT eyeLookAt_t &eye)
{
	vector<string> values;
	int result = 0;

	//if(isload)
	if(isLoad())
	{
		//get overViewPoint
		result = getValues("/config/visualization/overViewPoint/value", &values);
		if(result!=1)
		return false;
		eye.eyePosition.x = stringToNum<GLfloat>(values.at(0));
		eye.eyePosition.y = stringToNum<GLfloat>(values.at(1));
		eye.eyePosition.z = stringToNum<GLfloat>(values.at(2));
		//set  lookatPosition
		eye.lookatPosition.x = eye.eyePosition.x;
		eye.lookatPosition.y = 0;
		eye.lookatPosition.z = eye.eyePosition.z;
	}
	else
	{
		cout<<"the file wasn't loaded"<<endl;
		return false;
	}
	
	return true;
}

bool configParse::getCameraPort(OUT int &cameraPort)
{
	vector<string> values;
	int result = 0;

	//if(isload)
	if(isLoad())
	{
		//get cameraPort
		result = getValues("/config/datasource/cameraPort", &values);
		if(result!=1)
		return false;

		cameraPort = stringToNum<int>(values.at(0));
	}
	else
	{
		cout<<"the file wasn't loaded"<<endl;
		return false;
	}

	return true;
}


#if(RD_USE_CAMERA == OFF) 
bool configParse::getAviGpsFileList(OUT AviAndGpsFiles &aviAndgpsFlist)
{
	int result = 0;
	vector<string> values;
	uint32 fileNumber;
	uint32 nameLength;
	int index = 0;

	//if(isload)
	if(isLoad())
	{
		 result = getValues("/config/datasource/avigpsfile/value", &values);
		 if(result != 1)
		 {
			 cout<<" get the avigpsfile value fail!!! "<<endl;
			 return false;
		 }
		 
		 fileNumber = (uint32)values.size();
		 if(fileNumber > MAX_AVIGPS_FIlE_NUMBER)
		 fileNumber = MAX_AVIGPS_FIlE_NUMBER;

		 for(index = 0; index < fileNumber; index++)
		 {	
			 
			 nameLength = (uint32)values[index].size();
			 if(nameLength > (MAX_FIlENAME_lENGTH -1))
			 {
				 cout<<" The length of file "<<values[index]<<" is out of range!!!"<<endl;
				 return false;
			 }	
			
			 if((index &0x1) == 0)
			 {
				 //copy string to char array aviNameList
				 values[index].copy(aviAndgpsFlist.aviNames[index>>1], nameLength, 0);
				 aviAndgpsFlist.aviNames[index>>1][nameLength] = '\0';				 				 
			 } 
			 else
			 {	 
				 //copy string to char array gpsNameList
				 values[index].copy(aviAndgpsFlist.gpsNames[index>>1], nameLength, 0);
				 aviAndgpsFlist.gpsNames[index>>1][nameLength] = '\0';	
			 } 	 	 
		 }	
		 aviAndgpsFlist.numFiles = index>>1;
	}
	else
	{
		cout<<"the file wasn't loaded"<<endl;
		return false;
	}

	return true;
}

#endif