/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  NEMA_GPGGA_Proc.h
* @brief 
*
* Change Log:
*      Date                Who             What
*	   2015/01/09		  Yuan Zhang	  create
*******************************************************************************
*/
#pragma once

#include <stdio.h>
#include <WINBASE.H>

class CNemaGpggaInfo
{
public:
    int positioningState; // 0=unlocated£¬1=non-Differential position£¬2=Differential position£¬6=calculation
	char cLatitude;  //latitude
    char cLongitude;  //longitude
    double dLatitude;
    double dLongitude;
	double altitude;
	unsigned int st;
	int    inParamIdxs;

	int positioningStatePre;
	char cLatitudePre;  //latitude
    char cLongitudePre;  //longitude
    double dLatitudePre;
    double dLongitudePre;
	double altitudePre;
	unsigned int stPre;
	int    inParamIdxsPre;

	int positioningStatePrePre;
	char cLatitudePrePre;  //latitude
    char cLongitudePrePre;  //longitude
    double dLatitudePrePre;
    double dLongitudePrePre;
	double altitudePrePre;
	unsigned int stPrePre;
	int     inParamIdxsPrePre;
};

class CNEMA_GPGGA_PROC
{
public:
    int NEMA_GPGGA_checksum(char *gpsBuffer);
    void NEMA_GPGGA_parser(char *string);
	BOOL Set_NEMA_GPGGA_INFO(CNemaGpggaInfo &info);

private:
	int NEMA_GPGGA_ASCIIToHex(char ch);
    double CNEMA_GPGGA_PROC::Conv_GPGGA_to_GPS(char c, double gpgga);

	int GPS_status;
	char cLat;  //latitude
    char cLon;  //longitude
    double dLat;
    double dLon;
	double alti;
};

extern CNemaGpggaInfo gGpsInfo;