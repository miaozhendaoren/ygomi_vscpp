/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  NEMA_GPGGA_Proc.cpp
* @brief 
*
* Change Log:
*      Date                Who             What
*	   2015/01/09		  Yuan Zhang	  create
*******************************************************************************
*/
#include <stdio.h>
#include <Windows.h> 
#include "NEMA_GPGGA_Proc.h"

CNemaGpggaInfo gGpsInfo;

/*void saveGpsData(CNemaGpggaInfo &info)
{
	FILE *fp;
	errno_t err;
	
	err = fopen_s(&fp, "../InVehicleDemo_NewCo/config/NEMA_GPS_data.txt", "at");
	if(NULL != err)
	{
		return;
	}
	else
	{
		fprintf_s(fp, "%f, %f, %f\n", info.dLatitude, info.dLongitude, info.altitude);
	}

	fflush(fp);
	fclose(fp);

	return;
}*/

int CNEMA_GPGGA_PROC::NEMA_GPGGA_ASCIIToHex(char ch)
{
	if((ch >= '0')&&(ch <= '9'))
	{
		return (int)(ch - 0x30);
	}
	else if((ch >= 'A')&&(ch <= 'F'))
	{
		return (int)(ch - 'A' + 10);
	}
	else if((ch >= 'a')&&(ch <= 'f'))
	{
		return (int)(ch - 'a' + 10);
	}
	else
	{
		return -1;
	}
}

int CNEMA_GPGGA_PROC::NEMA_GPGGA_checksum(char *gpsBuffer)
{
	int i, result = 0, check = 0;
	if('$' != gpsBuffer[0])
	{
		return 0;
	}
	for(result=gpsBuffer[1], i=2; gpsBuffer[i] != '*'; i++)
	{
		result^=gpsBuffer[i];
	}
	check = (NEMA_GPGGA_ASCIIToHex(gpsBuffer[i+1])<<4) + (NEMA_GPGGA_ASCIIToHex(gpsBuffer[i+2]));

	if(check != result)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
/*
<0>:$GPGGA
<1>:085014.955
<2>:2839.2050   latitude
<3>:N
<4>:11549.5721  longitude
<5>:E
<6>:1
<7>:04
<8>:03.6
<9>:76.6    altitude
<10>:M
<11>:-6.2
<12>:M
<13>:0
<14>:*4C
*/
void CNEMA_GPGGA_PROC::NEMA_GPGGA_parser(char *string)
{
    char block[50] = {0};  
    unsigned int i = 0;   
    int j = 0;  
    int blockNum = 0;

    if(strlen(string) < 1)  
        return;

	memset(block, 0, sizeof(block));
    // start
    if(string[0] == '$')  
    {
        while(i <= strlen(string))  
        {              
            while(string[i] != ',')  
            {
                if(i > strlen(string))  
                    break;

                block[j++] = string[i++];

                if(j >= sizeof(block))  
                    break;                
            }
            if(string[i-1] == ',')
            {  
                block[0] = '0';
            }  
            //printf("Current Block <%d>:%s\n", blockNum, block);
            //pass comma 
            i++;
            // start from block[0]
            j = 0;
            
            switch(blockNum)
            {  
                case 0: // GPGGA
					break;
                  
                case 1:
					break;
                  
                case 2:
					dLat = atof(block);
					break;

				case 3:
					cLat = block[0];
					break;

				case 4:
					dLon = atof(block);
					break;

				case 5:
					cLon = block[0];
					break;

				case 6:
					GPS_status = atoi(block);
					break;

				case 9:
					alti = atof(block);
					break;
            }
            memset(block, 0, sizeof(block));
            blockNum++;  
        }
		Set_NEMA_GPGGA_INFO(gGpsInfo);

		//saveGpsData(gGpsInfo);
    }
}

double CNEMA_GPGGA_PROC::Conv_GPGGA_to_GPS(char c, double gpgga)
{
    double DD = (double)((int)(gpgga / 100));
    double MM = gpgga - DD * 100;
    double gpsAbs = (DD + MM / 60);
    double gps;

    switch(c)
    {
        case 'N':
            gps = gpsAbs;
            break;
        case 'S':
            gps = -1 * gpsAbs;
            break;
        case 'E':
            gps = gpsAbs;
            break;
        case 'W':
            gps = -1 * gpsAbs;
            break;
        default:
            // error
            break;
    }

    return gps;
}

BOOL CNEMA_GPGGA_PROC::Set_NEMA_GPGGA_INFO(CNemaGpggaInfo &info)
{
	info.positioningStatePrePre = info.positioningStatePre;
	info.cLatitudePrePre        = info.cLatitudePre;  //latitude
    info.cLongitudePrePre       = info.cLongitudePre;  //longitude
    info.dLatitudePrePre        = info.dLatitudePre;
    info.dLongitudePrePre       = info.dLongitudePre;
	info.altitudePrePre         = info.altitudePre;

	info.positioningStatePre = info.positioningState;
	info.cLatitudePre        = info.cLatitude;  //latitude
    info.cLongitudePre       = info.cLongitude;  //longitude
    info.dLatitudePre        = info.dLatitude;
    info.dLongitudePre       = info.dLongitude;
	info.altitudePre         = info.altitude;

	info.positioningState    = GPS_status;
	info.cLatitude           = 0;  //Not used
    info.cLongitude          = 0;  //Not used
    info.dLatitude           = Conv_GPGGA_to_GPS(cLat, dLat);
    info.dLongitude          = Conv_GPGGA_to_GPS(cLon, dLon);
	info.altitude            = 0;// Always set to 0.  Previously: alti;

	return TRUE;
}


