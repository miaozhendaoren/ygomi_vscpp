#include "stdafx.h"
#include "afxmt.h"
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include "GPS_NEMA_Data.h"

int GPS_NEMA_Data::GetSubStrCount(CString str)
{
     char cFlag1 = ',';
     char cFlag2 = '*';
     int  count = 0;
     BOOL isHas = FALSE;
     
     for(int iStart = -1; \
     -1!= (iStart = str.Find(cFlag1, iStart+1)); count++)
     {
        isHas = TRUE;
     }
     
     if(-1 != str.Find(cFlag2, 0))
     {
        count++;
     }
     
     if(isHas)
     {
        return count+1;
     }else
     {
        return 0;
     }
}

std::vector<CString> GPS_NEMA_Data::GetSubStr(CString str)
{
    char cFlag1=',';
    char cFlag2='*';
    int total = GetSubStrCount(str);
    int idx = 0;
    int iStart = -1;
    int iEnd   = -1;
    std::vector<CString> SubStrV;
    CString tmp;
    
    // the first n-2 sub string
    for(idx = 0; idx < (total-2); idx++)
    {
        iEnd = str.Find(cFlag1, iStart+1);
        tmp = str.Mid(iStart+1,(iEnd - iStart - 1));
        SubStrV.push_back(tmp);
        iStart = iEnd;
    }
    
    //the last two sub string
    iEnd = str.Find(cFlag2, iStart+1);
    tmp = str.Mid(iStart+1,(iEnd - iStart - 1));
    SubStrV.push_back(tmp);
    iStart = iEnd;
    
    iEnd = str.GetLength()-iStart-1;
    tmp = str.Mid(iStart+1, iEnd);
    SubStrV.push_back(tmp);
    
    return SubStrV;
}

int GPS_NEMA_Data::ComputeCheckSum(CString str)
{
    char cFlag1='$';
    char cFlag2='*';
    int iStart = str.Find(cFlag1, 0);
    int iEnd   = str.Find(cFlag2, 0);
    int checkSum = 0;
    
    char *p = (LPSTR)(LPCTSTR)str;
    
    for(int i = (iStart+1); i < iEnd; i++)
    {
        checkSum ^= p[i];
    }
    
    return checkSum;
}

int GPS_NEMA_Data::ASCIIToHex(char ch)
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
GPS_NEMA_GPGGA_Data::GPS_NEMA_GPGGA_Data()
{
    cLat = 'N';
    cLon = 'E';
    dLat = 0.0;
    dLon = 0.0;
}

int GPS_NEMA_GPGGA_Data::CStringToHex(CString str)
{
	int total = 0;
	//int length = str.GetLength();
	char *p = (LPSTR)(LPCTSTR)str;

	for(int i = 0; i < 2; i++)
	{
		int out = ASCIIToHex(p[i]);
		total = (total<<4) + out;
	}
	return total;

}
BOOL GPS_NEMA_GPGGA_Data::dataProc(CString str)
{
    int numStr = GetSubStrCount(str);
    std::vector<CString> subStrV = GetSubStr(str);
    
    //only process longtitude and latitude information.
    
    if("$GPGGA" == subStrV[0])
    {
        //check the checksum
		if(subStrV.size() != 16)
		{
			return FALSE;
		}

        int computeCheckSum = ComputeCheckSum(str);
		int checkSum = CStringToHex(subStrV[15]);

		//compare the checksum result.
		if(computeCheckSum != checkSum)
		{

			return FALSE;
		}
        
		mutex.Lock();
        dLat = atof(subStrV[2]);
        dLon = atof(subStrV[4]); 
		altitude = atof(subStrV[9]);
        
        if('N' == subStrV[3])
        {
            cLat = 'N';
        }else if('S' == subStrV[3])
        {
            cLat = 'S';
        }else
        {
            return FALSE;
        }
        
        if('E' == subStrV[5])
        {
            cLon = 'E';
        }else if('W' == subStrV[5])
        {
            cLon = 'W';
        }else
        {
            return FALSE;
        }
        
		mutex.Unlock();
        return TRUE;
    }
    
    return FALSE;
}

BOOL GPS_NEMA_GPGGA_Data::getGPGGAInfo(GPGGA_Info &info)
{
	mutex.Lock();

	info.altitude = altitude;
	info.cLat     = cLat;
	info.cLon     = cLon;
	info.dLat     = dLat;
	info.dLon     = dLon;

	mutex.Unlock();

	return TRUE;
}