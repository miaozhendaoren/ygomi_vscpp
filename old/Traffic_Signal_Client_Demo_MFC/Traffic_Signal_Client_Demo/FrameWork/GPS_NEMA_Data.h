#pragma once

#include <stdio.h>
#include <vector>

class GPGGA_Info
{
public:
	char cLat;  //latitude
    char cLon;  //longitude
    double dLat;
    double dLon;
	double altitude;
};

class GPS_NEMA_Data
{
public:
    int GetSubStrCount(CString str);
    std::vector<CString> GetSubStr(CString str);
    int ComputeCheckSum(CString str);
	int ASCIIToHex(char ch);
    
};

class GPS_NEMA_GPGGA_Data :public GPS_NEMA_Data 
{
public:
    GPS_NEMA_GPGGA_Data();
    BOOL dataProc(CString str);
	BOOL getGPGGAInfo(GPGGA_Info &info);

private:
	int CStringToHex(CString str);
	CMutex mutex; //
	char cLat;  //latitude
    char cLon;  //longitude
    double dLat;
    double dLon;
	double altitude;

};

