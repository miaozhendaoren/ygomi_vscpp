#include <stdio.h>
//#include <windows.h>
#include "winsock2.h"

#pragma comment(lib,"ws2_32.lib")

typedef struct
{
	float latitude;
	float longitude;
	float altitude; 
}GPS_Point_t;

GPS_Point_t GpsVector[1000];
char sendBuffer[100];
int numberGps;
int gpsIdx = 0;

SOCKET sockSend;
SOCKADDR_IN clientAddr;

int geneGPGGAData(GPS_Point_t *point,char* output)
{
    char cLat;
    char cLon;
    float lat;
    float lon;
	int ddLat;
    float mmLat;
    int ddLon;
    float mmLon;
    int checkSum = 0;
    int index;
	int lengthHeight = 0;
	float height = point->altitude;
  
    if(point->latitude > 0)
    {
        cLat = 'N';
        lat = point->latitude;
    }else
    {
        cLat = 'S';
        lat = -(point->latitude);
    }
    
    if(point->longitude > 0)
    {
        cLon = 'E';
        lon = point->longitude;
    }else
    {
        cLon = 'W';
        lon = -(point->longitude);
    }
    
    ddLat = (int)lat;
    mmLat = (lat - ddLat)*60;
    ddLon = (int)lon;
    mmLon = (lon - ddLon)*60;
    
    //$GPGGA, UTCtime,
    memcpy((void*)output,"$GPGGA,071112.324,",18);
    
    //latitude
    sprintf(&output[18],"%2d",ddLat);
    sprintf(&output[20],"%2.4f",mmLat);
    memcpy(&output[27],",",1);
    
    //
    memcpy(&output[28],&cLat,1);
    memcpy(&output[29],",",1);

	//
	sprintf(&output[30],"%2d",ddLon);
    sprintf(&output[32],"%2.4f",mmLon);
    memcpy(&output[39],",",1);

	//longitude E/W
	memcpy(&output[40],&cLon,1);
    memcpy(&output[41],",",1);

	//
	memcpy(&output[42],"8,",2);

	//satillate number
	memcpy(&output[44],"07,",3);

	//
	memcpy(&output[47],"2.0,",4);

	//height
	if(height < 0)
	{
		memcpy(&output[51],"-",1);
		lengthHeight += 1;
		height = -height;
	}
	sprintf(&output[51+lengthHeight],"%4.1f",height);
	
	lengthHeight += 3;

	if(height >= 10.0f)
	{
		lengthHeight+= 1;
	}

	if(height >= 100.0f)
	{
		lengthHeight+= 1;
	}

	if(height >= 1000.0f)
	{
		lengthHeight+= 1;
	}
	
    memcpy(&output[51+lengthHeight],",",1);

	//
    memcpy(&output[52+lengthHeight],"M,",2);
    
    //
    memcpy(&output[54+lengthHeight],"-34.0,",6);
    
    //
    memcpy(&output[60+lengthHeight],"M,",2);
    
    //
    memcpy(&output[62+lengthHeight],",*",2);
    
    //checksum
    for(index = 1; index < (63+lengthHeight); index++)
    {
        checkSum ^= output[index];
    }    
    sprintf(&output[64+lengthHeight],"%2x",checkSum);
    
    //
    memset(&output[66+lengthHeight],10,1);
    
    return 67+lengthHeight;

}


int InitSocket(void)
{
    FILE* f = fopen(".//Config//SendIP.txt", "r");
    char buff[100];
    int nPort;
    int nRet;
    
    // Initialize WinSock and check version  
	WORD wVersionRequested = MAKEWORD(2,2);  
    WSADATA wsaData;   
    
    nRet = WSAStartup(wVersionRequested, &wsaData); 
    
    if (wsaData.wVersion != wVersionRequested)  
    {     
        fprintf(stderr,"\n Wrong version\n");  
        return 255;  
    }  
    
    fscanf(f,"%s",buff);//server IP
	fscanf(f,"%d",&nPort);// server port
	fclose(f);
    
    sockSend = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if (sockSend == INVALID_SOCKET)  
    {  
        printf("creating socket failed!\n");  
        return 255;  
    }

    clientAddr.sin_family           = AF_INET; 
	//clientAddr.sin_addr.S_un.S_addr = inet_addr(buff);//(ULONG)&buff[0];
	//clientAddr.sin_port             = atoi(nPort);
	clientAddr.sin_addr.s_addr = inet_addr(buff);
	clientAddr.sin_port = htons((short)nPort);

	//nRet = sendto(sockSend,sendBuffer,6,0,(SOCKADDR*)&clientAddr, sizeof(SOCKADDR));

    return 0;
    
}

int load_vector_data(const char* file_name, GPS_Point_t* buffer)
{
	FILE* pFile = fopen(file_name, "r");
	int index = 0;
    fseek(pFile, 0, SEEK_SET);

	while(3 == fscanf(pFile,"%f,%f,%f\n",&(buffer[index].latitude),&(buffer[index].longitude),&(buffer[index].altitude)))
	{
		index++;
	}

	fclose(pFile);

	return index;
}

int main(int argc, char* argv[])
{
    
    numberGps = load_vector_data(".//Config//road_data_middle.txt",GpsVector);
    
    InitSocket();
   
    while(1)
	{
		int nSend;
		int length;
		
		
        length = geneGPGGAData( &GpsVector[gpsIdx],sendBuffer);
        
        if(gpsIdx >= (numberGps-1))
        {
            gpsIdx = 0;
        }else
        {
            gpsIdx++;
        }
        
		nSend = sendto(sockSend,sendBuffer,length,0,(SOCKADDR*)&clientAddr, sizeof(SOCKADDR));

		Sleep(1000);
	}
    
    
}