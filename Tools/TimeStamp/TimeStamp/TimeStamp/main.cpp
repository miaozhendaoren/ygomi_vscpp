#include <iostream>   
#include "TimeStamp.h"

int main(int argc, char* argv[])
{
    int i,j;
    TimeStampInit();
    for(i=0;i<100;i++)
    {
        //delay
        for(j=0;j<10000000;j++){}
        TimeStamp(i%8 , 8);
    }
    TimeStampExit();

    return 0;
}