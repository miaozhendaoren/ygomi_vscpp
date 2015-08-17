#pragma once
#include <stdio.h>

extern "C"
{
void Composit(long int np,double **coplImage,double **copl,double fLength,double POSITRot1[3][3],double POSITTrans1[3],double POSITRot2[3][3],double POSITTrans2[3]);
}