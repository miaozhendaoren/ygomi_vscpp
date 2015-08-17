#pragma once

#include "atlimage.h"
#include "Traffic_Camera_Image.h"

#define WM_DISPLAY  WM_USER+2

struct ThreadImgProcInfo
{
     HWND hWnd;  
};

extern CEvent eventPause;
extern CTraffic_Camera_Image sharedMem_Img_UI;
UINT ThreadImgProc(LPVOID lpParam);