
#include "stdafx.h"
#include "afxmt.h"
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include "Traffic_Camera_Image.h"
#include "ThreadImgProc.h"
#include "ThreadSocketClientTCP.h"
#include "imageProc.h"

using namespace cv;
using namespace imageProc;
CEvent eventPause(FALSE,TRUE);
CTraffic_Camera_Image sharedMem_Img_UI;
trafficSignRec trafficRecObj(0, 0);


UINT ThreadImgProc(LPVOID lpParam)
{
    ThreadImgProcInfo* pInfo =  (ThreadImgProcInfo*) lpParam;
	Mat currentFrm;
	std::vector<cv::Mat>* outputImgVP;
	std::vector<int>* outputIdxVP;
	
    //HWND* uiWindow = pInfo->hWnd;
	//fill the 4 window size to trafficRecObj
	std::vector<imgSize> inputSizeV;
	imgSize scaledTmp;
	CRect*  rectTmp; 

	inputSizeV.clear();
	
	//
	rectTmp = sharedMem_Img_UI.ReadCRect1();
	//CSize sz = rectTmp->Size();
	scaledTmp.cols = rectTmp->Width();   //sz.cx;
	scaledTmp.rows = rectTmp->Height();  //sz.cy;
	inputSizeV.push_back(scaledTmp);

	//
	rectTmp = sharedMem_Img_UI.ReadCRect2();
	//sz = rectTmp->Size();
	scaledTmp.cols = rectTmp->Width();//sz.cx;
	scaledTmp.rows = rectTmp->Height();//sz.cy;
	inputSizeV.push_back(scaledTmp);

	//
	rectTmp = sharedMem_Img_UI.ReadCRect3();
	//sz = rectTmp->Size();
	scaledTmp.cols = rectTmp->Width();//sz.cx;
	scaledTmp.rows = rectTmp->Height();//sz.cy;
	inputSizeV.push_back(scaledTmp);

	//
	rectTmp = sharedMem_Img_UI.ReadCRect4();
	//sz = rectTmp->Size();
	scaledTmp.cols = rectTmp->Width();//sz.cx;
	scaledTmp.rows = rectTmp->Height();//sz.cy;
	inputSizeV.push_back(scaledTmp);

	trafficRecObj.setOutputScale(inputSizeV);
    
    for(;;)
    {
        WaitForSingleObject(eventPause.m_hObject, INFINITE);
		if(sharedMem_Img_UI.ReadMatFromCamera(currentFrm)) //camera is open
		{
			if(currentFrm.empty())
			{
				break;
			}

			//Image process and detection
			trafficRecObj.imageProc(currentFrm);
			outputImgVP = trafficRecObj.getOutputImg();
			outputIdxVP = trafficRecObj.getOutputIdx();

			if(outputImgVP->size())
			{
				if(1 == outputImgVP->size())
				{
					sharedMem_Img_UI.loadImage(&(*outputImgVP)[0]);
				}

				if(2 == outputImgVP->size())
				{
					sharedMem_Img_UI.loadImage(&(*outputImgVP)[0],&(*outputImgVP)[1]);
				}

                if(3 == outputImgVP->size())
				{
					sharedMem_Img_UI.loadImage(&(*outputImgVP)[0],&(*outputImgVP)[1],&(*outputImgVP)[2]);
				}

                if(4 == outputImgVP->size())
				{
					sharedMem_Img_UI.loadImage(&(*outputImgVP)[0],&(*outputImgVP)[1],&(*outputImgVP)[2],&(*outputImgVP)[3]);
				}
			}
			//sharedMem_Img_UI.loadImage(&currentFrm,&currentFrm,&currentFrm,&currentFrm);
			::SendMessage(pInfo->hWnd,WM_DISPLAY,NULL,NULL);

			if(outputIdxVP->size())  //detect some traffic signal
			{
				send(g_ClientSockTCP,"detect traffic signal", 50, 0);
			}

		}else //camera is closed
		{
			break;
		}

    }

	inputSizeV.clear();
	return 0;
    
}