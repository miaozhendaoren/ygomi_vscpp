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

using namespace cv;

CTraffic_Camera_Image::CTraffic_Camera_Image()
{
	cImageIdx = 0;
}

CTraffic_Camera_Image::~CTraffic_Camera_Image()
{
    //close captRefrnc
    //captRefrnc
    if(captRefrnc.isOpened())
    {
        captRefrnc.release();
    }
    //close MainPicture
}

BOOL CTraffic_Camera_Image::OpenCameraDevice()
{
    if(captRefrnc.isOpened())
    {
        captRefrnc.release();
    }
    captRefrnc.open(0);
    return captRefrnc.isOpened();
}

BOOL CTraffic_Camera_Image::CloseCameraDevice()
{
	if(captRefrnc.isOpened())
    {
        captRefrnc.release();
		return TRUE;
    }
	return FALSE;
}
BOOL CTraffic_Camera_Image::ReadMatFromCamera(Mat& image)
{
    if(captRefrnc.isOpened())
    {
        captRefrnc.read(image);
        return TRUE;
    }
    return FALSE;
}

CImage* CTraffic_Camera_Image::ReadCImage1()
{
	if(cImage1[cImageIdx^1].IsNull())
	{
		return NULL;
	}
	return &cImage1[cImageIdx^1];
}

CImage* CTraffic_Camera_Image::ReadCImage2()
{
	if(cImage2[cImageIdx^1].IsNull())
	{
		return NULL;
	}
	return &cImage2[cImageIdx^1];
}

CImage* CTraffic_Camera_Image::ReadCImage3()
{
	if(cImage3[cImageIdx^1].IsNull())
	{
		return NULL;
	}
	return &cImage3[cImageIdx^1];
}

CImage* CTraffic_Camera_Image::ReadCImage4()
{
	if(cImage4[cImageIdx^1].IsNull())
	{
		return NULL;
	}
	return &cImage4[cImageIdx^1];
}

void CTraffic_Camera_Image::MatToCImage(Mat &mat, CImage &cImage)
{
//create new CImage  
int width    = mat.cols;  
int height   = mat.rows;  
int channels = mat.channels();  
	
//cImage.ReleaseDC();
//cImage.ReleaseGDIPlus();
cImage.Destroy(); //clear  
cImage.Create(width,   
              height, //positive: left-bottom-up   or negative: left-top-down  
              8*channels ); //numbers of bits per pixel  

 //copy values  
uchar* ps;  
uchar* pimg = (uchar*)cImage.GetBits(); //A pointer to the bitmap buffer  
          
    //The pitch is the distance, in bytes. represent the beginning of   
    // one bitmap line and the beginning of the next bitmap line  
    int step = cImage.GetPitch();  

	for (int i = 0; i < height; ++i)  
	{  
        ps = (mat.ptr<uchar>(i));  
        for ( int j = 0; j < width; ++j )  
        {  
            if ( channels == 1 ) //gray  
            {  
                *(pimg + i*step + j) = ps[j];  
            }  
            else if ( channels == 3 ) //color  
            {  
                for (int k = 0 ; k < 3; ++k )  
                {  
                    *(pimg + i*step + j*3 + k ) = ps[j*3 + k];  
                }             
            }  
        }     
    }  

}

void CTraffic_Camera_Image::setPicCRect(CRect& mainRect,CRect& upRect,CRect& centerRect,CRect& downRect)
{
	cRect1 = mainRect;
	cRect2 = upRect;
	cRect3 = centerRect;
	cRect4 = downRect;
}

CRect*  CTraffic_Camera_Image::ReadCRect1()
{
	return &cRect1;
}

CRect*  CTraffic_Camera_Image::ReadCRect2()
{
	return &cRect2;
}

CRect*  CTraffic_Camera_Image::ReadCRect3()
{
	return &cRect3;
}

CRect*  CTraffic_Camera_Image::ReadCRect4()
{
	return &cRect4;
}

void CTraffic_Camera_Image::loadImage(Mat *mat1,Mat *mat2, Mat *mat3, Mat *mat4)
{
    if(~cImage1[cImageIdx^1].IsNull())
    {
        cImage1[cImageIdx^1].Destroy();
    }

    if(~cImage2[cImageIdx^1].IsNull())
    {
        cImage2[cImageIdx^1].Destroy();
    }

    if(~cImage3[cImageIdx^1].IsNull())
    {
        cImage3[cImageIdx^1].Destroy();
    }

    if(~cImage4[cImageIdx^1].IsNull())
    {
        cImage4[cImageIdx^1].Destroy();
    }

	if(!mat1->empty())
	{
		MatToCImage(*mat1, cImage1[cImageIdx]);
	}

	if(NULL != mat2)
	{
		if(!mat2->empty())
		{
			MatToCImage(*mat2, cImage2[cImageIdx]);
		}
	}

	if(NULL != mat3)
	{
		if(!mat3->empty())
		{
			MatToCImage(*mat3, cImage3[cImageIdx]);
		}
	}

	if(NULL != mat4)
	{
		if(!mat4->empty())
		{
			MatToCImage(*mat4, cImage4[cImageIdx]);
		}
	}
	cImageIdx ^= 1;
}