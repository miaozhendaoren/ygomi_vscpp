
#pragma once

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O

//this class is used for communication with UI thread and image color/shape detection thread
class CTraffic_Camera_Image
{
public:
    CTraffic_Camera_Image();
    ~CTraffic_Camera_Image();
               
    cv::VideoCapture captRefrnc;  //the video capture 

	void loadImage(cv::Mat *mat1,
		cv::Mat *mat2 = NULL, 
		cv::Mat *mat3 = NULL, 
		cv::Mat *mat4 = NULL
		);

	void setPicCRect(CRect& mainRect,CRect& upRect,CRect& centerRect,CRect& downRect);


    BOOL OpenCameraDevice();
	BOOL CloseCameraDevice();
    BOOL ReadMatFromCamera(cv::Mat& image);
	CImage* ReadCImage1();
	CImage* ReadCImage2();
	CImage* ReadCImage3();
	CImage* ReadCImage4();
	CRect*  ReadCRect1();
	CRect*  ReadCRect2();
	CRect*  ReadCRect3();
	CRect*  ReadCRect4();
    
protected:

private:
	int cImageIdx;
	CImage cImage1[2];     //picture of main window
	CImage cImage2[2];    //picture of up window
	CImage cImage3[2];    //picture of center window
	CImage cImage4[2];    //picture of down window
	void MatToCImage(cv::Mat &mat, CImage &cImage);
	CRect cRect1;
	CRect cRect2;
	CRect cRect3;
	CRect cRect4;

    
};