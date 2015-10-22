/******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  detection.cpp
* @brief Road furniture detection source file
*
* Change Log:
*      Date                Who             What
*      2015/1/13         Bingtao Gao      Create
*******************************************************************************
*/
#include <fstream>
#include <iostream>
#include <vector>

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "svm.h"
#include "detection_blackWhite.h"
#include "configure.h"

using namespace cv;
using namespace std;

namespace ns_detection
{

Detector_blackWhite::Detector_blackWhite(float highStep, double dist_per_piexl,int horizon_line, int featureNum) :Detector(highStep,dist_per_piexl,horizon_line),
				_MAX_NUM_FEATURES(featureNum)
{
    _sw = 32;
    _d.winSize = Size(_sw, _sw);
    _d.blockSize = Size(_sw/4, _sw/4);
    _d.blockStride =  Size(_sw/8, _sw/8);
    _d.cellSize = Size(_sw/8, _sw/8);
    _d.nbins = 8;

    _kernel_size = 3;

#if(RD_LOCATION == RD_GERMAN_LEHRE)
    

    _MAX_NUM_SIGN_PER_IMG = 10;
    _MIN_DISTANCE = 130.0;
    _START_ROW = 160;
    _END_ROW = 320;
    _THESHOLD = 3;
    _START_COL = 384;
    _MAX_RADUS = 18;
    _MIN_RADUS = 6;
    _STEP_RADUS = 1;

    _image.push_back(imread("./resource/Germany/png/10100.png"));
    _image.push_back(imread("./resource/Germany/png/12300.png"));
    _image.push_back(imread("./resource/Germany/png/13100.png"));
    _image.push_back(imread("./resource/Germany/png/13310.png"));
    _image.push_back(imread("./resource/Germany/png/13810.png"));
    _image.push_back(imread("./resource/Germany/png/20500.png"));
    _image.push_back(imread("./resource/Germany/png/20600.png"));
    _image.push_back(imread("./resource/Germany/png/20930.png"));
    _image.push_back(imread("./resource/Germany/png/21500.png"));
    _image.push_back(imread("./resource/Germany/png/22220.png"));
    _image.push_back(imread("./resource/Germany/png/22400.png"));
    _image.push_back(imread("./resource/Germany/png/23700.png"));
    _image.push_back(imread("./resource/Germany/png/23900.png"));
    _image.push_back(imread("./resource/Germany/png/24000.png"));
    _image.push_back(imread("./resource/Germany/png/25000.png"));
    _image.push_back(imread("./resource/Germany/png/25900.png"));
    _image.push_back(imread("./resource/Germany/png/26100.png"));
    _image.push_back(imread("./resource/Germany/png/26210.png"));
    _image.push_back(imread("./resource/Germany/png/27452.png"));
    _image.push_back(imread("./resource/Germany/png/27453.png"));
    _image.push_back(imread("./resource/Germany/png/27454.png"));
    _image.push_back(imread("./resource/Germany/png/27455.png"));
    _image.push_back(imread("./resource/Germany/png/27456.png"));
    _image.push_back(imread("./resource/Germany/png/27458.png"));
    _image.push_back(imread("./resource/Germany/png/28300.png"));
    _image.push_back(imread("./resource/Germany/png/28600.png"));
    _image.push_back(imread("./resource/Germany/png/30100.png"));
    _image.push_back(imread("./resource/Germany/png/30600.png"));
    _image.push_back(imread("./resource/Germany/png/31400.png"));
    _image.push_back(imread("./resource/Germany/png/33100.png"));
    _image.push_back(imread("./resource/Germany/png/33600.png"));
    _image.push_back(imread("./resource/Germany/png/35010.png"));
    _image.push_back(imread("./resource/Germany/png/99900.png"));
    _image.push_back(imread("./resource/Germany/png/20910.png"));
    _image.push_back(imread("./resource/Germany/png/25400.png"));
    _image.push_back(imread("./resource/Germany/png/26700.png"));
    _image.push_back(imread("./resource/Germany/png/31401.png"));
    _image.push_back(imread("./resource/Germany/png/27600.png"));
    _image.push_back(imread("./resource/Germany/png/44100.png"));
    _image.push_back(imread("./resource/Germany/png/44200.png"));

    _cir_model = svm_load_model("./resource/Germany/svmLehre/cir_model.txt"); 

    loadFeat(_feat_cir, "./resource/Germany/svmLehre/CIRfeat.txt");
#elif(RD_LOCATION == RD_US_PALO_ALTO)
    _MAX_NUM_SIGN_PER_IMG = 4;
    _MIN_DISTANCE = 100.0;
    _START_ROW = 100;
    _END_ROW = 240;
    _THESHOLD = 1.6;
    _START_COL = 400;
    _MAX_RADUS = 28;
    _MIN_RADUS = 10;
    _STEP_RADUS = 2;

    _image.push_back(imread("./resource/US/png/1.png"));
    _image.push_back(imread("./resource/US/png/2.png"));
    _image.push_back(imread("./resource/US/png/3.png"));
    _image.push_back(imread("./resource/US/png/4.png"));
    _image.push_back(imread("./resource/US/png/5.png"));
    _image.push_back(imread("./resource/US/png/6.png"));
    _image.push_back(imread("./resource/US/png/7.png"));
    _image.push_back(imread("./resource/US/png/8.png"));
    _image.push_back(imread("./resource/US/png/35.png"));

    _rec_model = svm_load_model("./resource/US/svm/Palo_Alto/rec_model.txt");
    loadFeat(_feat_rec, "./resource/US/svm/Palo_Alto/RECfeat.txt");
#else
    _MAX_NUM_SIGN_PER_IMG = 3;
    _MIN_DISTANCE = 400.0;
    _START_ROW = 120;
    _END_ROW = 320;
    _THESHOLD = 1.05;
	_START_COL = 0;
    _MAX_RADUS = 42;
    _MIN_RADUS = 10;
    _STEP_RADUS = 4;
	
    _image.push_back(imread("./resource/US/png/1.png"));
    _image.push_back(imread("./resource/US/png/2.png"));
    _image.push_back(imread("./resource/US/png/3.png"));
    _image.push_back(imread("./resource/US/png/4.png"));
    _image.push_back(imread("./resource/US/png/5.png"));
    _image.push_back(imread("./resource/US/png/6.png"));
    _image.push_back(imread("./resource/US/png/7.png"));
    _image.push_back(imread("./resource/US/png/8.png"));

    _cir_model = svm_load_model("./resource/US/svm/Detroit/cir_model.txt");
    _rec_model = svm_load_model("./resource/US/svm/Detroit/rec_model.txt");

    loadFeat(_feat_cir, "./resource/US/svm/Detroit/CIRfeat.txt");
    loadFeat(_feat_rec, "./resource/US/svm/Detroit/RECfeat.txt");
#endif
}

Mat Detector_blackWhite::ID2Image(int target) 
{
   Mat s;
#if((RD_LOCATION&RD_NATION_MASK) == RD_GERMAN)
    switch (target)
    {
	case 10100:
		{
			s = _image[0];
			break;
		}
	case 12300:
		{
			s = _image[1];
			break;
		}
	case 13100:
		{
			s = _image[2];
			break;
		}
	case 13310:
		{
			s = _image[3];
			break;
		}
	case 13810:
		{
			s = _image[4];
			break;
		}
	case 20500:
		{
			s = _image[5];
			break;
		}
	case 20600:
		{
			s = _image[6];
			break;
		}
	case 20930:
		{
			s = _image[7];
			break;
		}
	case 21500:
		{
			s = _image[8];
			break;
		}
	case 22200:
		{
			s = _image[9];
			break;
		}
	case 22400:
		{
			s = _image[10];
			break;
		}
	case 23700:
		{
			s = _image[11];
			break;
		}
	case 23900:
		{
			s = _image[12];
			break;
		}
	case 24000:
		{
			s = _image[13];
			break;
		}
	case 25000:
		{
			s = _image[14];
			break;
		}
	case 25900:
		{
			s = _image[15];
			break;
		}
	case 26100:
		{
			s = _image[16];
			break;
		}
	case 26210:
		{
			s = _image[17];
			break;
		}
	case 27452:
    case 27552:
		{
			s = _image[18];
			break;
		}
	case 27453:
    case 27553:
		{
			s = _image[19];
			break;
		}
	case 27454:
    case 27554:
		{
			s = _image[20];
			break;
		}
	case 27455:
    case 27555:
		{
			s = _image[21];
			break;
		}
	case 27456:
    case 27556:
		{
			s = _image[22];
			break;
		}
	case 27458:
    case 27558:
		{
			s = _image[23];
			break;
		}
	case 28300:
		{
			s = _image[24];
			break;
		}
	case 28600:
		{
			s = _image[25];
			break;
		}
	case 30100:
		{
			s = _image[26];
			break;
		}
	case 30600:
		{
			s = _image[27];
			break;
		}
	case 31400:
		{
			s = _image[28];
			break;
		}
	case 33100:
		{
			s = _image[29];
			break;
		}
	case 33600:
		{
			s = _image[30];
			break;
		}
	case 35010:
		{
			s = _image[31];
			break;
		}
	case 99900:
		{
			s = _image[32];
			break;
		}
    case 20910:
        {
            s = _image[33];
            break;
        }
    case 25400:
        {
            s = _image[34];
            break;
        }
    case 26700:
        {
            s = _image[35];
            break;
        }
    case 31401:
        {
            s = _image[36];
            break;
        }
    case 27600:
        {
            s = _image[37];
            break;
        }
    case 44100:
        {
            s = _image[38];
            break;
        }
    case 44200:
        {
            s = _image[39];
            break;
        }

	default:
		break;
        }
#else
    switch (target)
    {
    case 1:
        {
            s = _image[0];
            break;
        }
    case 2:
        {
            s = _image[1];
            break;
        }
    case 3:
        {
            s = _image[2];
            break;
        }
    case 4:
        {
            s = _image[3];
            break;
        }
    case 5:
        {
            s = _image[4];
            break;
        }
    case 6:
        {
            s = _image[5];
            break;
        }
    case 7:
        {
            s = _image[6];
            break;
        }
    case 8:
        {
            s = _image[7];
            break;
        }
    case 35:
        {
            s = _image[8];
            break;
        }
    default:
        s = _image[0];
        break;
    }
#endif
    return s;
}

int Detector_blackWhite::targetClassify(Mat image, vector<int> &feat, svm_model *model) 
{
    int target = 0;

    //Get Image ROI
    Mat image_roi;
    image.copyTo(image_roi);

    //Resize and Convert Color
    Mat imag;
    resize(image_roi, imag, Size(_sw,_sw));
    //cvtColor(imag, imag, CV_RGB2GRAY);

    //Compute HOG features.
    vector<float> descriptorsValues;
    vector<Point> locations;
    _d.compute(imag, descriptorsValues,Size(0,0), Size(0,0), locations);

    //malloc memory for each svm_node
    struct svm_node node;
    node.values = new double[_MAX_NUM_FEATURES];
    node.dim = _MAX_NUM_FEATURES;

    //Select reduced Features
    for (int k = 0;k < _MAX_NUM_FEATURES; k++)
    {
        node.values[k] = descriptorsValues[feat[k]];
    }

    //SVM Prediction with the given node.
    double prob_estimates[100];
    target = svm_predict_probability(model, &node, prob_estimates);    

    delete node.values; 
    return target;
}

void Detector_blackWhite::fft2(Mat &src, Mat &dst,Mat &complexI)  
{    
    Mat image_Re;
    Mat image_Im;
    // Real part conversion from u8 to 64f (double)  
    src.convertTo(image_Re,CV_64F);
    Mat planes[] = {Mat_<double>(image_Re), Mat::zeros(src.size(), CV_64F) };
    merge(planes, 2, complexI);
    
    dft(complexI, complexI);   
    //split(complexI, planes);
 //   magnitude(planes[0], planes[1], image_Re);

    //double minVal = 0, maxVal = 0;  
 //   // Localize minimum and maximum values  
 //   minMaxLoc( image_Re, &minVal, &maxVal ); 
 //  // Normalize image (0 - 255) to be observed as an u8 image  
    //double scale, shift;
 //   scale = 255/(maxVal - minVal);  

    //image_Re = (image_Re - minVal)*scale;

    //image_Re.convertTo(dst,CV_8U);
}  

void Detector_blackWhite::ifft2(Mat &src, Mat &dst)  
{    
    Mat image_Re;
    Mat image_Im;
    Mat planes[2] ;
    Mat complexI;
   
    idft(src,complexI);
    split(complexI, planes);   
    magnitude(planes[0], planes[1], image_Re);

    double minVal = 0, maxVal = 0;  
    // Localize minimum and maximum values  
    minMaxLoc( image_Re, &minVal, &maxVal );  
   // Normalize image (0 - 255) to be observed as an u8 image  
    double scale, shift;
    scale = 255/(maxVal - minVal);  
    image_Re = (image_Re - minVal)*scale;
    image_Re.convertTo(dst,CV_8U);
}

void Detector_blackWhite::processNoise(Mat &src, Mat &dst)
{
    // fft process
    if(src.channels() != 1)
    {    
        cvtColor(src,src,COLOR_RGB2GRAY);
    }

    Mat fftOut,fftOutTemp;

    //namedWindow("src",0);
    //imshow("src",src);

    fft2(src,fftOut,fftOutTemp);

    // delete the DC subcarrier
    int width = src.cols;
    int height = src.rows;

    double *dataPtr = (double*)fftOutTemp.data;
    int step = fftOutTemp.step/sizeof(double);  
    int channels = fftOutTemp.channels(); 
    for(int rowIdx=0;rowIdx<height;rowIdx++)  
    {
        for(int colIdx = width/2 - 40;colIdx <  width/2 + 40;colIdx++)  
        {
                dataPtr[rowIdx*step + colIdx*channels+0] = 0.0;
                dataPtr[rowIdx*step + colIdx*channels+1] = 0.0;
        }
    }
    for(int rowIdx = height/2 - 40;rowIdx < height/2 + 40;rowIdx++) 
    {
        for(int colIdx = 0; colIdx < width;colIdx++)  
        {
                dataPtr[rowIdx*step + colIdx*channels+0] = 0.0;
                dataPtr[rowIdx*step + colIdx*channels+1] = 0.0;
        }
    }
    ifft2(fftOutTemp,dst); 
}

double Detector_blackWhite::searchFixedRadusShape(Mat &dxImg, Mat &dyImg,Mat &Sr,int radus,int nSide)
{
    // step 2: calculate the voting image Qr
    
    Mat QrAndBr(dxImg.rows,dxImg.cols,CV_32FC3);
    //Mat Qr(dxImg.rows,dxImg.cols,CV_16S);
    //Mat Brx(dxImg.rows,dxImg.cols,CV_32F);
    //Mat Bry(dxImg.rows,dxImg.cols,CV_32F);
    //

    QrAndBr.setTo(0);
    //Qr.setTo(0);
    //Brx.setTo(0);
    //Bry.setTo(0);
    
    
    float tanN = tan(_PI/nSide);
    int w = cvRound(radus*tanN);
    
    int feat1 = QrAndBr.step / sizeof(float);

    for(int rowIdx = _START_ROW;rowIdx < _END_ROW;rowIdx++)
    {
        float* dxInRow = (float*)(dxImg.data + rowIdx*dxImg.step);
        float* dyInRow = (float*)(dyImg.data + rowIdx*dyImg.step);

        for(int colIdx = _START_COL;colIdx < dxImg.cols;colIdx++)
        {
            float *dxIn = dxInRow + colIdx;
            float *dyIn = dyInRow + colIdx;
            
            float dx = *dxIn;
            float dy = *dyIn;

            if((dx != 0) || (dy != 0))
            {
                float sqrtd = sqrt(dx*dx + dy*dy);
                float sinTheta = dy/sqrtd;
                float cosTheta = dx/sqrtd;
                int detaX = cvRound(sinTheta*radus);
                int detaY = cvRound(cosTheta*radus);


                int XXP = rowIdx + detaX;
                int YYP = colIdx + detaY;
                int XXN = rowIdx - detaX;
                int YYN = colIdx - detaY;

                // step 2.1: calculate the Qr
            //    circle(src,Point(YYP,XXP),2,Scalar(0,0,255));
            //    circle(src,Point(YYN,XXN),2,Scalar(0,255,255));
                {
                    // step 2.1.1: calculate the voting line which  gradient points to the center
                        // for Qr positive voting space
                    {
                        for(int ii = -w;ii <= w;ii ++)
                        {
                            int detaWx = XXP - cvRound(ii*cosTheta);
                            int detaWy = YYP + cvRound(ii*sinTheta);
                            int step1 = detaWx*feat1 + detaWy*3;
                            float* inDataRow = (float*)(QrAndBr.data) + step1; 

                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            {
                                *inDataRow += 1.0;

                            }
                        }
                        // step 2.2.2: negative voting left space
                        for(int ii = -2*w;ii <= -w-1;ii ++)
                        {
                            int detaWx = XXP - cvRound(ii*cosTheta);
                            int detaWy = YYP + cvRound(ii*sinTheta);
                            //short* inDataRow = (short*)(Qr.data + detaWx*Qr.step) + detaWy;
                            int step1 = detaWx*feat1 + detaWy*3;
                            float* inDataRow = (float*)(QrAndBr.data) + step1; 
                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            {
                                *inDataRow -= 1.0;

                            }
                        }


                        // step 2.2.3: negative voting right space
                        for(int ii = w+1;ii <= 2*w;ii ++)
                        {
                            int detaWx = XXP - cvRound(ii*cosTheta);
                            int detaWy = YYP + cvRound(ii*sinTheta);
                            //short* inDataRow = (short*)(Qr.data + detaWx*Qr.step) + detaWy;
                            int step1 = detaWx*feat1 + detaWy*3;
                            float* inDataRow = (float*)(QrAndBr.data) + step1;  

                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            {
                                *inDataRow -= 1.0;
                            }
                        }
                    }
                    // step 2.1.2: calculate the voting line which  gradient points away the center
                    {
                        for(int ii = -w;ii <= w;ii ++)
                        {
                            int detaWx = XXN - cvRound(ii*cosTheta);
                            int detaWy = YYN + cvRound(ii*sinTheta);
                            //short* inDataRow = (short*)(Qr.data + detaWx*Qr.step) + detaWy;
                            int step1 = detaWx*feat1 + detaWy*3;
                            float* inDataRow = (float*)(QrAndBr.data) + step1; 
                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            {
                                *inDataRow += 1.0;
                            }
                        }

                        // step 2.2.2: negative voting left part
                        for(int ii = -2*w;ii <= -w-1;ii ++)
                        {
                            int detaWx = XXN - cvRound(ii*cosTheta);
                            int detaWy = YYN + cvRound(ii*sinTheta);

                            //short* inDataRow = (short*)(Qr.data + detaWx*Qr.step) + detaWy;
                            int step1 = detaWx*feat1 + detaWy*3;
                            float* inDataRow = (float*)(QrAndBr.data) + step1; 

                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            { 
                                *inDataRow -= 1.0;
                            }
                        }
                        // step 2.2.3: negative voting right part
                        for(int ii = w+1;ii <= 2*w;ii ++)
                        {
                            int detaWx = XXN - cvRound(ii*cosTheta);
                            int detaWy = YYN + cvRound(ii*sinTheta);
                        //    short* inDataRow = (short*)(Qr.data + detaWx*Qr.step) + detaWy;
                            int step1 = detaWx*feat1 + detaWy*3;
                            float* inDataRow = (float*)(QrAndBr.data) + step1; 
                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            {
                                *inDataRow -= 1.0;
                            }
                        }
                    }
                }
            }
        }
    }

    // step 3: caculate the Br
    Mat Br(dxImg.rows,dxImg.cols,CV_32FC2);
    Br.setTo(0);

    for(int rowIdx = _START_ROW;rowIdx < _END_ROW;rowIdx++)
    {
        float* dxInRow = (float*)(dxImg.data + rowIdx*dxImg.step);
        float* dyInRow = (float*)(dyImg.data + rowIdx*dyImg.step);

        for(int colIdx = _START_COL;colIdx < dxImg.cols;colIdx++)
        {
            float *dxIn = dxInRow + colIdx;
            float *dyIn = dyInRow + colIdx;
            
            float dx = *dxIn;
            float dy = *dyIn;

            if((dx != 0) || (dy != 0))
            {
                float sqrtD = sqrt(dx*dx + dy*dy);
                float sinTheta = dy/sqrtD;
                float cosTheta = dx/sqrtD;
                int detaX = cvRound(sinTheta*radus);
                int detaY = cvRound(cosTheta*radus);            

                int XXP = rowIdx + detaX;
                int YYP = colIdx + detaY;
                int XXN = rowIdx - detaX;
                int YYN = colIdx - detaY;

                float thetaG = fastAtan2(dy,dx) ;
                float nThetaG = nSide*thetaG*_PI_DIV_180;// n-angle
                float vpTheta = nThetaG;// angle for positive voting line
                float vnTheta = nThetaG + _PI;// angle for negative voting line

                float cosPosTheta = cos(vpTheta);
                float sinPosTheta = sin(vpTheta);
                float cosNegTheta = cos(vnTheta);
                float sinNegTheta = sin(vnTheta);


                // step 2.1: calculate the Br

                {
                    // step 2.1.1: calculate the voting line which  gradient points to the center
                    {
                        // for Br positive voting space
                        for(int ii = -w;ii <= w;ii ++)
                        {
                            int detaWx = XXP - cvRound(ii*cosTheta);
                            int detaWy = YYP + cvRound(ii*sinTheta);

                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            {
                                //short* inDataRow = (short*)(Qr.data + detaWx*Qr.step) + detaWy; 
                                float* inDataRow = (float*)(QrAndBr.data + detaWx*QrAndBr.step) + detaWy*3; 

                                //float* brPtr = (float*)(Br.data + detaWx*Br.step) + detaWy*2;
                                inDataRow[1]  += inDataRow[0]*cosPosTheta;
                                inDataRow[2]  += inDataRow[0]*sinPosTheta;

                            }
                        }
                        // step 2.2.2: negative voting left space
                        for(int ii = -2*w;ii <= -w-1;ii ++)
                        {
                            int detaWx = XXP - cvRound(ii*cosTheta);
                            int detaWy = YYP + cvRound(ii*sinTheta);

                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            {
                                float* inDataRow = (float*)(QrAndBr.data + detaWx*QrAndBr.step) + detaWy*3; 
                                inDataRow[1]  += inDataRow[0]*cosNegTheta;
                                inDataRow[2]  += inDataRow[0]*sinNegTheta;
                            }
                        }
                        // step 2.2.3: negative voting right space
                        for(int ii = w+1;ii <= 2*w;ii ++)
                        {
                            int detaWx = XXP - cvRound(ii*cosTheta);
                            int detaWy = YYP + cvRound(ii*sinTheta);

                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            {
                                float* inDataRow = (float*)(QrAndBr.data + detaWx*QrAndBr.step) + detaWy*3; 
                                inDataRow[1]  += inDataRow[0]*cosNegTheta;
                                inDataRow[2]  += inDataRow[0]*sinNegTheta;
                            }
                        }
                    }
                    // step 2.1.2: calculate the voting line which  gradient points away the center
                    {
                        for(int ii = -w;ii <= w;ii ++)
                        {
                            int detaWx = XXN - cvRound(ii*cosTheta);
                            int detaWy = YYN + cvRound(ii*sinTheta);

                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            {
                                float* inDataRow = (float*)(QrAndBr.data + detaWx*QrAndBr.step) + detaWy*3; 
                                inDataRow[1]  += inDataRow[0]*cosPosTheta;
                                inDataRow[2]  += inDataRow[0]*sinPosTheta;
                            }
                        }
                        // step 2.2.2: negative voting left part
                        for(int ii = -2*w;ii <= -w-1;ii ++)
                        {
                            int detaWx = XXN - cvRound(ii*cosTheta);
                            int detaWy = YYN + cvRound(ii*sinTheta);

                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            {
                                float* inDataRow = (float*)(QrAndBr.data + detaWx*QrAndBr.step) + detaWy*3; 
                                inDataRow[1]  += inDataRow[0]*cosNegTheta;
                                inDataRow[2]  += inDataRow[0]*sinNegTheta;
                            }
                        }
                        // step 2.2.3: negative voting right part
                        for(int ii = w+1;ii <= 2*w;ii ++)
                        {
                            int detaWx = XXN - cvRound(ii*cosTheta);
                            int detaWy = YYN + cvRound(ii*sinTheta);

                            if((detaWx >= 0) && (detaWx < dxImg.rows) && (detaWy >= 0) && (detaWy < dxImg.cols))
                            {
                                float* inDataRow = (float*)(QrAndBr.data + detaWx*QrAndBr.step) + detaWy*3; 
                                inDataRow[1]  += inDataRow[0]*cosNegTheta;
                                inDataRow[2]  += inDataRow[0]*sinNegTheta;
                            }
                        }
                    }
                }
            }
        }
    }

    // step 3: calculate the Sr

    float pow2 = 2.0*radus*w *2.0*radus*w;

    for(int rowIdx = _START_ROW - _MAX_RADUS;rowIdx < QrAndBr.rows;rowIdx++)
    {
        float* inDataRow = (float*)(QrAndBr.data + rowIdx*QrAndBr.step); 
        float* srInPtr = (float*)(Sr.data + rowIdx*Sr.step);

        for(int colIdx = 0;colIdx < QrAndBr.cols;colIdx++)
        {
            float qr = inDataRow[0];
            float dx = inDataRow[1];
            float dy = inDataRow[2];
            float mulVal = dx*dx + dy*dy;
            float sqrtVal = sqrt(mulVal);
            *srInPtr = qr * sqrtVal / pow2;

            inDataRow += 3;
            srInPtr++;
        }
    }

    //Mat Br = Brx.mul(Brx) + Bry.mul(Bry);
    //sqrt(Br,Br);

    //
    //Mat divMat(dxImg.rows,dxImg.cols,CV_32F);
    //divMat.setTo(pow2);
    //Qr.convertTo(Qr,CV_32F);

    //Sr = Qr.mul(Br);
    //divide(Sr,divMat,Sr);

    // *************for debug************************
    //double minVal = 0, maxVal = 0;  
    //Point maxValLoc,minValLoc;
 //   // Localize minimum and maximum values  
 //   minMaxLoc( Qr, &minVal, &maxVal,&minValLoc,&maxValLoc);  
 //  // Normalize image (0 - 255) to be observed as an u8 image  
    //double scale;
 //   scale = 255/(maxVal - minVal);  
 //   Qr = (Qr - minVal)*scale;
    //Qr.convertTo(Qr,CV_8U);

    //minMaxLoc( Br, &minVal, &maxVal,&minValLoc,&maxValLoc);  
 //  // Normalize image (0 - 255) to be observed as an u8 image  
    ////double scale;
 //   scale = 255/(maxVal - minVal);  
 //   Br = (Br - minVal)*scale;
    //Br.convertTo(Br,CV_8U);

 //   // Localize minimum and maximum values  
    //Mat SrTemp(Sr.rows,Sr.cols,CV_32F);
    //
 //   minMaxLoc( Sr, &minVal, &maxVal,&minValLoc,&maxValLoc);  
 //  // Normalize image (0 - 255) to be observed as an u8 image  
 //   scale = 255/(maxVal - minVal);  
 //   Sr = (Sr - minVal)*scale;
    //Sr.convertTo(SrTemp,CV_8U);

    //namedWindow("Qr",1);
    //imshow("Qr",Qr);

    //namedWindow("Br",1);
    //imshow("Br",Br);

    //namedWindow("Sr",1);
    //imshow("Sr",SrTemp);

    //int key = waitKey(1);
    return 0;
}

int Detector_blackWhite::searchRegularPolygon(Mat &src,Mat &dxImg, Mat &dyImg,vector<int> &radusVec,int nSide,TS_Structure &target,Mat &dstTemp)
{
    int radusNum = radusVec.size();
    bool detectedFlag = false;

    vector<double> maxValVec;
    double maxValSr = 0;
    int maxValSrLoc = 0;
    vector<Point> maxLoc;

    Mat dst(src.rows,src.cols,CV_32F);
    dst.setTo(0);

    for(int radusIdx = 0; radusIdx < radusNum; radusIdx++)
    {
        

        Mat Sr(src.rows,src.cols,CV_32F);
        int radus = radusVec[radusIdx];
        searchFixedRadusShape(dxImg,dyImg,Sr,radus,nSide);
        double minVal, maxVal;
        Point maxValLoc,minValLoc;
        minMaxLoc( Sr, &minVal, &maxVal,&minValLoc,&maxValLoc);  
        maxLoc.push_back(maxValLoc);
        if(maxValSr < maxVal)
        {
            maxValSr = maxVal;
            maxValSrLoc = radusIdx;
        }
        maxValVec.push_back(maxVal);
        dst += Sr;
    }
        //Mat dstTemp;
      dst.copyTo(dstTemp);
    // look up the 10 maximum value location
    vector<Point> maxLoc3;
    vector<double> valVec;
    unsigned int counter = 0;
    double valSum = 0;
    while(counter < _MAX_NUM_SIGN_PER_IMG)
    {
        double minVal1,maxVal1;
        Point minValLoc1,maxValLoc1;
        
        minMaxLoc( dst, &minVal1, &maxVal1,&minValLoc1,&maxValLoc1);   
        
        if(maxVal1 == 0.0)
            return 0;
        dst.at<float>(maxValLoc1.y,maxValLoc1.x) = -9e-10;
        bool findFlag = true;
        for(int ii = 0; ii < maxLoc3.size();++ii)
        {
            int dX1 = maxValLoc1.x - maxLoc3[ii].x;
            int dY1 = maxValLoc1.y - maxLoc3[ii].y;
            int dist1 = dX1*dX1 + dY1*dY1;
            if(dist1 < _MIN_DISTANCE)
            {
                dst.at<float>(maxValLoc1.y,maxValLoc1.x) = -9e-10;
                findFlag = false;
                break;
            }
        }
        if(findFlag)
        {
            dst.at<float>(maxValLoc1.y,maxValLoc1.x) = -9e-10;
            maxLoc3.push_back(maxValLoc1);
            valSum += maxVal1;
            valVec.push_back(maxVal1);
            ++counter;
        }
    }
    for(int index = 0;index < _MAX_NUM_SIGN_PER_IMG;index++)
    {
        if( valVec[index] > _THESHOLD*valSum/_MAX_NUM_SIGN_PER_IMG)
        {
            float minDist = 9e+10;
            int minIdx = radusNum + 1;
            for(int radusIdx = 0; radusIdx < radusNum; radusIdx++)
            {
                int distX = maxLoc3[index].x - maxLoc[radusIdx].x;
                int distY = maxLoc3[index].y - maxLoc[radusIdx].y;
                float dist = distX * distX + distY * distY;
                if(dist < minDist)
                {
                    minDist = dist;
                    minIdx = radusIdx;
                }
            }
            if(minIdx <= radusNum)
            {
        
                int realRadus = radusVec[minIdx];

                // calculate the ROI start point

                vector<Point> rectRoi;
            //    Point center(maxLoc[minIdx]);
                Point P1,P2;
                P1.x = (maxLoc3[index].x - realRadus) > 0 ? (maxLoc3[index].x - realRadus):0 ;
                P1.y = (maxLoc3[index].y - realRadus) > 0 ? (maxLoc3[index].y - realRadus):0;
                P2.x = (P1.x + 2*realRadus) >= src.cols ? src.cols-1 :(P1.x + 2*realRadus) ;
                P2.y = (P1.y + 2*realRadus) >= src.rows ? src.rows-1 : (P1.y + 2*realRadus);

                rectRoi.push_back(P1);
                rectRoi.push_back(P2);

                //circle(src,P1,2,Scalar(0,255,255));
                //circle(src,P2,2,Scalar(0,255,0));
                cv::Rect roi = boundingRect(rectRoi);


                detectedFlag = true;
                //circle(src,maxLoc3[index],realRadus,Scalar(0,0,255));
        
                if(detectedFlag == true)
                {
                    int type;
                    // identify the stop traffic sign
#if(RD_LOCATION == RD_GERMAN_LEHRE)                      
                    type = targetClassify(src(roi),_feat_cir,_cir_model);
#else
                    if(nSide == 8)
                    {
                        type = targetClassify(src(roi),_feat_cir,_cir_model); 
                        //cout<<"detected type:"<<type<<endl;
                    }
                    else
                    {
                        //identify the square sign
                        type = targetClassify(src(roi),_feat_rec,_rec_model); 
                        //cout<<"detected type:"<<type<<endl;
                        /*sprintf_s( currFileName, 1000, "D:/Newco/testImage0620/rectangle/square/%06d.png",squareID++);
                        imwrite(currFileName,src(roi));*/
                    }
#endif
                    if((type != 0) && (type != 2) && (type != 3) && (type != 4))
                    {
                        TS_Structure::TS_element detectSign;
                        detectSign.type = type;
                        detectSign.area = roi.area();
                        detectSign.rect = roi;
                        detectSign.center = maxLoc3[index];         
                        target.trafficSign.push_back(detectSign);                       
                    }        
                }
            }
        }
    }

    // Normalize image (0 - 255) to be observed as an u8 image for debug

    double scale2 = 0;
    double minVal2,maxVal2;
    Point minValLoc2,maxValLoc2;
    minMaxLoc( dstTemp, &minVal2, &maxVal2,&minValLoc2,&maxValLoc2);   
    scale2 = 255/(maxVal2 - minVal2);  
    dstTemp = (dstTemp - minVal2)*scale2;

    dstTemp.convertTo(dstTemp,CV_8U);

    //circle(src,maxValLoc2,2,Scalar(0,0,255));

    //threshold(dstTemp,dstTemp,120,255,THRESH_TOZERO);

    //namedWindow("output",1);
    //imshow("output",dstTemp);
    return 1;
}

void Detector_blackWhite::trafficSignDetect(Mat image, TS_Structure &target)
{
    Mat imageIn;
    image.copyTo(imageIn);

    //processNoise(src,src);

    Mat dxImg(image.rows,image.cols, CV_32F);
    Mat dyImg(image.rows,image.cols, CV_32F);
    calculateGradient(image,dxImg,dyImg);

    vector<int> radus;
    for(int kk = _MIN_RADUS; kk <= _MAX_RADUS; kk += _STEP_RADUS)
    {
        radus.push_back(kk);
    }

    Mat dst1,dst2;
#if(RD_LOCATION == RD_GERMAN_LEHRE)
    searchRegularPolygon(image,dxImg,dyImg,radus,12,target,dst1); // 12: circle
#elif(RD_LOCATION == RD_US_PALO_ALTO)
    searchRegularPolygon(image,dxImg,dyImg,radus,4,target,dst2);  // 4: rectangle
#else
    searchRegularPolygon(image,dxImg,dyImg,radus,8,target,dst1);
    searchRegularPolygon(image,dxImg,dyImg,radus,4,target,dst2);
#endif

    Mat background(image.rows,4*image.cols/3, CV_8UC3,Scalar(128,128,128));    

    for(int i = 0; i< target.trafficSign.size(); i++)
    {
        char currFileName[1000];
        int type = target.trafficSign[i].type;
        Rect rect = target.trafficSign[i].rect;

        Mat subWin1 = ID2Image(type);


        Mat roiTemp = image(rect);

        if (roiTemp.data)
        {
            resize(roiTemp,roiTemp, Size(image.cols/6,2*image.rows/9));
            //cvtColor(roiTemp,roiTemp,COLOR_GRAY2BGR);
            roiTemp.copyTo(background(Rect(0,i*2*image.rows/9,image.cols/6,2*image.rows/9)));

        }
        if (subWin1.data)
        {
            resize(subWin1,subWin1, Size(image.cols/6,2*image.rows/9));
            subWin1.copyTo(background(Rect(image.cols/6,i*2*image.rows/9,image.cols/6,2*image.rows/9)));
            //static int stopID = 1;
            //static int squareID = 1;
            //char currFileName[1000];
            //sprintf_s( currFileName, 1000, "D:/Newco/testImage0620/sign/%06d.png",stopID++);
            //imwrite(currFileName,imageIn);
        }    

        rectangle(image,rect,Scalar(0,255,255), 1,8);
    }    
    for (int i=0; i<2; i++)
    {
        int x = image.cols/6 * i;
        for (int j=0; j<3;j++)
        {   
            int y = 2*image.rows/9*j;
            rectangle(background,Rect(x,y,image.cols/6,2*image.rows/9),Scalar(255,255,255), 1,8);
        }
    }    
    if(dst1.data != 0) 
    {
        resize(dst1,dst1,Size(image.cols/3,image.rows/3));
        cvtColor(dst1,dst1,COLOR_GRAY2BGR);
        dst1.copyTo(background(Rect(0,2*image.rows/3,image.cols/3,image.rows/3)));
    }
    if(dst2.data != 0) 
    {
        resize(dst2,dst2,Size(image.cols/3,image.rows/3));
        cvtColor(dst2,dst2,COLOR_GRAY2BGR);
        dst2.copyTo(background(Rect(0,2*image.rows/3,image.cols/3,image.rows/3)));
    }
    //cvtColor(src,src,COLOR_GRAY2BGR);
    image.copyTo(background(Rect(image.cols/3,0,image.cols,image.rows)));
    
    rectangle(background,Rect(0,2*image.rows/3,image.cols/3,image.rows/3),Scalar(255,255,255), 1,8);
    rectangle(background,Rect(image.cols/3,0,image.cols,image.rows),Scalar(255,255,255), 1,8);
    namedWindow("trafficSign",CV_WINDOW_NORMAL);
    imshow("trafficSign",background);
    waitKey(1);
}
}
