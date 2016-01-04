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
#include <direct.h>

//#include <opencv2\core\core.hpp>
//#include <opencv2\highgui\highgui.hpp>
//#include <opencv2\imgproc\imgproc.hpp>
//#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>

#include "svm.h"
#include "detection_colored.h"
#include "database.h"
#include "markLocate.h"

#include "roadScan.h"
#include "configure.h"

using namespace ns_roadScan;

using namespace cv;
using namespace std;

namespace ns_detection
{
svm_model * Detector_colored::_cir_model;
svm_model * Detector_colored::_rec_model;
svm_model * Detector_colored::_tri_model;
svm_model * Detector_colored::_cir_model_added;
    
std::vector<int> Detector_colored::_feat_cir;
std::vector<int> Detector_colored::_feat_rec;
std::vector<int> Detector_colored::_feat_tri;
std::vector<int> Detector_colored::_feat_cir_added;

std::vector<cv::Mat> Detector_colored::_image;

void loadModels()
{
    Detector_colored::_image.push_back(imread("./resource/Germany/png/10100.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/10200.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/12300.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/13100.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/13310.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/13810.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/20500.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/20600.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/20910.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/20930.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/21500.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/22220.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/22400.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/23700.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/23900.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/24000.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/24100.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/25000.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/25400.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/25900.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/26100.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/26210.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/26700.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/27452.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/27453.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/27454.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/27455.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/27456.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/27458.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/27600.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/28300.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/28400.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/28500.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/28600.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/28700.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/30100.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/30600.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/31400.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/31401.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/33100.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/33600.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/34100.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/35010.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/44100.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/44200.png"));
    Detector_colored::_image.push_back(imread("./resource/Germany/png/99900.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/10310.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/10320.png"));
	Detector_colored::_image.push_back(imread("./resource/Germany/png/22210.png"));

#if (RD_LOCATION == RD_GERMAN_LEHRE2)
    Detector_colored::_cir_model = svm_load_model("./resource/Germany/svmLehre2/cir_model.txt");
    Detector_colored::_rec_model = svm_load_model("./resource/Germany/svmLehre2/rec_model.txt");

    loadFeat(Detector_colored::_feat_cir, "./resource/Germany/svmLehre2/CIRfeat.txt");
    loadFeat(Detector_colored::_feat_rec, "./resource/Germany/svmLehre2/RECfeat.txt");
#else
    Detector_colored::_cir_model = svm_load_model("./resource/Germany/svm/cir_model.txt");
    Detector_colored::_rec_model = svm_load_model("./resource/Germany/svm/rec_model.txt");
    Detector_colored::_tri_model = svm_load_model("./resource/Germany/svm/tri_model.txt");
	Detector_colored::_cir_model_added = svm_load_model("./resource/Germany/svm/cir_model_added.txt");
    
    Detector_colored::loadFeat(Detector_colored::_feat_cir, "./resource/Germany/svm/CIRfeat.txt");
    Detector_colored::loadFeat(Detector_colored::_feat_rec, "./resource/Germany/svm/RECfeat.txt");
    Detector_colored::loadFeat(Detector_colored::_feat_tri, "./resource/Germany/svm/TRIfeat.txt");
	Detector_colored::loadFeat(Detector_colored::_feat_cir_added, "./resource/Germany/svm/CIRfeat_added.txt");
#endif
}

Detector_colored::Detector_colored(float highStep, double dist_per_piexl,int horizon_line,int featureNum) :Detector(highStep,dist_per_piexl,horizon_line),
										_MAX_NUM_FEATURES(featureNum), 
                                       _MAX_RED_CANDIDATES(10),
                                       _INVALID_TYPE(100)
{
#if WRITE_IMAGE == 1
	_triImageID = 0;
	_recImageID = 0;
	_cirImageID = 0;
#endif
#ifdef TRACK_ENABLE
	_frameNumber = 0;
#endif
    _sw = 32;
    _d.winSize = Size(_sw, _sw);
    _d.blockSize = Size(_sw/4, _sw/4);
    _d.blockStride =  Size(_sw/8, _sw/8);
    _d.cellSize = Size(_sw/8, _sw/8);
    _d.nbins = 8;

    _kernel_size = 3;
    _canny_low_Threshold = 50;
    _canny_high_Threshold = 300;
    _saturation_Threshold = 20;

    _ratioT = 30; 
    _areaT = 250;
}

int Detector_colored::contoursSelect(vector<vector<Point>> &contours,int *validIdx, double*validVal, double ratioT, int *maxIndex, double *maxValue) 
{
    int totalValid = 0;
    int cSize = contours.size();
    // find right shape.
    for(int idx = 0; idx< cSize; idx++)
    {
        double area = contourArea( contours[idx]); 
        double length = arcLength( contours[idx],true); 
        bool insideFlag = false;
		if ((length*length <= area*double(ratioT)) && ( (area > 29*29))) // && (area < 150*150)
		{
            // test one road sign whether insides other road sign.
            Rect rect = boundingRect(contours[idx]);
            Point center;
            center.x = rect.x + rect.width/2;
            center.y = rect.y + rect.height/2;
            for(int idx1 = 0; idx1 != idx; idx1 < cSize)
            {   
               double dist = pointPolygonTest(contours[idx1],center,false);
               if(dist == 1)
               {
                   insideFlag = true;
                   break;
               }
                ++idx1;
            }
            if(insideFlag == false)
            {
                validIdx[totalValid] = idx;
                validVal[totalValid] = area;
                totalValid ++;
            }
		}	 
    }
    int canNumber = min(totalValid,_MAX_RED_CANDIDATES);

    //find max 10 contourArea    
    for (int i = 0; i < canNumber; i++)
    {
        double maxV = 0;
        int maxI = 0;
        for(int idx = 0; idx< totalValid; idx++)
        {
            int index = validIdx[idx];
            double area = validVal[idx]; 
            if (area > maxV)
            {
                maxV = area;
                maxI = idx;
            }
        }
        if ((maxV > double (_areaT)))
        {
            maxIndex[i] = validIdx[maxI];
            maxValue[i] = maxV;
        }
        else
        {
            canNumber = i;
            break;
        }
        validVal[maxI] = - 1; 
    }
    return canNumber;
}

Detector::Shape Detector_colored::ShapeDetect(vector<Point> &approx, InputArray &curve) 
{
    Shape Shape = unknown;
    int vtcH = approx.size();
    if ((vtcH == 3)||(vtcH == 4))
    {
        // Number of vertices of polygonal curve
        std::vector<double> cosH;
        // Get the cosines of all corners
        {
            for (int j = 0; j < vtcH; j++) cosH.push_back(angle(approx[j%vtcH], approx[(j+2)%vtcH], approx[(j+1)%vtcH]));

            // Sort ascending the cosine values
            std::sort(cosH.begin(), cosH.end());                                
        }
        // Get the lowest and the highest cosine
        double mincosH = cosH.front();
        double maxcosH = cosH.back();
        // Use the degrees obtained above and the number of vertices to determine the shape of the contour                            
        if (vtcH == 3)
        {
            if(mincosH >= 0.34 && maxcosH <= 0.64)            // 60+/-10 = 50/70.
            {                
                Shape = triangles;
            }
        }
        else 
        {
            if(mincosH >= -0.2 && maxcosH <= 0.2)
            {
                cv::Rect r = cv::boundingRect(curve);
                if (std::abs(1 - ((double)r.width / r.height)) <= 0.2)
                {            
                    Shape = rectangles;
                }                
            }
        }
    }
    else if(vtcH > 6)
    {
        double area = cv::contourArea(curve);
        cv::Rect r = cv::boundingRect(curve);
        int radius = r.width / 2;

        if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
            std::abs(1 - (area / (CV_PI *radius * radius))) <= 0.2)
        {            
            Shape = circles;
        }
    }
    return Shape;
}
///////////////////////////////////////////////////////////////////////////////////////////
Mat Detector_colored::ID2Image(int target)
{
	Mat s;
	switch (target)
    {
	case 10100:
		{
			s = _image[0];
			break;
		}
	case 10200:
		{
			s = _image[1];
			break;
		}
	case 12300:
		{
			s = _image[2];
			break;
		}
	case 13100:
		{
			s = _image[3];
			break;
		}
	case 13310:
		{
			s = _image[4];
			break;
		}
	case 13810:
		{
			s = _image[5];
			break;
		}
	case 20500:
		{
			s = _image[6];
			break;
		}
	case 20600:
		{
			s = _image[7];
			break;
		}
	case 20910:
		{
			s = _image[8];
			break;
		}
	case 20930:
		{
			s = _image[9];
			break;
		}
	case 21500:
		{
			s = _image[10];
			break;
		}
	case 22220:
		{
			s = _image[11];
			break;
		}
	case 22400:
		{
			s = _image[12];
			break;
		}
	case 23700:
		{
			s = _image[13];
			break;
		}
	case 23900:
		{
			s = _image[14];
			break;
		}
	case 24000:
		{
			s = _image[15];
			break;
		}
	case 24100:
		{
			s = _image[16];
			break;
		}
	case 25000:
		{
			s = _image[17];
			break;
		}
	case 25400:
		{
			s = _image[18];
			break;
		}
	case 25900:
		{
			s = _image[19];
			break;
		}
	case 26100:
		{
			s = _image[20];
			break;
		}
	case 26210:
		{
			s = _image[21];
			break;
		}
	case 26700:
		{
			s = _image[22];
			break;
		}
	case 27452:
		{
			s = _image[23];
			break;
		}
	case 27453:
		{
			s = _image[24];
			break;
		}
	case 27454:
		{
			s = _image[25];
			break;
		}
	case 27455:
		{
			s = _image[26];
			break;
		}
	case 27456:
		{
			s = _image[27];
			break;
		}
	case 27458:
		{
			s = _image[28];
			break;
		}
	case 27600:
		{
			s = _image[29];
			break;
		}
	case 28300:
		{
			s = _image[30];
			break;
		}
	case 28400:
		{
			s = _image[31];
			break;
		}
	case 28500:
		{
			s = _image[32];
			break;
		}
	case 28600:
		{
			s = _image[33];
			break;
		}
	case 28700:
		{
			s = _image[34];
			break;
		}
	case 30100:
		{
			s = _image[35];
			break;
		}
	case 30600:
		{
			s = _image[36];
			break;
		}
	case 31400:
		{
			s = _image[37];
			break;
		}
	case 31401:
		{
			s = _image[38];
			break;
		}
	case 33100:
		{
			s = _image[39];
			break;
		}
	case 33600:
		{
			s = _image[40];
			break;
		}
	case 34100:
		{
			s = _image[41];
			break;
		}
	case 35010:
		{
			s = _image[42];
			break;
		}
	case 44100:
		{
			s = _image[43];
			break;
		}
	case 44200:
		{
			s = _image[44];
			break;
		}
	case 99900:
		{
			s = _image[45];
			break;
		}
	case 10310:
		{
			s = _image[46];
			break;
		}
	case 10320:
		{
			s = _image[47];
			break;
		}
	case 22210:
		{
			s = _image[48];
			break;
		}

	default:
		break;
	}
	return s;
}

void Detector_colored::CannyThreshold(Mat src_gray,Mat &dst)
{
    // Reduce noise with a kernel 3x3
    blur(src_gray, dst, Size(_kernel_size, _kernel_size));
    // Canny detector
    Canny(dst, dst, _canny_low_Threshold, _canny_high_Threshold, _kernel_size);
}

int Detector_colored::targetClassify(cv::Mat image,InputArray& curve,std::vector<int> &feat,struct svm_model *model) 
{
    int target = 0;

    //Get Image ROI
    cv::Rect r = cv::boundingRect(curve);
    Mat image_roi = image(r);

    //Resize and Convert Color
    Mat imag;
    resize(image_roi, imag, Size(_sw,_sw));
    cvtColor(imag, imag, CV_RGB2GRAY);

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

// Determine which tracked point should be accepted
bool Detector_colored::acceptTrackedPoint(int i,std::vector<uchar> status,std::vector<cv::Point2f> *points) 
{
    return status[i];// &&
        // if point has moved
        //(abs(points[0][i].x-points[1][i].x)+(abs(points[0][i].y-points[1][i].y))>2);
}

// Handle the currently tracked points
void Detector_colored::handleTrackedPoints(cv:: Mat &output,std::vector<cv::Point2f> *points,std::vector<cv::Point2f>initial) 
{
    // for all tracked points
    for(int i= 0; i < points[1].size(); i++ ) {
        // draw line and circle
        cv::line(output, initial[i], points[1][i], cv::Scalar(255,255,255));
        cv::circle(output, points[1][i], 3, cv::Scalar(255,255,255),-1);
    }
}

#if WRITE_IMAGE == 1
int mkdir_r(const char *path) {
	if (path == NULL) {
		return -1;
	}
	char *temp = strdup(path);
	char *pos = temp;

	/* remove the beginning './' or '/' */
	if (strncmp(temp, "/", 1) == 0) {
		pos += 1;
	} else if (strncmp(temp, "./", 2) == 0) {
		pos += 2;
	}
	/* make dir round */
	for ( ; *pos != '\0'; ++ pos) {
		if (*pos == '/') {
			*pos = '\0';
			mkdir(temp);
			//printf("for %s\n", temp);
			*pos = '/';
		}
	}
	/* if the last level dir is end with '/'
	stop loop at '\0'
	not make the last level dir */
	//if (*(pos - 1) != '/') {
	//	printf("if %s\n", temp);
	//	mkdir(temp);
	//}
	free(temp);
	return 1;
}
#endif

int Detector_colored::TS_classify(Detector::Shape shape,Mat image,InputArray curve,string path)
{
    // default type
    int type = 0;
    if (shape == triangles)
    { 
		type = targetClassify(image,curve,_feat_tri,_tri_model);
#if WRITE_IMAGE == 1
        {
            cv::Rect r = cv::boundingRect(curve);
            Mat image_roi = image(r);
            char currFileName[500];
            int stringLen = path.size();
            sprintf_s( currFileName, 100, (char *)path.data());
            sprintf_s(&currFileName[stringLen], 400, "./captureImg/TRI/%05d/%06d.jpg",type,_triImageID++);
			mkdir_r(currFileName);
            imwrite(currFileName, image_roi);
        }
#endif 
    }
    else if (shape == rectangles)
    {    
		 type = targetClassify(image,curve,_feat_rec,_rec_model);
#if WRITE_IMAGE == 1
        {
            cv::Rect r = cv::boundingRect(curve);
            Mat image_roi = image(r);
            char currFileName[500];
            int stringLen = path.size();
            sprintf_s( currFileName, 100, (char *)path.data());
            sprintf_s( &currFileName[stringLen], 400, "./captureImg/REC/%05d/%06d.jpg",type,_recImageID++);
			mkdir_r(currFileName);  
            imwrite(currFileName, image_roi);
        }
#endif                     
    }
    else if (shape == circles)
    {
		type = targetClassify(image,curve,_feat_cir,_cir_model);
		if ( type == 28400 || type == 28500 || type == 28700 )
		{
			type = targetClassify(image,curve,_feat_cir_added,_cir_model_added);
		}
#if WRITE_IMAGE == 1
		{
			cv::Rect r = cv::boundingRect(curve);
			Mat image_roi = image(r);
			char currFileName[500];
			int stringLen = path.size();
			sprintf_s( currFileName, 100, (char *)path.data());
			sprintf_s( &currFileName[stringLen], 400, "./captureImg/CIR/%05d/%06d.jpg",type,_recImageID++);
			mkdir_r(currFileName);
			imwrite(currFileName, image_roi);
		}
#endif                                                     
    }
    return type;
}

void Detector_colored::trafficSignDetect(Mat image, TS_Structure &target)
{    
    //////////////////////////////////////////////////////////////////////////////
    // Variable for track
    //////////////////////////////////////////////////////////////////////////////
#ifdef TRACK_ENABLE
    static cv::Mat gray;                        // current gray-level image
    static cv::Mat gray_prev;                    // previous gray-level image
    static std::vector<cv::Point2f> points[2];    // tracked features from 0->1
    static std::vector<cv::Point2f> initial;    // initial position of tracked points
    static std::vector<int> frameList;            // frameNumber list
    static std::vector<int> typeList;            // type list    
    std::vector<uchar> status;                    // status of tracked center
    std::vector<float> err;                        // error in tracking
#endif
    cv::Point2f center;                            // detected center    
    //////////////////////////////////////////////////////////////////////////////
    cv::Mat imag;
    image.copyTo(imag);
    // 1. convert Color to HSV
    cv::Mat hsv;            
    cv::cvtColor(image, hsv, CV_BGR2HSV);

    std::vector<cv::Mat> planes;    
    cv::split(hsv,planes);                                
    // 3. show saturation to LCD. 
    Mat satImage = planes[1];    
#ifdef TRACK_ENABLE
    satImage.copyTo(gray);
#endif
    // 4. the second sub-windows for display.
    Mat subWin2;
    resize(satImage,subWin2, Size(satImage.cols/3,satImage.rows/3));
    cvtColor(subWin2, subWin2, COLOR_GRAY2RGB);
    // 5. find Contours with canny detector
    threshold(satImage, satImage, _saturation_Threshold, 255, THRESH_TOZERO); 
    CannyThreshold(satImage,satImage);
    // 6. the third sub-windows for display
    Mat subWin3;
    resize(satImage,subWin3, Size(satImage.cols/3,satImage.rows/3));
    cvtColor(subWin3, subWin3, COLOR_GRAY2RGB);
    // 7. find findContours.
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;    
    findContours( satImage, contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

    // 8. shape detection and signs classfication.
    int k = 0;
    int totalNumber = 0;
    {
        int cSize = contours.size();

        // allocate memory for each curve.
        int    *validIdx    = new int[cSize];
        double *validVal    = new double[cSize];
        int    *maxIndex    = new int[_MAX_RED_CANDIDATES];
        double *maxValue    = new double[_MAX_RED_CANDIDATES];

        // select candidates
        int canNumber = contoursSelect(
            contours,
            validIdx,
            validVal,
            _ratioT, 
            maxIndex, 
            maxValue); 

        RNG rng(12345);
   
        for(int idx = 0; idx< canNumber; idx++)
        {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            //
            int index   = maxIndex[idx];
            double area = maxValue[idx]; 
            InputArray curve = contours[index];
            double length = arcLength( curve,true); 

            // calculate center
            cv::Rect r = cv::boundingRect(curve);

            center.x = r.x+r.width/2;
            center.y = r.y+r.height/2;

            Shape shape;
            std::vector<cv::Point> approx,hull;

            // approximate a curve with polygon.
            convexHull(curve, hull);    
            approxPolyDP(Mat(hull), approx,length*0.03, true);

            // shape detection
            shape = ShapeDetect(approx,curve); 

   //         if(unknown != shape)
			//{
			//	polylines(image,approx,1,Scalar(0,255,255),2);
			//}
#if WRITE_IMAGE != 1
            //drawLine(image,approx,color);
#endif
            // traffic sign classification             
            int type = TS_classify(shape,image,curve);

#if RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE
			if ( type == 28500 || type == 28700 )
			{
				type = 28400;
			}
#endif

            // if is valid traffic sign type
            if(type > _INVALID_TYPE)
            {    
                int flag = 1;                                

                for (int j = 0; j < k; j++)
                {
                    cv::Point2f center2 = target.trafficSign[j].center;
                    //if same type and same centers,jump off
                    if(cv::norm(center2-center)< 10) // if same target!
                    {
                        if (type == target.trafficSign[j].type)
                        {
                            flag = 0;
                            break;
                        }
                    }
                }//end for    

                if(((flag ==1)  && (type != 31400))// FIXME: remove 31400 for Frankfurt video
                    && !((type == 20500) && (center.x  < image.cols/2.0)))//FIXME:porcess the two 20500 sign for Frankfurt video                  
                {
                    TS_Structure::TS_element detectSign;
                    detectSign.type = type;
                    detectSign.area = area;
                    detectSign.rect = r;
                    detectSign.center = center;         
                    target.trafficSign.push_back(detectSign);
#ifdef TRACK_ENABLE                     
                    points[0].push_back (center);
                    initial.push_back (center);
                    frameList.push_back (_frameNumber);
                    typeList.push_back (type);
#endif                    
                    //cout<< " frameNumber = "<<frameNumber <<"  Sign type = " << ID2Name(type) << endl;
                    // display the label.                    
                    rectangle(image, r , Scalar(0,255,0), 2, 8, 0);
                /*    string s = ID2Name(type); 
                    detector.setLabel(image,s.c_str(), contours[index]);*/
                    k++;                        
                }
            }//end if
            else
            {
#if 0
                //FIXME: process 30600
                if( (type == 0) && (shape = rectangles))
                {
                    Mat image_roi = imag(r);

                    //cv::namedWindow("Original Image");                            // define the window
                    //cv::imshow("Original Image", image_roi);                    // show the image
                    //cv::waitKey(1);

                    // 1. convert Color to HSV
                    cv::Mat hsvR;            
                    cv::cvtColor(image_roi, hsvR, CV_BGR2HSV);

                    std::vector<cv::Mat> planesR;    
                    cv::split(hsvR,planesR);

                    Mat hMat = planesR[0];

                     /// set bin number
                     int histSize = 36;
                     float range[] = { -180, 180 } ;
                     const float* histRange = { range };
                     bool uniform = true; bool accumulate = false;
                     Mat y_hist;
                     double minVal,maxVal;
                     Point minIdx,maxIdx;

                     calcHist(&planesR[0], 1, 0, Mat(), y_hist, 1, &histSize, &histRange, uniform, accumulate);
                     minMaxLoc(y_hist, &minVal, &maxVal, &minIdx, &maxIdx);

                     double ratio = (float)r.width /(float)r.height;
                     if(maxIdx == Point(0,20) && (ratio > 0.9) && (ratio < 1.1))
                     {
                        TS_Structure::TS_element detectSign;
                        detectSign.type = 30600;
                        detectSign.area = area;
                        detectSign.rect = r;
                        detectSign.center = center;    
                        rectangle(image, r , Scalar(0,255,0), 2, 8, 0);
                        target.trafficSign.push_back(detectSign);
                        k++;
                     }
                }
#endif
            }
        }        

        delete validIdx;
        delete validVal;
        delete maxIndex;
        delete maxValue;
    }

    // for tracking
#ifdef TRACK_ENABLE 
    {
        // 1.first image of the sequence
        if(gray_prev.empty())
            gray.copyTo(gray_prev);

        // 2. track center
        if (points[0].size())
        {
            cv::calcOpticalFlowPyrLK(gray_prev, gray, // 2 consecutive images
                points[0], // input point position in first image
                points[1], // output point postion in the second image
                status,    // tracking success
                err);      // tracking error
        }
        else
        {
            points[1].resize(0);
            initial.resize(0);
        }
        // 2. loop over the tracked points to reject the undesirables
        int k=0;
        for( int i= 0; i < points[1].size(); i++ ) 
        {
            // do we keep this point?
            if (acceptTrackedPoint(i,status,points)) 
            {
                // keep this point in vector
                initial[k]= initial[i];
                points[1][k] = points[1][i];
                frameList[k] = frameList[i];
                typeList[k] = typeList[i];    

                k++;
            }
        }
        // eliminate unsuccesful points
        {
            points[1].resize(k);
            initial.resize(k);
            frameList.resize(k);
            typeList.resize(k);
        }

        //cout<< " track size = "<< k << endl;

        // 3. handle the accepted tracked points
        handleTrackedPoints(image,points,initial);

        // 4. current points and image become previous ones
        std::swap(points[1], points[0]);
        cv::swap(gray_prev, gray);
    }
#endif

    {
        //Mat background = Mat::zeros(image.rows, 4*image.cols/3, CV_8UC3);
        Mat background(image.rows,4*image.cols/3, CV_8UC3,Scalar(128,128,128));    

        for(int i = 0; i< min(target.trafficSign.size(),4); i++)
        {
            int type = target.trafficSign[i].type;
            Rect rect = target.trafficSign[i].rect;
            Mat hsv_roi = hsv(rect);
            Mat rgb_roi = imag(rect);
            Mat subWin1 = ID2Image(type);

            if (rgb_roi.data)
            {
                resize(rgb_roi,rgb_roi, Size(image.cols/12,image.rows/9));
                rgb_roi.copyTo(background(Rect(i*image.cols/12,0*image.rows/9,image.cols/12,image.rows/9)));
            }
            if (hsv_roi.data)
            {
                resize(hsv_roi,hsv_roi, Size(image.cols/12,image.rows/9));
                hsv_roi.copyTo(background(Rect(i*image.cols/12,1*image.rows/9,image.cols/12,image.rows/9)));
            }
            if (subWin1.data)
            {
                resize(subWin1,subWin1, Size(image.cols/12,image.rows/9));
                subWin1.copyTo(background(Rect(i*image.cols/12,2*image.rows/9,image.cols/12,image.rows/9)));
            }            
        }

        subWin2.copyTo(background(Rect(0,1*image.rows/3,image.cols/3,image.rows/3)));
        subWin3.copyTo(background(Rect(0,2*image.rows/3,image.cols/3,image.rows/3)));
        image.copyTo(background(Rect(image.cols/3,0,image.cols,image.rows)));
        
        for (int i=0; i<4; i++)
        {
            int x = image.cols/12 * i;
            for (int j=0; j<3;j++)
            {   
                int y = image.rows/9 * j;
                rectangle(background,Rect(x,y,image.cols/12,image.rows/9),Scalar(255,255,255), 1,8);
            }
        }        

        rectangle(background,Rect(0,0*image.rows/3,image.cols/3,image.rows/3),Scalar(255,255,255), 1,8);
        rectangle(background,Rect(0,1*image.rows/3,image.cols/3,image.rows/3),Scalar(255,255,255), 1,8);
        rectangle(background,Rect(0,2*image.rows/3,image.cols/3,image.rows/3),Scalar(255,255,255), 1,8);
        rectangle(background,Rect(image.cols/3,0,image.cols,image.rows),Scalar(255,255,255), 1,8);        

        //resize(background,background, Size(background.rows/4,background.cols/4));
        namedWindow( "VEHICLE DISPLAY",CV_WINDOW_NORMAL);
        imshow( "VEHICLE DISPLAY", background);    
    }
    waitKey(1);
}

}
