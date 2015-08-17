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
#include <opencv2/video/tracking.hpp>

#include "svm.h"
#include "Detection.h"
#include "database.h"

#define MAX_NUM_FEATURES   200
#define MAX_RED_CANDIDATES 10
#define WRITE_IMAGE 0

using namespace cv;
using namespace std;

struct svm_model *cir_model = svm_load_model("./Resource/cir_model.txt");
struct svm_model *rec_model = svm_load_model("./Resource/rec_model.txt");
struct svm_model *tri_model = svm_load_model("./Resource/tri_model.txt");

int feat_cir[1568] = {
#include "../Resource/CIRfeat.txt"
};
int feat_rec[1568] = {
#include "../Resource/RECfeat.txt"
};
int feat_tri[1568] = {
#include "../Resource/TRIfeat.txt"
};
Mat image27452 = cv::imread("./Resource/27452.png");
Mat image27453 = cv::imread("./Resource/27453.png");
Mat image27454 = cv::imread("./Resource/27454.png");
Mat image27455 = cv::imread("./Resource/27455.png");
Mat image27456 = cv::imread("./Resource/27456.png");
Mat image28300 = cv::imread("./Resource/28300.png");
Mat image28600 = cv::imread("./Resource/28600.png");
Mat image20600 = cv::imread("./Resource/20600.png");
Mat image22400 = cv::imread("./Resource/22400.png");
Mat image24000 = cv::imread("./Resource/24000.png");
Mat image23900 = cv::imread("./Resource/23900.png");
Mat image22220 = cv::imread("./Resource/22220.png");
Mat image99900 = cv::imread("./Resource/99900.png");
Mat image23700 = cv::imread("./Resource/23700.png");
Mat image31400 = cv::imread("./Resource/31400.png");
Mat image35010 = cv::imread("./Resource/35010.png");
Mat image30600 = cv::imread("./Resource/30600.png");
Mat image20500 = cv::imread("./Resource/20500.png");
Mat image30100 = cv::imread("./Resource/30100.png");
Mat image13100 = cv::imread("./Resource/13100.png");	
Mat image12300 = cv::imread("./Resource/12300.png");
Mat image13810 = cv::imread("./Resource/13810.png");
Mat image10100 = cv::imread("./Resource/10100.png");
Mat image13310 = cv::imread("./Resource/13310.png");


#if WRITE_IMAGE == 1
int triImageID = 0;
int recImageID = 0;
int cirImageID = 0;
#endif
///////////////////////////////////////////////////////////////////////////////////////////
double Detector::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
/**
* Helper function to display text in the center of a contour
*/
void Detector::setTitle(cv::Mat& im, const std::string label)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);

	cv::Point pt(0, text.height);
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(255,0,0), thickness, 8);
}
/**
* Helper function to display Label in the bottom of a contour
*/
void Detector::setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + (2*(r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(255,0,0), thickness, 8);
}

cv::Mat Detector::imageFilter(const cv::Mat& image) 
{
	cv::Mat imageFilted;
	cv::pyrDown(image, imageFilted, cv::Size(image.cols/2, image.rows/2));
	cv::pyrUp(imageFilted, imageFilted, image.size());
	return image;
}

int Detector::contoursSelect(vector<vector<Point>> &contours,int *validIdx, double*validVal, double ratioT, int *maxIndex, double *maxValue) 
{
	int totalValid = 0;
	int cSize = contours.size();
	// find right shape.
	for(int idx = 0; idx< cSize; idx++)
	{
		double area = contourArea( contours[idx]); 
		double length = arcLength( contours[idx],true); 

		if (length*length <= area*double(ratioT))
		{
			validIdx[totalValid] = idx;
			validVal[totalValid] = area;
			totalValid ++;
		}				
	}
	int canNumber = min(totalValid,MAX_RED_CANDIDATES);

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
		if ((maxV > double (areaT)))
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

Shape Detector::ShapeDetect(vector<cv::Point> &approx, InputArray &curve) 
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
			if(mincosH >= 0.34 && maxcosH <= 0.64)			// 60+/-10 = 50/70.
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
Mat ID2Image(int target) 
{
	Mat s;
	switch (target)
	{
	case 27452:
		{
			s = image27452;
			break;
		}
	case 27453:
		{
			s = image27453;
			break;
		}
	case 27454:
		{
			s = image27454;
			break;
		}
	case 27455:
		{
			s = image27455;
			break;
		}
	case 27456:
		{
			s = image27456;
			break;
		}
	case 28300:
		{
			s = image28300;
			break;
		}
	case 28600:
		{
			s = image28600;
			break;
		}
	case 20600:
		{
			s = image20600;
			break;
		}
	case 22400:
		{
			s = image22400;
			break;
		}
	case 99900:
		{
			s = image99900;
			break;
		}
	case 24000:
		{
			s = image24000;
			break;
		}
	case 23900:
		{
			s = image23900;
			break;
		}
	case 22220:
		{
			s = image22220;
			break;
		}
	case 23700:
		{
			s = image23700;
			break;
		}
	case 31400:
		{
			s = image31400;
			break;
		}
	case 35010:
		{
			s = image35010;
			break;
		}
	case 30600:
		{
			s = image30600;
			break;
		}
	case 20500:
		{
			s = image20500;
			break;
		}
	case 13100:
		{
			s = image13100;
			break;
		}
	case 30100:
		{
			s = image30100;
			break;
		}
	case 12300:
		{
			s = image12300;
			break;
		}
	case 13810:
		{
			s = image13810;
			break;
		}
	case 10100:
		{
			s = image10100;
			break;
		}
	case 13310:
		{
			s = image13310;
			break;
		}
	default:
		s = image99900;
		break;
	}
	return s;
}

void Detector::CannyThreshold(Mat src_gray,Mat &dst)
{
	// Reduce noise with a kernel 3x3
	blur(src_gray, dst, Size(kernel_size,kernel_size));
	// Canny detector
	Canny(dst, dst, canny_low_Threshold, canny_high_Threshold, kernel_size);
}

void drawLine(cv::Mat image,std::vector<cv::Point> &approx,Scalar color)
{
	std::vector<cv::Point>::const_iterator itp= approx.begin();
	while (itp!=(approx.end()-1)) 
	{
		line(image,*itp,*(itp+1),color,2);
		++itp;
	}
	// last point linked to first point
	cv::line(image,*(approx.begin()),*(approx.end()-1),color,2);
}

int Detector::targetClassify(cv::Mat image,InputArray& curve,const int *feat,struct svm_model *model) 
{
	int target = 0;

	//Get Image ROI
	cv::Rect r = cv::boundingRect(curve);
	Mat image_roi = image(r);

	//Resize and Convert Color
	Mat imag;
	resize(image_roi, imag, Size(sw,sw));
	cvtColor(imag, imag, CV_RGB2GRAY);

	//Compute HOG features.
	vector<float> descriptorsValues;
	vector<Point> locations;
	d.compute(imag, descriptorsValues,Size(0,0), Size(0,0), locations);

	//malloc memory for each svm_node
	struct svm_node node;
	node.values = new double[MAX_NUM_FEATURES];
	node.dim = MAX_NUM_FEATURES;

	//Select reduced Features
	for (int k = 0;k < MAX_NUM_FEATURES; k++)
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
bool acceptTrackedPoint(int i,std::vector<uchar> status,std::vector<cv::Point2f> *points) 
{
	return status[i];// &&
		// if point has moved
		//(abs(points[0][i].x-points[1][i].x)+(abs(points[0][i].y-points[1][i].y))>2);
}

// Handle the currently tracked points
void handleTrackedPoints(cv:: Mat &output,std::vector<cv::Point2f> *points,std::vector<cv::Point2f>initial) 
{
	// for all tracked points
	for(int i= 0; i < points[1].size(); i++ ) {
		// draw line and circle
		cv::line(output, initial[i], points[1][i], cv::Scalar(255,255,255));
		cv::circle(output, points[1][i], 3, cv::Scalar(255,255,255),-1);
	}
}

int TS_classify(Detector detector,Shape shape,Mat image,InputArray curve)
{
	// default type
	int type = 0;
	if (shape == triangles)
	{					
#if WRITE_IMAGE == 1
		{
			cv::Rect r = cv::boundingRect(curve);
			Mat image_roi = image(r);
			char currFileName[1000];
			sprintf_s( currFileName, 1000, ".\\testImage\\TRI\\%05d.jpg",triImageID++);
			imwrite(currFileName, image_roi);
		}
#else
		type = detector.targetClassify(image,curve,feat_tri,tri_model); 
#endif
	}
	else if (shape == rectangles)
	{					
#if WRITE_IMAGE == 1
		{
			cv::Rect r = cv::boundingRect(curve);
			Mat image_roi = image(r);
			char currFileName[1000];
			sprintf_s( currFileName, 1000, ".\\testImage\\REC\\%05d.jpg",recImageID++);
			imwrite(currFileName, image_roi);
		}
#else
		type = detector.targetClassify(image,curve,feat_rec,rec_model); 					
#endif
	}
	else if (shape == circles)
	{
#if WRITE_IMAGE == 1
		{
			cv::Rect r = cv::boundingRect(curve);
			Mat image_roi = image(r);
			char currFileName[1000];
			sprintf_s( currFileName, 1000, ".\\testImage\\CIR\\%05d.jpg",cirImageID++);
			imwrite(currFileName, image_roi);
		}
#else
		type = detector.targetClassify(image,curve,feat_cir,cir_model); 					
#endif								
	}
	return type;
}

int Detector::TS_Detect_withTrack(cv::Mat image,Detector detector,TS_Structure &target,int frameNumber) 
{	
	//////////////////////////////////////////////////////////////////////////////
	// Variable for track
	//////////////////////////////////////////////////////////////////////////////
#ifdef TRACK_ENABLE
	static cv::Mat gray;						// current gray-level image
	static cv::Mat gray_prev;					// previous gray-level image
	static std::vector<cv::Point2f> points[2];	// tracked features from 0->1
	static std::vector<cv::Point2f> initial;	// initial position of tracked points
	static std::vector<int> frameList;			// frameNumber list
	static std::vector<int> typeList;			// type list	
	std::vector<uchar> status;					// status of tracked center
	std::vector<float> err;						// error in tracking
#endif
	cv::Point2f center;							// detected center	
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
	threshold(satImage, satImage, saturation_Threshold, 255, THRESH_TOZERO); 
	CannyThreshold(satImage,satImage);
	// 6. the third sub-windows for display
	Mat subWin3;
	resize(satImage,subWin3, Size(satImage.cols/3,satImage.rows/3));
	cvtColor(subWin3, subWin3, COLOR_GRAY2RGB);
	// 7. find findContours.
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;	
	findContours( satImage, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

	// 8. shape detection and signs classfication.
	int k = 0;
	int totalNumber = 0;
	{
		int cSize = contours.size();

		// allocate memory for each curve.
		int    *validIdx    = new int[cSize];
		double *validVal    = new double[cSize];
		int    *maxIndex    = new int[MAX_RED_CANDIDATES];
		double *maxValue    = new double[MAX_RED_CANDIDATES];

		// select candidates
		int canNumber = detector.contoursSelect(
			contours,
			validIdx,
			validVal,
			ratioT, 
			maxIndex, 
			maxValue); 

		RNG rng(12345);

		// clear the output target.
		for(int idx = 0; idx< MAX_RED_CANDIDATES; idx++)
		{
			target.TS_type[idx] = 0;
		}	

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
			shape = detector.ShapeDetect(approx,curve); 

#if WRITE_IMAGE != 1
			//drawLine(image,approx,color);
#endif
			// traffic sign classification 			
			int type = TS_classify(detector,shape,image,curve);

			// if is valid traffic sign type
			if(type!=0)
			{	
				int flag = 1;								

				for (int j = 0; j < k; j++)
				{
					cv::Point2f center2 = target.TS_center[j];
					//if same type and same centers,jump off
					if ((type == target.TS_type[j])&&(cv::norm(center2-center)< 10))
					{
						flag = 0;
						break;
					}					
				}//end for	

				if (flag ==1)
				{
					target.TS_type[k] = type;
					target.TS_area[k] = area;
					target.TS_rect[k] = r;
					target.TS_center[k] = center;					
#ifdef TRACK_ENABLE 					
					points[0].push_back (center);
					initial.push_back (center);
					frameList.push_back (frameNumber);
					typeList.push_back (type);
#endif					
					//cout<< " frameNumber = "<<frameNumber <<"  Sign type = " << ID2Name(type) << endl;
					// display the label.					
					rectangle(image, r , Scalar(0,255,0), 2, 8, 0);
					string s = ns_database::ID2Name(type); 
					detector.setLabel(image,s.c_str(), contours[index]);
					k++;						
				}
			}//end for
		}
		totalNumber = k;		
		target.totalNumber = k;

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

		for(int i = 0; i< min(totalNumber,4); i++)
		{
			int type = target.TS_type[i];
			Rect rect = target.TS_rect[i];
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

		namedWindow( "VEHICLE DISPLAY",CV_WINDOW_NORMAL);
		imshow( "VEHICLE DISPLAY", background);	


	}
	waitKey(1);
	return totalNumber;
}
