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
#include "detection_colored.h"
#include "database.h"

using namespace cv;
using namespace std;

namespace ns_detection
{

Detector_colored::Detector_colored() : _MAX_NUM_FEATURES(200), _MAX_RED_CANDIDATES(10)
{
    _sw = 32;
    _d.winSize = Size(_sw, _sw);
    _d.blockSize = Size(_sw/4, _sw/4);
    _d.blockStride =  Size(_sw/8, _sw/8);
    _d.cellSize = Size(_sw/8, _sw/8);
    _d.nbins = 8;

    _kernel_size = 3;
    _canny_low_Threshold = 50;
    _canny_high_Threshold = 300;
    _saturation_Threshold = 50;

    _ratioT = 30; 
    _areaT = 250;

    _image.push_back(imread("./resource/Germany/png/27452.png"));
    _image.push_back(imread("./resource/Germany/png/27453.png"));
    _image.push_back(imread("./resource/Germany/png/27454.png"));
    _image.push_back(imread("./resource/Germany/png/27455.png"));
    _image.push_back(imread("./resource/Germany/png/27456.png"));
    _image.push_back(imread("./resource/Germany/png/28300.png"));
    _image.push_back(imread("./resource/Germany/png/28600.png"));
    _image.push_back(imread("./resource/Germany/png/20600.png"));
    _image.push_back(imread("./resource/Germany/png/22400.png"));
    _image.push_back(imread("./resource/Germany/png/24000.png"));
    _image.push_back(imread("./resource/Germany/png/23900.png"));
    _image.push_back(imread("./resource/Germany/png/22220.png"));
    _image.push_back(imread("./resource/Germany/png/99900.png"));
    _image.push_back(imread("./resource/Germany/png/23700.png"));
    _image.push_back(imread("./resource/Germany/png/31400.png"));
    _image.push_back(imread("./resource/Germany/png/35010.png"));
    _image.push_back(imread("./resource/Germany/png/30600.png"));
    _image.push_back(imread("./resource/Germany/png/20500.png"));
    _image.push_back(imread("./resource/Germany/png/30100.png"));
    _image.push_back(imread("./resource/Germany/png/13100.png"));
    _image.push_back(imread("./resource/Germany/png/12300.png"));
    _image.push_back(imread("./resource/Germany/png/13810.png"));
    _image.push_back(imread("./resource/Germany/png/10100.png"));
    _image.push_back(imread("./resource/Germany/png/13310.png"));

    _cir_model = svm_load_model("./resource/Germany/svm/cir_model.txt");
    _rec_model = svm_load_model("./resource/Germany/svm/rec_model.txt");
    _tri_model = svm_load_model("./resource/Germany/svm/tri_model.txt");
    
    loadFeat(_feat_cir, "./resource/Germany/svm/CIRfeat.txt");
    loadFeat(_feat_rec, "./resource/Germany/svm/RECfeat.txt");
    loadFeat(_feat_tri, "./resource/Germany/svm/TRIfeat.txt");
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

        if (length*length <= area*double(ratioT))
        {
            validIdx[totalValid] = idx;
            validVal[totalValid] = area;
            totalValid ++;
        }                
    }
    int canNumber = min(totalValid, _MAX_RED_CANDIDATES);

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
    case 27452:
        {
            s = _image[0];
            break;
        }
    case 27453:
        {
            s = _image[1];
            break;
        }
    case 27454:
        {
            s = _image[2];
            break;
        }
    case 27455:
        {
            s = _image[3];
            break;
        }
    case 27456:
        {
            s = _image[4];
            break;
        }
    case 28300:
        {
            s = _image[5];
            break;
        }
    case 28600:
        {
            s = _image[6];
            break;
        }
    case 20600:
        {
            s = _image[7];
            break;
        }
    case 22400:
        {
            s = _image[8];
            break;
        }
    case 24000:
        {
            s = _image[9];
            break;
        }
    case 23900:
        {
            s = _image[10];
            break;
        }
    case 22220:
        {
            s = _image[11];
            break;
        }
    case 99900:
        {
            s = _image[12];
            break;
        }
    case 23700:
        {
            s = _image[13];
            break;
        }
    case 31400:
        {
            s = _image[14];
            break;
        }
    case 35010:
        {
            s = _image[15];
            break;
        }
    case 30600:
        {
            s = _image[16];
            break;
        }
    case 20500:
        {
            s = _image[17];
            break;
        }
    case 30100:
        {
            s = _image[18];
            break;
        }
    case 13100:
        {
            s = _image[19];
            break;
        }
    case 12300:
        {
            s = _image[20];
            break;
        }
    case 13810:
        {
            s = _image[21];
            break;
        }
    case 10100:
        {
            s = _image[22];
            break;
        }
    case 13310:
        {
            s = _image[23];
            break;
        }
    default:
        s = _image[0];
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

int Detector_colored::TS_classify(Detector::Shape shape,Mat image,InputArray curve)
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
            sprintf_s( currFileName, 1000, ".\\testImage\\TRI\\%05d.jpg",_triImageID++);
            imwrite(currFileName, image_roi);
        }
#else
        type = targetClassify(image,curve,_feat_tri,_tri_model); 
#endif
    }
    else if (shape == rectangles)
    {                    
#if WRITE_IMAGE == 1
        {
            cv::Rect r = cv::boundingRect(curve);
            Mat image_roi = image(r);
            char currFileName[1000];
            sprintf_s( currFileName, 1000, ".\\testImage\\REC\\%05d.jpg",_recImageID++);
            imwrite(currFileName, image_roi);
        }
#else
        type = targetClassify(image,curve,_feat_rec,_rec_model);                     
#endif
    }
    else if (shape == circles)
    {
#if WRITE_IMAGE == 1
        {
            cv::Rect r = cv::boundingRect(curve);
            Mat image_roi = image(r);
            char currFileName[1000];
            sprintf_s( currFileName, 1000, ".\\testImage\\CIR\\%05d.jpg",_cirImageID++);
            imwrite(currFileName, image_roi);
        }
#else
        type = targetClassify(image,curve,_feat_cir,_cir_model);                     
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
    findContours( satImage, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

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

        // clear the output target.
        for(int idx = 0; idx< _MAX_RED_CANDIDATES; idx++)
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
            shape = ShapeDetect(approx,curve); 

#if WRITE_IMAGE != 1
            //drawLine(image,approx,color);
#endif
            // traffic sign classification             
            int type = TS_classify(shape,image,curve);

            // if is valid traffic sign type
            if(type!=0)
            {    
                int flag = 1;                                

                for (int j = 0; j < k; j++)
                {
                    cv::Point2f center2 = target.TS_center[j];
                    //if same type and same centers,jump off
                    if(cv::norm(center2-center)< 10) // if same target!
                    {
                        if (type == target.TS_type[j])
                        {
                            flag = 0;
                            break;
                        }
                        else if ((type == 20600)&&(target.TS_type[j] ==27453)) // Fixme later, fast workaround to avoid false alarm of SL30
                        {
                            flag = 0;
                            break;
                        }
                        else if ((type == 27453)&&(target.TS_type[j] ==20600)) // update the existing 20600.
                        {
                            flag = 2;
                            target.TS_type[j] = type;
                            target.TS_area[j] = area;
                            target.TS_rect[j] = r;
                            target.TS_center[j] = center;    
                            break;
                        } 
                    }
                }//end for    

                if ((flag ==1)&&(type != 28300)&&(type != 28600)) // remove 28300 && 28600
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
                    //detector.setLabel(image,s.c_str(), contours[index]);
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
}
}
