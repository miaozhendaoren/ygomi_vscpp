/******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  lineDetection.cpp
* @brief Road detection source file
*
* Change Log:
*      Date                Who             What
*      2015/05/20         Bingtao Gao      Create
*******************************************************************************
*/

#include <windows.h>
#include <opencv2\opencv.hpp>
#include "lineDetection.h"

using namespace std;
using namespace cv;

void LineDetect::initialVanishPointKF(KalmanFilter &KF,Size S)
{
    Mat_<float> state(4, 1); /* (x, y, Vx, Vy) */    
    Mat processNoise(4, 1, CV_32F);    


    KF.statePre.at<float>(0) = S.width/2;
    KF.statePre.at<float>(1) = S.height/2;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
    KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

    setIdentity(KF.measurementMatrix);
    //i f the value of Q is high, it will mean that the signal you are measuring is varies quickly
    //and you need the filter to be adaptable. 
    //If it is small, then big variations will be attributed to noise in the measure.
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    //If the value of R is high (compared to Q), it will indicate that the measuring is noisy so it will be filtered more.
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
}
void resetVanishPointKF(KalmanFilter &KF,Size S)
{
    KF.statePre.at<float>(0) = S.width/2;
    KF.statePre.at<float>(1) = S.height/2;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
}

void LineDetect::iniLaneMarkKF(KalmanFilter &KF,Size S)
{
    Mat_<float> state(2, 1); /* (x, y, Vx, Vy) */    
    Mat processNoise(2, 1, CV_32F);    


    KF.statePre.at<float>(0) = 0;
    KF.statePre.at<float>(1) = S.width;

    KF.transitionMatrix = *(Mat_<float>(2, 2) << 1,0, 0,1);

    setIdentity(KF.measurementMatrix);
    //i f the value of Q is high, it will mean that the signal you are measuring is varies quickly
    //and you need the filter to be adaptable. 
    //If it is small, then big variations will be attributed to noise in the measure.
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    //If the value of R is high (compared to Q), it will indicate that the measuring is noisy so it will be filtered more.
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-3));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
}

void resetLaneMarkKF(KalmanFilter &KF,Size S)
{

    KF.statePre.at<float>(0) = 0;
    KF.statePre.at<float>(1) = S.width;    
}

void sweep(Mat in,Point center, double *bottom) 
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // fix the vanish point location.
    //center = Point(502,92);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int w = in.cols;
    int h = in.rows;

    //Bottom line:
    for(int i=0; i< 3*w ; i++) 
    {
        Point BP(i-w,h-1) ;
        cv::LineIterator it(in, center, BP, 8,0);

        bottom[i]=0;        
        //////////////////////////////////////////////////////
        for(int j = 0; j < it.count; j++, ++it)
        {
            bottom[i] += in.at<uchar>(it.pos());        
        }
        ////////////////////////////////////////////////////////
        // average
        bottom[i]/=it.count;                
    }
}

void laneDetect(Mat in,Point center,Mat predidict, double *bottom,Point &left, Point &right)
{
    int w = in.cols;
    int h = in.rows;

    int x0 = (int) predidict.at<float>(0);
    int x1 = (int) predidict.at<float>(1);

    //if (abs(x0 - x1) > 3*w/5)
    if (0)
    {
        //circle(in,center,1,CV_RGB(255, 0, 255),10);

        //bottom distribution
        for(int i=0; i< 3*w ; i++) 
        {
            Point a = cvPoint(i-w, h-bottom[i]-1);
            Point b = cvPoint(i-w, h-1);
            line(in, a, b, YELLOW, 1, CV_AA);
        }    

        int leftPos = 0, rightPos =0;    
        int range =  w/4;

        double maxValue = 0;
        for(int i=max(0,w + x0 -range); i<  min(w + x0 + range,3*w ) ; i++)
        {
            if( maxValue < bottom[i])
            {
                maxValue = bottom[i];
                leftPos  = (i-w);
            }        
        }

        maxValue = 0;
        for(int i=max(0,w + x1 -range); i< min(w + x1 + range,3*w ); i++)
        {
            if( maxValue < bottom[i])
            {
                maxValue = bottom[i];
                rightPos  = (i-w);
            }        
        }

        left = Point(leftPos, h-1);
        right = Point(rightPos, h-1);    
    }
    else
    {
        //circle(in,center,1,CV_RGB(255, 0, 255),10);

        //bottom distribution
        for(int i=0; i< w ; i++) 
        {
            Point a = cvPoint(i, h-bottom[i]-1);
            Point b = cvPoint(i, h-1);
            line(in, a, b, YELLOW, 1, CV_AA);
        }    
        
        for(int i=w; i< 2*w ; i++) 
        {
            Point a = cvPoint(i-w, h-bottom[i]-1);
            Point b = cvPoint(i-w, h-1);
            line(in, a, b, RED, 1, CV_AA);
        }    

        for(int i=2*w; i< 3*w ; i++) 
        {
            Point a = cvPoint(i-2*w, h-bottom[i]-1);
            Point b = cvPoint(i-2*w, h-1);
            line(in, a, b, GREEN, 1, CV_AA);
        }    
        
        int leftPos = 0, rightPos =0;    
        int range =  w/8;

        // left side
        double maxValue = 0;
        for(int i=0; i< center.x+w ; i++)
        {
            if( maxValue < bottom[i])
            {
                maxValue = bottom[i];
                leftPos  = (i-w);
            }        
        }

        // right side
        maxValue = 0;
        for(int i=center.x+w; i< 3*w ; i++)
        {
            if( maxValue < bottom[i])
            {
                maxValue = bottom[i];
                rightPos  = (i-w);
            }        
        }

        left = Point(leftPos, h-1);
        right = Point(rightPos, h-1);    
    }
}

void voteDraw(Mat in,Point center, double *bottom,Point &left, Point &right)
{
    int w = in.cols;
    int h = in.rows;

    //circle(in,center,1,CV_RGB(255, 0, 255),10);

    //bottom
    for(int i=0; i< 3*w ; i++) 
    {
        Point a = cvPoint(i-w, h-bottom[i]-1);
        Point b = cvPoint(i-w, h-1);
        //line(in, a, b, color, 1, CV_AA);
    }    

    int leftPos = 0, rightPos = 0;

    double maxValue = 0;
    for(int i=0; i< center.x+w ; i++)
    {
        if( maxValue <= bottom[i])
        {
            maxValue = bottom[i];
            leftPos  = (i-w);
        }        
    }

    maxValue = 0;
    for(int i=center.x+w; i< 3*w ; i++)
    {
        if( maxValue <= bottom[i])
        {
            maxValue = bottom[i];
            rightPos  = (i-w);
        }        
    }

    left = Point(leftPos, h-1);
    right = Point(rightPos, h-1);    
}

int indenseDraw(Mat in,Point center, double *bottom,Point &left, Point &right)
{
    int w = in.cols;
    int h = in.rows;
    //circle(in,center,1,CV_RGB(255, 0, 255),10);

    //bottom distribution
    for(int i=0; i< w ; i++) 
    {
        Point a = cvPoint(i, h-bottom[i]-1);
        Point b = cvPoint(i, h-1);
        line(in, a, b, YELLOW, 1, CV_AA);
    }    

    for(int i=w; i< 2*w ; i++) 
    {
        Point a = cvPoint(i-w, h-bottom[i]-1);
        Point b = cvPoint(i-w, h-1);
        line(in, a, b, RED, 1, CV_AA);
    }    

    for(int i=2*w; i< 3*w ; i++) 
    {
        Point a = cvPoint(i-2*w, h-bottom[i]-1);
        Point b = cvPoint(i-2*w, h-1);
        line(in, a, b, GREEN, 1, CV_AA);
    }    

    // first lane
    double average[3] = {0,0,0};
    for(int i=0; i< (left.x+w) ; i++) 
    {
        average[0] += bottom[i];
    }    
    average[0]/=(left.x+w);
    //second
    for(int i=left.x+w; i< right.x+w ; i++) 
    {
        average[1] += bottom[i];
    }    
    average[1]/=(right.x-left.x);
    //third
    for(int i=right.x+w; i< 3*w-1 ; i++) 
    {
        average[2] += bottom[i];
    }    
    average[2]/=(2*w-right.x);

    int flag = 0;
    if (average[2] > average[1])
        flag = 0;
    else if (average[1] > average[0])
        flag = 1;

    //cout<<"average ="<< average[0] <<" ; "<< average[1]<<";"<< average[2]<<endl;
    
    return (flag);
}

int findNewVP (Mat src,Mat Image,Mat Image2, Point &VP)
{
    //step1:  Hough line detection
    vector<Vec4i> lines;
    HoughLinesP( src, lines, 1, CV_PI/45, 20, 20, 1 );

    for( size_t i = 0; i < lines.size(); i++ )
    {
        Point A =  Point(lines[i][0], lines[i][1]);
        Point B =  Point(lines[i][2], lines[i][3]);

        line( Image,A,B, Scalar(255,0,0), 1, 8 );
        line( Image2,A,B, Scalar(255,0,0), 1, 8 );
    }

    //get center of the Image
    Point Center = Point(Image.cols/2,Image.rows/2);

    //Step2: Group the lines into two group, left and right.
    // condition 1: on the lower part
    // condition 2: on the left side K > 0
    // condition 3: on the right side K < 0;
    vector<Vec4i> leftLines;
    vector<Vec4i> rightLines;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        double a1 = lines[i][0];
        double b1 = lines[i][1];
        double a2 = lines[i][2];
        double b2 = lines[i][3];

        double K = (b1-b2)/(a2-a1);

        if ((a1 < Center.x)&&(b1 > Center.y)&& (K > 0.2))
        {
            leftLines.push_back(lines[i]);            
        }
        if ((a1 > Center.x)&&(b1 > Center.y)&&(K < -0.2))
        {
            rightLines.push_back(lines[i]);            
        }
    }

    int leftSize = leftLines.size();
    int rightSize = rightLines.size();

    //Step3: find all the intersectioin points which will be the canidates of the real Vanish Point.
    vector<Point> vanishPoint;
    for(int i =0; i< leftSize; i++)
    {
        double x1 = leftLines[i][0];
        double y1 = leftLines[i][1];
        double x2 = leftLines[i][2];
        double y2 = leftLines[i][3];

        for(int j =0; j< rightSize; j++)
        {
            double x3 = rightLines[j][0];
            double y3 = rightLines[j][1];
            double x4 = rightLines[j][2];
            double y4 = rightLines[j][3];
            // compute the intersection
            double denominator = (x1 - x2)*(y3-y4) - (y1-y2)*(x3-x4);
            double x = (x1*y2 - y1*x2)*(x3-x4) - (x1 - x2)*(x3*y4 - y3*x4);
            x = x/ denominator + 0.5;

            double y = (x1*y2 - y1*x2)*(y3-y4) - (y1 - y2)*(x3*y4 - y3*x4);
            y = y/ denominator + 0.5;

            // add some boundary of the vanish point to make it robust to error.
            if (sqrt((x-Center.x)*(x-Center.x)+ (y-Center.y)*(y-Center.y)) < Image.cols/4)
            {
                vanishPoint.push_back(Point(x,y));
            }            
        }
    }

    int VPsize = vanishPoint.size();

    //Step4: find the best vanish point which will have the minimal error area calculated as belows.
    vector<double> error;
    for(int k =0; k< VPsize; k++)
    {
        double area = 0;
        for(int i =0; i< leftSize; i++)
        {
            double x1 = leftLines[i][0];
            double y1 = leftLines[i][1];
            double x2 = leftLines[i][2];
            double y2 = leftLines[i][3];

            double A = x2 - x1;
            double B = y1 - y2;
            double C = y1*y2 - y1*y1 - x1*x2 + x1*x1;

            // distance between point to line((x1,y1),(x2,y2)).
            double distance = abs(A *vanishPoint[k].x + B*vanishPoint[k].y + C)/sqrt(A*A + B*B);    
            area += distance * sqrt(B*B+A*A);
        }

        for(int i =0; i< rightSize; i++)
        {
            double x1 = rightLines[i][0];
            double y1 = rightLines[i][1];
            double x2 = rightLines[i][2];
            double y2 = rightLines[i][3];

            double A = x2 - x1;
            double B = y1 - y2;
            double C = y1*y2 - y1*y1 - x1*x2 + x1*x1;

            double distance = abs(A *vanishPoint[k].x + B*vanishPoint[k].y + C)/sqrt(A*A + B*B);    
            area += distance * sqrt(B*B+A*A);
        }
        error.push_back(area);
    }

    double minValue=10e10, minIndex=-1;
    for(int k =0; k< VPsize; k++)
    {
        if(error[k] < minValue)
        {
            minValue = error[k];
            minIndex = k;
        }
    }    

    int flag = 0;
    if (VPsize > 0)
    {
        circle(Image, vanishPoint[minIndex], 4,Scalar(0,255,0),1,8, 0);
        VP = vanishPoint[minIndex];
        flag = 1;
    }    
    return (flag);
}

void ridgeDetect(Mat image_f,Mat &Kapa)
{
    Mat image_f2;
    //Step1: Gaussian filter
    double sigma = 1;
    int ksize = ((sigma - 0.8)/0.3 +1)*2 + 1; 
    Mat gaussKernelx =  getGaussianKernel(ksize, sigma); 
    sepFilter2D(image_f, image_f2, CV_32F, gaussKernelx , gaussKernelx);

    //Step2: Compute the Gradient Vector Field
    Mat KernelX = (Mat_<float>(1, 3) << -0.5, 0, 0.5);
    Mat KernelY = (Mat_<float>(3, 1) << -0.5, 0, 0.5);

    Mat fx,fy;
    filter2D(image_f2,fx,-1,KernelX);
    filter2D(image_f2,fy,-1,KernelY);

    //Step3: 
    Mat xx,xy,yy;
    multiply(fx, fx, xx, 1./255);
    multiply(fx, fy, xy, 1./255);
    multiply(fy, fy, yy, 1./255);

    //Step4: Gaussian filter
    {
        double sigma = 1;
        int ksize = ((sigma - 0.8)/0.3 +1)*2 + 1; 
        Mat gaussKernelx =  getGaussianKernel( ksize, sigma); 

        sepFilter2D(xx, xx, CV_32F, gaussKernelx , gaussKernelx);
        sepFilter2D(xy, xy, CV_32F, gaussKernelx , gaussKernelx);
        sepFilter2D(yy, yy, CV_32F, gaussKernelx , gaussKernelx);
    }

    //Step5:eigen value and eigen vector.
    int w = image_f.cols;
    int h = image_f.rows;

    Mat u(h, w, CV_32F);
    Mat v(h, w, CV_32F);

    {
        for (int i = 0; i< w; i++)
        {
            for (int j = 0; j< h; j++)
            {
                float a = xx.at<float>(j,i);
                float b = xy.at<float>(j,i);
                float c = b;
                float d = yy.at<float>(j,i);

                float T = a+d;
                float D = a*d - c*b;
                float L1 = T/2.0 + sqrt(T*T/4 - D);

                float ex = L1-d;
                float ey = c;
                float sign ;

                if (ex*fx.at<float>(j,i) + ey*fy.at<float>(j,i)> 0)
                    sign = 1;
                else
                    sign = -1;

                u.at<float>(j,i) = sign *ex;
                v.at<float>(j,i) = sign *ey;
            }
        }
    }

    //Step6: -DIV(X).    
    {
        Mat fu,fv;
        filter2D(u,fu,-1,KernelX);
        filter2D(v,fv,-1,KernelY);

        add(fu, fv,Kapa);
        Kapa = Kapa*-1.0;
    }
}


int LineDetect::line_Detection(cv::Mat image, KalmanFilter &LaneMarkKF, KalmanFilter &KF, Mat_<float> &measurement, Size &S, Mat_<float> &measLandMark,
								int *lineIdx, double *leftW, double *rightW)
{
    ///////////////////////////////////////////////////////////////////////////////////////////
    //Step2: Read Video, Pre-Processing.
    if(image.empty())
    {
        return 0;
    }
    cvtColor(image, image, COLOR_RGB2GRAY);
            //double scaleSize = 1;
            //resize(image, image, Size(image.cols/scaleSize,image.rows/scaleSize));
            medianBlur(image,image,3);

    int w = image.cols;
    int h = image.rows;

    Mat image_f,Kapa;
    image.convertTo(image_f,CV_32F);        
    ///////////////////////////////////////////////////////////////////////////////////////////
    //Step3: Lane detection with ridge algorithm.
    ridgeDetect(image_f,Kapa);

    ///////////////////////////////////////////////////////////////////////////////////////////
    //Step4: Vanish Point and Lane Marker Estimation
    Point VanishPoint, KF_VP,left, right;
            Mat T;
            double bottom[10240];    
    Mat ColorImage;
            Mat ColorImage2;
    {
                
        Kapa.convertTo(Kapa,CV_8UC1,512); 

        // add thresholdd
        threshold( Kapa, T, 50, 255,0 );        
        //imshow("T",T);
        //Mat src = T(Rect(0,0,T.cols,3*T.rows/5));
        //src = Scalar::all(0);
        cvtColor(image, ColorImage, COLOR_GRAY2RGB);    
                cvtColor(image, ColorImage2, COLOR_GRAY2RGB);    

        // use Hough line to detect the new vanish Point location.

        Mat HoughLine = Mat::zeros(image.rows, image.cols, CV_8UC3);

        int flag = findNewVP (T,HoughLine,ColorImage,VanishPoint);                

        //imshow("HoughLine",HoughLine);                
        cvtColor(HoughLine, HoughLine, COLOR_RGB2GRAY);    
                    

        // if new vanish point is detected
        if (flag == 1) 
        {
            //Predict VP location.
            Mat prediction = KF.predict();
            Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

            //if prediciton is not reliable, reset the Kalman filter for VP.
            if (norm(predictPt - Point(w/2,h/2)) > 2*w/10)
            {
                predictPt = Point(w/2,h/2);
                resetVanishPointKF(KF,S);
                //cout<<"+++++++++++++++++++++++++++++++++++++++"<<endl;
            }

            measurement(0) = VanishPoint.x;
            measurement(1) = VanishPoint.y;

            Point measPt(measurement(0),measurement(1));

            Mat estimated = KF.correct(measurement);
            Point statePt(estimated.at<float>(0),estimated.at<float>(1));

            KF_VP = statePt;
            // new VP
            circle(ColorImage, VanishPoint,1,CV_RGB(0, 255, 0),10);
            // KF VP
            circle(ColorImage, KF_VP,1,CV_RGB(255, 0, 255),10);

            /////////////////////////////////////////////////////////////////////////
            //Kalman Filter for the lane marker, only estimate the x_left and x_right value of the lane.
            {
                Mat prediction = LaneMarkKF.predict();        

                // check the validility of the prediction.
                if (abs(prediction.at<float>(0) - prediction.at<float>(1)) < 7*w/10)
                {
                    resetLaneMarkKF(LaneMarkKF,S);

                    prediction.at<float>(0) = 0;
                    prediction.at<float>(1) = w;

                    //cout<<"============================="<<endl;
                }
                // sweep from the center to each points.
                        sweep(Kapa,KF_VP,bottom); 

                laneDetect(ColorImage,KF_VP,prediction,bottom, left,right);    

                measLandMark(0) = left.x;
                measLandMark(1) = right.x;

                Mat estimated = LaneMarkKF.correct(measLandMark);

                left.x =  estimated.at<float>(0);
                right.x = estimated.at<float>(1);                        
            }                
        }
        else if(1)
        {
            Mat prediction = KF.predict();
            Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

            if (norm(predictPt - Point(w/2,h/2)) > 2*w/10)
            {
                predictPt = Point(w/2,h/2);
                resetVanishPointKF(KF,S);
                //cout<<"+++++++++++++++++++++++++++++++++++++++"<<endl;
            }

            KF_VP = predictPt;
            circle(ColorImage, KF_VP,1,CV_RGB(255, 0, 255),10);

            // sweep from the center to each points.
                    sweep(Kapa,predictPt,bottom);     


            voteDraw(ColorImage,predictPt,bottom, left,right);        

            {
                Mat prediction = LaneMarkKF.predict();

                left.x  = prediction.at<float>(0);
                right.x = prediction.at<float>(1);
            }                    
        }
        line(ColorImage, KF_VP, left, YELLOW, 1, CV_AA);                    
        line(ColorImage, KF_VP, right, YELLOW, 1, CV_AA);
    }
    ///////////////////////////////////////////////////////////////////////////////////////////
    sweep(image,KF_VP,bottom);     
    //void indenseDraw(Mat in,Point center, double *bottom,Point &left, Point &right)
    int laneIndex = indenseDraw(ColorImage2,KF_VP,bottom, left,right);    
    //cout << "laneIndex=" <<laneIndex<<endl;
    //imshow("ColorImage2",ColorImage2);

	*lineIdx = laneIndex;

#if 0
    ///////////////////////////////////////////////////////////////////////////////////////////
    // left & right solid or dashed.
    if(0)
    {
        RNG rng(12345);
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        Mat edge;
        GaussianBlur(Kapa, Kapa, Size( 5, 5 ), 0, 0 );
        Canny(image,edge,10,200);
            
        /// Find contours
        findContours( edge, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        /// Draw contours
        Mat drawing = Mat::zeros( edge.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
        }

        /// Show in a window
        namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        imshow( "Contours", drawing );

        contours.clear(); // Error!!
        imshow("edge",edge);
    }
    if(0)
    {
        //KF_VP
        //left
        //right
        //T

        //step 1: calcalue Left K
        double K_left  = double((h-1) - KF_VP.y)/double(KF_VP.x-left.x);
        double K_right = double((h-1) - KF_VP.y)/double(KF_VP.x-right.x);

        vector<Vec4i> lines;
        HoughLinesP(T, lines, 2, CV_PI/90, 20, 20, 1 );


        //left side, longest line

        double leftLen = 0;
        double rightLen = 0;
        for( size_t i = 0; i < lines.size(); i++ )
        {
            Point A =  Point(lines[i][0], lines[i][1]);
            Point B =  Point(lines[i][2], lines[i][3]);

            double kk = (A.y - B.y)/double(B.x-A.x);
            double len = norm(A-B); 

            if (kk)
            {
                if ((K_left/kk > 0.8) && (K_left/kk <1.2))
                {

                    if (len > leftLen)
                        leftLen = len;
                }

                if ((K_right/kk > 0.8) && (K_right/kk <1.2))
                {                        
                    if (len > rightLen)
                        rightLen = len;
                }
            }
        }
        cout <<"left len = " << leftLen <<endl; 
        cout <<"right len = " << rightLen <<endl; 
    }
    ///////////////////////////////////////////////////////////////////////////////////////////
    //draw current running lane.
#if 1
    if(1)
    {
        Mat overlay; 
        overlay = ColorImage.clone();

        Point pts[1][3];
        pts[0][0] = KF_VP;  
        pts[0][1] = left;  
        pts[0][2] = right;              
        const Point* ppt[1] = { pts[0] }; 

        int npts = 3;  

        fillPoly(overlay, ppt, &npts, 1, Scalar(0, 0, 255));    


        //draw with Transparency 
        double opacity = 0.2;
        addWeighted(overlay, opacity, ColorImage, 1 - opacity, 0, ColorImage);        

        //////////////////////////////////////////////////////////////////////
        //draw center line
        double xx = KF_VP.x;
        double yy = h;
        {
            line(ColorImage, Point(xx-100,yy-30), Point(xx+100,yy-30), Scalar(0, 255, 255), 1, CV_AA);
            line(ColorImage, Point(xx-100,yy-30+5), Point(xx-100,yy-30-5), Scalar(0, 255, 255), 1, CV_AA);
            line(ColorImage, Point(xx+100,yy-30+5), Point(xx+100,yy-30-5), Scalar(0, 255, 255), 1, CV_AA);
        }
        double ww = abs(right.x-left.x);
        //draw triangle
        {
            // location of the current vehicle.
            double x1 = (xx-left.x)*200/ww + (xx - 100);

            Point pts[1][3] = { Point(x1,yy-30), Point(x1-4,yy-30-10), Point(x1+4,yy-30-10) };
            
            const Point* ppt[1] = {pts[0]};
        
            int npts[1] = {3};  
            fillPoly(ColorImage, ppt, npts, 1, Scalar(0, 255, 255));                
        }

        //put text                
        {
            //calcalute distance
            double roadWidth = 3.0*ww/500;

            double leftWidth = 3.0*(xx-left.x)/500;
            double rightWidth = 3.0*(right.x-xx)/500;

            char buffer[32];                    
            sprintf(buffer, "<<--%1.1fM-->>", roadWidth);
            CvFont font;    
            //cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.6, 0.6, 1, 1, CV_AA);    
            //putText(in, buffer, Point((right.middle.x + left.middle.x)/2-80,yy-10), &font, CV_RGB(255,255,255));

            putText(ColorImage, buffer, Point((right.x + left.x)/2-80,yy-10), FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(255,255,255), 1);
            sprintf(buffer, "%1.1fM", leftWidth);
            putText(ColorImage, buffer, Point(xx-80,yy-35), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,255,255), 1);
            sprintf(buffer, "%1.1fM", rightWidth);
            putText(ColorImage, buffer, Point(xx+30,yy-35), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,255,255), 1);

			*leftW  = leftWidth;
			*rightW = rightWidth;
        }
    }
#endif
    ///////////////////////////////////////////////////////////////////////////////////////////
    //Step5: Generate BirdView    
    Mat birds_image;
    Mat H;
    {
        Mat temp_frame = image_f.clone();

        Point center = KF_VP;
        //Point center = Point(300,232);
        Point A = Point(0,image.rows-1);
        Point B = Point(image.cols-1 ,image.rows-1);

        Point2f objPts[4], imgPts[4];

        imgPts[0].x=(center.x+A.x)/2;
        imgPts[0].y=(center.y+A.y)/2;

        imgPts[1].x= (center.x+B.x)/2;
        imgPts[1].y= (center.y+B.y)/2;

        imgPts[2].x= A.x;
        imgPts[2].y= A.y;

        imgPts[3].x= B.x;
        imgPts[3].y= B.y;

        double scale = 1;
        double xoff = image.cols*scale/2;
        double hh = scale*(A.y -imgPts[0].y)/1;
        objPts[0].x = scale*A.x/4+xoff;
        objPts[0].y = A.y*scale-hh;

        objPts[1].x = scale*B.x/4+xoff;
        objPts[1].y = A.y*scale-hh;

        objPts[2].x = scale*A.x/4+xoff;
        objPts[2].y = scale*A.y;

        objPts[3].x = scale*B.x/4+xoff;
        objPts[3].y = scale*B.y;

        line(image_f, cvPoint(imgPts[0].x,imgPts[0].y), cvPoint(imgPts[1].x,imgPts[1].y), Scalar(255, 0, 0), 1, CV_AA);

        H = getPerspectiveTransform( objPts, imgPts);            

        Mat homography = H;

        Size birdSize = Size(image_f.cols*scale,image_f.rows*scale);
        warpPerspective(
            ColorImage,
            birds_image,
            H,
            birdSize,
            INTER_LINEAR  | WARP_INVERSE_MAP
            );
        Mat RImage;

        warpPerspective(
            birds_image,
            RImage,
            H,
            birdSize,
            INTER_NEAREST
            );

        //outputVideo << birds_image;                
    }

    imshow("ColorImage",ColorImage);
    imshow( "Birds_Eye", birds_image );    
    imshow("Kapa",Kapa);

#endif
    //waitKey(1);
}
