#include "polynomialFitting.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "utils.h"
#include "AppInitCommon.h"

using namespace cv;
using namespace std;

namespace ns_roadScan
{
#define R  6378137

#ifdef DEBUG_ROAD_SCAN
	int landmarkSquence = 1;
#endif

#define COEFF_DD2METER (111320.0)
#define PI (3.14159265359)

 /*
 * @FUNC
 *     Source GPS to relative GPS according to ref GPS, lon and lat to x,y. unit: m;     
 *
 * @PARAMS
 *     in    -  src GPS,(lon ,lat);
 *     ref   -  the reference GPS point;
 *     out   -  relative GPS,(x,y),unit: m.
 *
 */
void coordinateChange(Point2d in,Point2d ref,Point2d &out)
{
	double dif_x = in.x - ref.x;
	double dif_y = in.y - ref.y;
	double latitude = (ref.x)*PI/180;

	out.x = dif_x*COEFF_DD2METER;  //latitude
	out.y = dif_y*(111413*cos(latitude)-94*cos(3*latitude));  //longitude

}
/*
 * @FUNC
 *     fft transform;     
 *
 * @PARAMS
 *     src        -  src image;
 *     dst        -  out image;
 *     complexI   -  out image.
 *
 */
void fft2(Mat &src, Mat &dst,Mat &complexI)  
{    
    Mat image_Re;
	Mat image_Im;
	// Real part conversion from u8 to 64f (double)  
	src.convertTo(image_Re,CV_64F);
	Mat planes[] = {Mat_<double>(image_Re), Mat::zeros(src.size(), CV_64F) };
    merge(planes, 2, complexI);
    
	dft(complexI, complexI);   
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
/*
 * @FUNC
 *     ifft transform;     
 *
 * @PARAMS
 *     src        -  src image;
 *     dst        -  out image;
 *
 */
void ifft2(Mat &src, Mat &dst)  
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
/*
 * @FUNC
 *     use fft transform to eliminate periodic noise;     
 *
 * @PARAMS
 *     src        -  src image;
 *     dst        -  out image;
 *
 */
void processNoise(Mat &src, Mat &dst)
{
	// fft process
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
/*
 * @FUNC
 *     get GPS location of every pixel in roadScanImage;     
 *
 * @PARAMS
 *     GPS1   -  current frame GPS in each interval;
 *     GPS1   -  next frame GPS in each interval;
 *     pixel  -  coordinate(x,y),in image,x:col;y:row;
 *     scopeOfScanImage -  hight of frame to cut 
 *     distancePerPixel -  distance(unit m) per pixel;
 *
 */
void getGPSLocationOfEveryPixelInRoadScanImage(Point2d GPS1,Point2d GPS2,Point2d pixel,
    int scopeOfScanImage,Point2d &GPSFinal, double distancePerPixel)
{
	if (scopeOfScanImage < 1)
	{
		cout<<"error : scope of scan image empty."<<endl;
	}

	// update for algorithm 2015.09.14
	double angle = atan2(GPS2.y-GPS1.y, GPS2.x-GPS1.x);
	double t = PI*1.5 + angle;

	// coordinate system 2
	double x2 = -1.0 * pixel.x * distancePerPixel / 100;
	double y2 = pixel.y * distancePerPixel / 100;

	// coordinate system 1
	GPSFinal.x = x2 * cos(t) - y2 * sin(t) + GPS1.x;
	GPSFinal.y = x2 * sin(t) + y2 * cos(t) + GPS1.y;
}

/*
 * @FUNC
 *     cal ridge image for roadScan image, to extract white painting ;     
 *
 * @PARAMS
 *     image_f   -  input image;
 *     Kapa      -  output ridge image;
 *     sigma1    -  Gaussian param;
 *     sigma2    -  Gaussian param 
 *
 */
void ridgeDetect(Mat &image_f,Mat &Kapa, double sigma1, double sigma2)
{
	Mat image_f2;

	//Step1: Gaussian filter
	double sigma = sigma1;
	int ksize = ((sigma - 0.8)/0.3 +1)*2 + 1; 
	Mat gaussKernelx =  getGaussianKernel(ksize, sigma); 
	sepFilter2D(image_f, image_f2, CV_32F, gaussKernelx , gaussKernelx);

	//Step2: Compute the Gradient Vector Field
	Mat KernelX = (Mat_<float>(1, 3) << -0.5, 0, 0.5);
	Mat KernelY = (Mat_<float>(3, 1) << -0.1, 0, 0.1);

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
		double sigma = sigma2;
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
				float L1 = T/2.0 + sqrt(T*T/4.0 - D);

				float ex = L1-d;
				float ey = c;
				float norm = sqrt(ex*ex + ey*ey);

				ex = L1*ex/norm;
				ey = L1*ey/norm;

				float sign ;

				if ((ex*fx.at<float>(j,i) + ey*fy.at<float>(j,i))> 0)
					sign = +1;
				else
					sign = -1;

				u.at<float>(j,i) = sign *ex;
				v.at<float>(j,i) = sign *ey;
			}
		}
	}
	{
		Mat fu,fv;
		filter2D(u,fu,-1,KernelX);
		filter2D(v,fv,-1,KernelY);

		add(fu, fv,Kapa);
		Kapa = Kapa*-1.0;
	}
}
/*
 * @FUNC
 *     get information of everyLine, cal left,middle, and right relGPSs ;     
 *
 * @PARAMS
 *     GPSAndInterval   -  include current and next frame GPS, the hight of each interval to cut;
 *     roadDraw         -  image, the painting lines has being linked;
 *     longLane         -  source roadScan image.
 *     lineNum          -  position in image 
 *     startPoint       -  interval start point
 *     inParam          -  config info for each video
 *     rowData          -  output painting and linkline relGPS; 
 */
void getInformationOfEveryLine(gpsInformationAndInterval &GPSAndInterval,Mat &roadDraw,
    Mat &longLane,int lineNum,int startPoint,dataEveryRow &rowData, Parameters inParam)
{
	//left and right paint
	int centerOfSertch = roadDraw.cols * 0.5 - 1;

	int number = roadDraw.rows - lineNum - 1;

	if(rowData.leftPoint.x==-1)
	{
		//no paint - need to find dotted point
		for(int i=centerOfSertch;i>=0;i--)
		{
			if((roadDraw.at<Vec3b>(number,i)[0] > 200) &&
				(roadDraw.at<Vec3b>(number,i)[1] > 200) &&
				(roadDraw.at<Vec3b>(number,i)[2] > 200))
			{
				rowData.leftPoint.x = i;
				rowData.leftPoint.y = number;
				rowData.isPaint_Left = 0;
				break;
			}
		}
	}

	if(rowData.rightPoint.x==-1)
	{
		//no paint - need to find dotted point
		for(int i=centerOfSertch;i<roadDraw.cols;i++)
		{
			if((roadDraw.at<Vec3b>(number,i)[0] > 200) &&
				(roadDraw.at<Vec3b>(number,i)[1] > 200) &&
				(roadDraw.at<Vec3b>(number,i)[2] > 200))
			{
				rowData.rightPoint.x = i;
				rowData.rightPoint.y = number;
				rowData.isPaint_Right = 0;
				break;
			}
		}
	}

	//Point2d ref = Point2d(51.447148732952009,-2.499363139099061);//latStand and lonStand

	Point2d GPS_1,GPS_2;
	coordinateChange(GPSAndInterval.GPS_now,inParam.GPSref,GPS_1);
	coordinateChange(GPSAndInterval.GPS_next,inParam.GPSref,GPS_2);
	// calculate offset of GPS
	resetGPSOffset(inParam.offsetDist, GPS_1, GPS_2);

    //middle
    Point2d Pixel = Point2d(0.0, lineNum-startPoint);
    getGPSLocationOfEveryPixelInRoadScanImage(GPS_1,GPS_2,Pixel,GPSAndInterval.intervalOfInterception,rowData.Middle_RelGPS,inParam.distancePerPixel);

	//left
	if(rowData.leftPoint.x!=-1)
	{
		Point2d Pixel = Point2d(rowData.leftPoint.x-centerOfSertch, lineNum-startPoint);
		getGPSLocationOfEveryPixelInRoadScanImage(GPS_1,GPS_2,Pixel,GPSAndInterval.intervalOfInterception,rowData.Left_Middle_RelGPS,inParam.distancePerPixel);

		if(rowData.left_Edge_XY[0].x != -1)
		{
			Pixel = Point2d(rowData.left_Edge_XY[0].x-centerOfSertch, lineNum-startPoint);
			getGPSLocationOfEveryPixelInRoadScanImage(GPS_1, GPS_2, Pixel, GPSAndInterval.intervalOfInterception, rowData.Left_Paint_Edge[0], inParam.distancePerPixel);
		}

		if(rowData.left_Edge_XY[1].x != -1)
		{
			Pixel = Point2d(rowData.left_Edge_XY[1].x-centerOfSertch, lineNum-startPoint);
			getGPSLocationOfEveryPixelInRoadScanImage(GPS_1, GPS_2, Pixel, GPSAndInterval.intervalOfInterception, rowData.Left_Paint_Edge[1], inParam.distancePerPixel);
		}

	}
    else
    {
        rowData.Left_Middle_RelGPS = rowData.Middle_RelGPS;
    }

	

	//right
	if(rowData.rightPoint.x!=-1)
	{
		Point2d Pixel = Point2d(rowData.rightPoint.x-centerOfSertch, lineNum-startPoint);
		getGPSLocationOfEveryPixelInRoadScanImage(GPS_1,GPS_2,Pixel,GPSAndInterval.intervalOfInterception,rowData.Right_Middle_RelGPS,inParam.distancePerPixel);

		if(rowData.right_Edge_XY[0].x != -1)
		{
			Pixel = Point2d(rowData.right_Edge_XY[0].x-centerOfSertch, lineNum-startPoint);
			getGPSLocationOfEveryPixelInRoadScanImage(GPS_1, GPS_2, Pixel, GPSAndInterval.intervalOfInterception, rowData.Right_Paint_Edge[0], inParam.distancePerPixel);
		}

		if(rowData.right_Edge_XY[1].x != -1)
		{
			Pixel = Point2d(rowData.right_Edge_XY[1].x-centerOfSertch, lineNum-startPoint);
			getGPSLocationOfEveryPixelInRoadScanImage(GPS_1, GPS_2, Pixel, GPSAndInterval.intervalOfInterception, rowData.Right_Paint_Edge[1], inParam.distancePerPixel);
		}

	}
    else
    {
        rowData.Right_Middle_RelGPS = rowData.Middle_RelGPS;
    }

	//brightness
	int interval = 20;

	double totalValue;

	//have left paint / dont have right paint
	if((rowData.leftPoint.x!=-1)&&(rowData.rightPoint.x==-1))
	{
		if(rowData.leftPoint.x>interval)
		{
			totalValue = 0.0;
			for(int i=0;i<(rowData.leftPoint.x-interval);i++)
			{
				totalValue += longLane.at<uchar>(number,i);
			}
			rowData.Left_Area_Pixel_Mean = totalValue/(rowData.leftPoint.x-interval);
		}

		totalValue = 0.0;
		for(int i=(rowData.leftPoint.x+interval+1);i<longLane.cols;i++)
		{
			totalValue += longLane.at<uchar>(number,i);
		}
		rowData.Middle_Area_Pixel_Mean = totalValue/(longLane.cols-rowData.leftPoint.x-interval-1);
	}

	//dont have left paint / have right paint
	if((rowData.leftPoint.x==-1)&&(rowData.rightPoint.x!=-1))
	{
		if((rowData.rightPoint.x+interval+1)<longLane.cols)
		{
			totalValue = 0.0;
			for(int i=(rowData.rightPoint.x+interval+1);i<longLane.cols;i++)
			{
				totalValue += longLane.at<uchar>(number,i);
			}
			rowData.Right_Area_Pixel_Mean = totalValue/(longLane.cols-rowData.rightPoint.x-interval-1);
		}

		totalValue = 0.0;
		for(int i=0;i<(rowData.rightPoint.x-interval);i++)
		{
			totalValue += longLane.at<uchar>(number,i);
		}
		rowData.Middle_Area_Pixel_Mean = totalValue/(rowData.rightPoint.x-interval);
	}

	//dont have left and right paint
	if((rowData.leftPoint.x==-1)&&(rowData.rightPoint.x==-1))
	{
		totalValue = 0.0;
		for(int i=0;i<longLane.cols;i++)
		{
			totalValue += longLane.at<uchar>(number,i);
		}
		rowData.Middle_Area_Pixel_Mean = totalValue/longLane.cols;
	}

	//have left and right paint
	if((rowData.leftPoint.x!=-1)&&(rowData.rightPoint.x!=-1))
	{
		if(rowData.leftPoint.x>interval)
		{
			totalValue = 0.0;
			for(int i=0;i<(rowData.leftPoint.x-interval);i++)
			{
				totalValue += longLane.at<uchar>(number,i);
			}
			rowData.Left_Area_Pixel_Mean = totalValue/(rowData.leftPoint.x-interval);
		}

		if((rowData.rightPoint.x-rowData.leftPoint.x)>(2*interval+1))
		{
			totalValue = 0.0;
			for(int i=(rowData.leftPoint.x+interval+1);i<(rowData.rightPoint.x-interval);i++)
			{
				totalValue += longLane.at<uchar>(number,i);
			}
			rowData.Middle_Area_Pixel_Mean = totalValue/(rowData.rightPoint.x-rowData.leftPoint.x-2*interval-1);
		}

		if((rowData.rightPoint.x+interval+1)<longLane.cols)
		{
			totalValue = 0.0;
			for(int i=(rowData.rightPoint.x+interval+1);i<longLane.cols;i++)
			{
				totalValue += longLane.at<uchar>(number,i);
			}
			rowData.Right_Area_Pixel_Mean = totalValue/(longLane.cols-rowData.rightPoint.x-interval-1);
		}
	}
}
/*
 * @FUNC
 *     cal threshold adaptive, based on histpgram method;     
 *
 * @PARAMS
 *     Inimg   -  input image;
 *     hisTH   -  output threshold;
 *     dstImage - output histogram image;
 */
void calThread(Mat &Inimg, int &hisTH, Mat&dstImage)
{
	MatND dstHist;
	int dims=1;
	float hranges[] = {0,255};
	const float *rangs[] = {hranges};
	int channels = 0;
	int size =256;
	calcHist(&Inimg,1,&channels,Mat(),dstHist,dims,&size,rangs);
	int scale=1;

	double minValue = 0;
	double maxValue = 0;
	minMaxLoc(dstHist,&minValue,&maxValue,0,0);

	int hpt = saturate_cast<int>(0.9 * size);
	for (int i=0;i<256;i++)
	{
		float binValue = dstHist.at<float>(i);
		int realValue = saturate_cast<int>(binValue * hpt/maxValue);
		rectangle(dstImage,Point(i*scale,size-1),Point((i+1)*scale-1,size-realValue),Scalar(255));
	}

	int sum = 0;
	for (int i=0;i<256;i++)
	{
		float binValue = dstHist.at<float>(i);
		sum +=binValue;
	}

	double widthPer =0;
	double numWidthPlus=0;
	for (int i=0;i<256;i++)
	{
		float binValue = dstHist.at<float>(i);
		numWidthPlus +=binValue;
		widthPer = numWidthPlus/(sum);
		if (widthPer>0.9)
		{
			//			cout<<i<<endl;
			hisTH = i;//=i for Ford
			break;
		}
	}

}
/*
 * @FUNC
 *     cal ridge params adaptive, these are based on the painting width;     
 *     returen painting width as params; 
 * @PARAMS
 *     Inimg   -  input BW image,after threshold;
 */
int calRidgePar(Mat &Inimg)
{
	uchar *dataImg = (uchar*)Inimg.data;
	int ridgePra=0;
	int s[100]={0};
	int numWidth=0;
	int numSupperWidth = 0;
	int h = Inimg.rows;
	int w = Inimg.cols;
	for (int i = h-1; i>0;i--)
	{
		int maxWidth = 0;
		
		Mat rowHist(256,w,CV_8UC1,Scalar(0));
		vector<int> paintWidth;
		for(int j = 0;j<w;j++)
		{
			int binvalue = dataImg[i*w+j];
			if (binvalue<5)
			{
				binvalue = 0;
			}
			rectangle(rowHist,Point(j,255),Point(j,255-binvalue),Scalar(255));								
		}
		//		imwrite("rowHist.png",rowHist);
		int posStart=0;
		int posEnd = 0;
		int count =0;
		
		for(int j1=0;j1<w;j1++)
		{
			int nowValue = rowHist.at<uchar>(254,j1);
			if (nowValue==255&&count == 0)
			{
				posStart = j1;
				count = 1;

			}
			if (nowValue==0&&count ==1)
			{
				posEnd = j1;
				count = 0;
				int lineWidth = posEnd-posStart;
				paintWidth.push_back(lineWidth);
			}
		}
		if (paintWidth.size())
		{
			for (int i2=0;i2<paintWidth.size();i2++)
			{
			//	if (maxWidth<=paintWidth[i2])
				{
					maxWidth = paintWidth[i2];
				}

				if (maxWidth<30&&maxWidth>4)
				{
					s[maxWidth]+=1;
					numWidth++;
				}
				if (maxWidth>30&&maxWidth<70)
				{
					numSupperWidth++;
				}
			}

		}
		i -= 20;
	}

	Mat rowHist2(256,100,CV_8UC1,Scalar(0));
	int minValue3=0;
	int maxValue3=0;
	for (int i=0;i<100;i++)
	{
		if (minValue3>s[i])
		{
			minValue3 = s[i];
		}
		if (maxValue3<s[i])
		{
			maxValue3 = s[i];
		}
	}

	int hpt2 = 0.9 * 256;
	double widthPer =0;
	double numWidthPlus=0;
	for (int i=0;i<100;i++)
	{
		float binValue = s[i];
		numWidthPlus +=binValue;
		widthPer = numWidthPlus/numWidth;
		
		if (Inimg.rows<500&&numSupperWidth>numWidth)
		{
			if (widthPer>0.80)
			{
				//	cout<<i<<endl;
				ridgePra = i;
				break;
			}
		}
		if (Inimg.rows<100&&numSupperWidth<=numWidth)
		{	
			if (widthPer>0.5)
			{
				//	cout<<i<<endl;
				ridgePra = i;
				break;
			}	
		}
		if (Inimg.rows>=100)
		{
			if (widthPer>0.80)
			{
				//	cout<<i<<endl;
				ridgePra = i;
				break;
			}

		}
		
		int realValue =binValue * hpt2/maxValue3;
		rectangle(rowHist2,Point(i,255),Point(i,255-realValue),Scalar(255));

	}
	//	imshow("rowHist2",rowHist2);
	//	waitKey(-1);

	return ridgePra;

}

void calculateGradient(Mat &src,Mat &dxImg,Mat &dyImg)
{
	Mat srcImg(src);
	if (src.channels() > 1)
	{
		cvtColor(srcImg,srcImg,COLOR_BGR2GRAY);
		//return false;
	}

	//double startT = static_cast<double>(cv::getTickCount());

	// step 1: caculate the digradient

	// set mask
	float mask[3][3]={{1,2,1},{0,0,0},{-1,-2,-1}};
	Mat y_mask=Mat(3,3,CV_32F,mask);
	Mat x_mask=y_mask.t();

	Mat sobelX,sobelY;
	filter2D(srcImg,sobelX,CV_32F,x_mask);
	filter2D(srcImg,sobelY,CV_32F,y_mask);

	Mat gradientNorm(src.rows,src.cols, CV_32F) ;
	Mat gradientSQ = sobelX.mul(sobelX) + sobelY.mul(sobelY);
	sqrt(gradientSQ,gradientNorm);

	//namedWindow("gradientN",1);
	//imshow("gradientN",gradientNorm/100);


	Scalar meanG = mean(gradientNorm);

	sobelX.copyTo(dxImg);
	sobelY.copyTo(dyImg);

	// set the gradient to zero if below the threshold
	for(int ii = 0; ii < gradientNorm.rows;ii++)
	{
		uchar* inDataRow = gradientNorm.data + ii*gradientNorm.step; 
		uchar* dxInRow = dxImg.data + ii*dxImg.step;
		uchar* dyInRow = dyImg.data + ii*dyImg.step;

		for(int jj = 0; jj < gradientNorm.cols;jj++)
		{
			float *inData = (float*)inDataRow + jj; 
			float *dxIn = (float*)dxInRow + jj;
			float *dyIn = (float*)dyInRow + jj;

			if ( *inData  < 4*meanG[0])
			{
				*dyIn = 0;
				*dxIn = 0;
			}
		}
	}

}
double searchFixedRadusShape(Mat &dxImg, Mat &dyImg,Mat &Sr,int radus,int nSide)
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


	float tanN = tan(PI/nSide);
	int w = cvRound(radus*tanN);

	int feat1 = QrAndBr.step / sizeof(float);

//	for(int rowIdx = START_ROW;rowIdx < END_ROW;rowIdx++)
	for(int rowIdx = 0;rowIdx < dxImg.rows;rowIdx++)
	{
		float* dxInRow = (float*)(dxImg.data + rowIdx*dxImg.step);
		float* dyInRow = (float*)(dyImg.data + rowIdx*dyImg.step);

		for(int colIdx = 0;colIdx < dxImg.cols;colIdx++)
		{
			if (colIdx==492)
			{
				int sss=1;
			}
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
				//	circle(src,Point(YYP,XXP),2,Scalar(0,0,255));
				//	circle(src,Point(YYN,XXN),2,Scalar(0,255,255));
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
							//	short* inDataRow = (short*)(Qr.data + detaWx*Qr.step) + detaWy;
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

	//for(int rowIdx = START_ROW;rowIdx < END_ROW;rowIdx++)
	for(int rowIdx = 0;rowIdx < dxImg.rows;rowIdx++)
	{
		float* dxInRow = (float*)(dxImg.data + rowIdx*dxImg.step);
		float* dyInRow = (float*)(dyImg.data + rowIdx*dyImg.step);

		for(int colIdx = 0;colIdx < dxImg.cols;colIdx++)
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
				float nThetaG = nSide*thetaG*PI_DIV_180;// n-angle
				float vpTheta = nThetaG;// angle for positive voting line
				float vnTheta = nThetaG + PI;// angle for negative voting line

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

	for(int rowIdx = 0;rowIdx < QrAndBr.rows;rowIdx++)
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
	return 0;
}

int searchRegularPolygon(Mat &src,Mat &dxImg, Mat &dyImg,vector<int> &radusVec,int nSide,TS_Structure &target,Mat &dstTemp)
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
//		imwrite("dst.png",dst);
	}
		//Mat dstTemp;
	dst.copyTo(dstTemp);
	


	// look up the 3 maximum value location
	vector<Point> maxLoc3;
	vector<double> valVec;
	unsigned int counter = 0;
	double valSum = 0;
	while(counter < MAX_NUM_SIGN_PER_IMG)
	{
		double minVal1,maxVal1;
		Point minValLoc1,maxValLoc1;
		
		minMaxLoc( dst, &minVal1, &maxVal1,&minValLoc1,&maxValLoc1);   
		
		if(maxVal1 == 0.0)
			return 0;
		dst.at<float>(maxValLoc1.y,maxValLoc1.x) = 0.0;

		if(counter == 2)
		{
			int dX1 = maxValLoc1.x - maxLoc3[1].x;
			int dY1 = maxValLoc1.y - maxLoc3[1].y;
			int dist1 = dX1*dX1 + dY1*dY1;

			int dX2 = maxValLoc1.x - maxLoc3[0].x;
			int dY2 = maxValLoc1.y - maxLoc3[0].y;
			int dist2 = dX2*dX2 + dY2*dY2;

			if((dist1> MIN_DISTANCE) && (dist2 > MIN_DISTANCE))
			{
				maxLoc3.push_back(maxValLoc1);
				valSum += maxVal1;
				valVec.push_back(maxVal1);
				counter++;
			}
		}
		else if(counter == 1)
		{
			int dX = maxValLoc1.x - maxLoc3[0].x;
			int dY = maxValLoc1.y - maxLoc3[0].y;
			if((dX*dX + dY*dY) > MIN_DISTANCE)
			{
				maxLoc3.push_back(maxValLoc1);
				valSum += maxVal1;
				valVec.push_back(maxVal1);
				counter++;
			}
		}
		else
		{
			valSum += maxVal1;
			valVec.push_back(maxVal1);
			maxLoc3.push_back(maxValLoc1);
			counter++;
		}
	}

	for(int index = 0;index < MAX_NUM_SIGN_PER_IMG;index++)
	{
		if( valVec[index] > THESHOLD*valSum)
		{
			float minDist = 90000.0;
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
			//	Point center(maxLoc[minIdx]);
				Point P1,P2;
				P1.x = (maxLoc3[index].x - realRadus) > 0 ? (maxLoc3[index].x - realRadus):0 ;
				P1.y = (maxLoc3[index].y - realRadus) > 0 ? (maxLoc3[index].y - realRadus):0;
				P2.x = (P1.x + 2*realRadus) >= src.cols ? src.cols-1 :(P1.x + 2*realRadus) ;
				P2.y = (P1.y + 2*realRadus) >= src.rows ? src.rows-1 : (P1.y + 2*realRadus);

				rectRoi.push_back(P1);
				rectRoi.push_back(P2);

				circle(src,P1,2,Scalar(0,255,255));
				circle(src,P2,2,Scalar(0,255,0));
				cv::Rect roi = boundingRect(rectRoi);		
				circle(src,maxLoc3[index],realRadus,Scalar(0,0,255));
                imwrite("src.png",src);

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
	imwrite("sdasda.png",src);
	//circle(src,maxValLoc2,2,Scalar(0,0,255));

	//threshold(dstTemp,dstTemp,120,255,THRESH_TOZERO);

	//namedWindow("output",1);
	//imshow("output",dstTemp);
	//waitKey(1);
	return 1;
}


void circleDection(Mat &Inimage,vector<landMark> &landmark,int histThres)
{

	Mat src;
	Mat roi = Inimage(Rect(0,0,Inimage.cols,Inimage.rows));
	roi.copyTo(src);
	Mat imgThres;
	threshold(Inimage,imgThres,histThres,255,0);
	//imwrite("imgThres.png",imgThres);
	Mat dxImg(src.rows,src.cols, CV_32F);
	Mat dyImg(src.rows,src.cols, CV_32F);
	calculateGradient(src,dxImg,dyImg);
//	imwrite("dxImg.png",dxImg);
//	imwrite("dyImg.png",dyImg);
	vector<int> radus;
	for(int kk = 35; kk < 150; kk += 4)
	{
		radus.push_back(kk);
	}

	TS_Structure targetDect;
	memset(&targetDect,0,sizeof(targetDect));

	Mat dst;
	searchRegularPolygon(src,dxImg,dyImg,radus,8,targetDect,dst);
	imwrite("dst.png",dst);
 	for (int i=0;i<MAX_NUM_SIGN_PER_IMG;i++)
	{
		double maxValue = 0;
		Point maxValuePoint = Point(0,0);
		double minValue = 0;
		Point minValuePoint = Point(0,0);
		minMaxLoc(dst,&minValue,&maxValue,&minValuePoint,&maxValuePoint);
	//	cout<<maxValue<<endl;
 		if (maxValue<50)
		{
			continue;
		}
		int w0 = 150;
		int h0 = 150;
		int x0 = maxValuePoint.x-75;
		int y0 = maxValuePoint.y-75;
		if (x0<0)
		{
			x0 = 0;
		}
		if (y0<0)
		{
			y0 = 0;
		}
		if (maxValuePoint.x+w0/2>=src.cols)
		{
			w0 =(w0/2+src.cols - maxValuePoint.x);
		}
		if (maxValuePoint.y+h0/2>=src.rows)
		{
			h0 = (h0/2+src.rows - maxValuePoint.y);
		}
		Mat roi = src(Rect(x0,y0,w0,h0)); 
		imwrite("roi.png",roi);	

		int numMatchTemp = 0;
		vector<int> matchIndex;
		for (int k=1;k<=102;k++)
		{
			char model[100];
			sprintf(model,"./circles/landmarkCircle_%d.png",k);
			Mat modelImg = imread(model);
			if (!modelImg.data)
			{
				break;
			}
			if (modelImg.cols/roi.cols<0.5||modelImg.rows/roi.rows<0.5||
				modelImg.cols/roi.cols>2||modelImg.rows/roi.rows>2)
			{
				continue;
			}
			int numMatch = SurfFeatureMatch(roi,modelImg);
			matchIndex.push_back(numMatch);
		}
		int index=0;
		for (int j=0;j<matchIndex.size();j++)
		{
			if (matchIndex[j]>=numMatchTemp)
			{
				numMatchTemp = matchIndex[j];
				index = j;
			}
		}	
		cout<<numMatchTemp<<endl;
		if (numMatchTemp>10)
		{			
			//imwrite("roi.png",roi);	
		}

 		Mat rect = dst(Rect(x0,y0,w0,h0));
		rect = Mat::zeros(h0,w0,CV_8UC1);

	}
 	imwrite("dst.png",dst);
	
}

/*
 * @FUNC
 *     detect landmarks, such as arrows;      
 * @PARAMS
 *     Inimg   -  input roadScan image;
 *     histThres  - BW threshold
 *     landmark - landmark information
 */
void landMarkDetection(Mat &Inimage,vector<landMark> &landmarks,int histThres)
{
	Mat imgThres;
	threshold(Inimage,imgThres,histThres,255,0);
	Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
	Mat Dilate,Erode;
	dilate(imgThres,Dilate,element,Point(-1,-1),2);
//	imshow("Dilate",Dilate);
	erode(Dilate,Erode,element,Point(-1,-1),1);

#ifdef DEBUG_ROAD_SCAN
	imwrite("Erode.png",Erode);
#endif
	

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(Dilate, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	// draw contours:   

	Mat T = Mat::zeros(Inimage.size(), CV_8UC1);
	Mat T2 = Mat::zeros(Inimage.size(), CV_8UC1);
	int w = Inimage.cols/4;
	int meanValue=0;
	for (unsigned int i = 0; i < contours.size(); i++)
	{			
		vector<Point> pointss = contours[i];
		Point upoint = pointss[0];
		Point dpoint = pointss[pointss.size()-1];
		Point lpoint = pointss[0];
		Point rpoint = pointss[pointss.size()-1];
		Point2d weightPoint = Point2d(-1,-1);
		for (int j=0;j<pointss.size();j++)
		{
			if (pointss[j].y<upoint.y)
			{
				upoint = pointss[j];
			}
			if (pointss[j].y>dpoint.y)
			{
				dpoint = pointss[j];
			}			

			if (pointss[j].x<lpoint.x)
			{
				lpoint = pointss[j];
			}
			if (pointss[j].x>rpoint.x)
			{
				rpoint = pointss[j];
			}
			weightPoint.x += pointss[j].x;
			weightPoint.y += pointss[j].y;

			meanValue += Inimage.at<uchar>(pointss[j].y,pointss[j].x);
		}
		weightPoint.x = weightPoint.x/pointss.size();
		weightPoint.y = weightPoint.y/pointss.size();
		meanValue = meanValue/contours.size();

		int dx = rpoint.x - lpoint.x;
		int dy = dpoint.y - upoint.y;
	
		double values = dy/(dx+0.0000000001);
		if (abs(values-1)<0.3)
		{
			continue;
		}

		if (meanValue<200)
		{
	//		continue;
		}
    	if (rpoint.x<w*3&&lpoint.x>w)
		{
			if (contourArea(contours[i])>2000&&dy<1000)
			{
				Mat roi = Inimage(Rect(lpoint.x,upoint.y,dx,dy));

				int result = arrowClassify(roi);

#ifdef ROAD_SCAN_UT
//				cout<<"result："<<result<<endl;
#endif							

				if (result !=-1&&result!=0)
				{
			//		cout<<"matchnum="<<numMatchTemp<<"model="<<index<<endl;					
					RotatedRect rect = minAreaRect(contours[i]);
					Point2f vertex[4];
					rect.points(vertex);
					for (int j =0;j<4;j++)
					{
						line(T,vertex[j],vertex[(j+1)%4],Scalar(255,0,0),2,8);
					}				
					drawContours(T, contours, i, Scalar(255), -1, 8, hierarchy, 0);					
					
#ifdef DEBUG_ROAD_SCAN
					char pName[100];
					sprintf(pName,"./landmark/landmark_%d_classify_%d.png",landmarkSquence,result);
					imwrite(pName,roi);
					landmarkSquence++;
#endif				
					
					
					landMark str_landmark;
					str_landmark.boundary = contours[i];
					str_landmark.landMarkWeight = weightPoint;
					str_landmark.uPoint = upoint;
					str_landmark.dPoint = dpoint;
					str_landmark.lPoint = lpoint;
					str_landmark.rPoint = rpoint;
					landmarks.push_back(str_landmark);

				}				
			}
		}		
	}
//	bitwise_not(Dilate,Dilate);
#ifdef DEBUG_ROAD_SCAN
	imwrite("number.png",T);
#endif			
	

}
/*
 * @FUNC
 *     recognize arrow,distinguish diffrenet arrows; 
 *     return the class of arrows,there are five classes,2001-2005,
 *     if it is not arrow,return -1;      
 * @PARAMS
 *     Inimg   -  input test image, ;
 *
 */
int arrowClassify(Mat &inImge)
{
	CvSVM svm_arrow;      
	CvSVM svm_arrow_F;
	CvSVM svm_arrow_L;
	CvSVM svm_arrow_LF;
	CvSVM svm_arrow_R;
	CvSVM svm_arrow_RF;

#ifdef ROAD_SCAN_UT
	svm_arrow.load("../../../../Ford/inVehicle/inVehicle/resource/Germany/roadarrow/ARROW_SVM_HOG.xml");
	svm_arrow_F.load("../../../../Ford/inVehicle/inVehicle/resource/Germany/roadarrow/F_ARROW_SVM_HOG.xml");
	svm_arrow_L.load("../../../../Ford/inVehicle/inVehicle/resource/Germany/roadarrow/L_ARROW_SVM_HOG.xml");
	svm_arrow_LF.load("../../../../Ford/inVehicle/inVehicle/resource/Germany/roadarrow/LF_ARROW_SVM_HOG.xml");
	svm_arrow_R.load("../../../../Ford/inVehicle/inVehicle/resource/Germany/roadarrow/R_ARROW_SVM_HOG.xml");
	svm_arrow_RF.load("../../../../Ford/inVehicle/inVehicle/resource/Germany/roadarrow/RF_ARROW_SVM_HOG.xml");
#else
	//svm.load("./resource/Germany/roadarrow/ARROW_SVM_HOG.xml");
	svm_arrow.load("./resource/Germany/roadarrow/ARROW_SVM_HOG.xml");
	svm_arrow_F.load("./resource/Germany/roadarrow/F_ARROW_SVM_HOG.xml");
	svm_arrow_L.load("./resource/Germany/roadarrow/L_ARROW_SVM_HOG.xml");
	svm_arrow_LF.load("./resource/Germany/roadarrow/LF_ARROW_SVM_HOG.xml");
	svm_arrow_R.load("./resource/Germany/roadarrow/R_ARROW_SVM_HOG.xml");
	svm_arrow_RF.load("./resource/Germany/roadarrow/RF_ARROW_SVM_HOG.xml");
#endif

	HOGDescriptor hog(Size(64,640),Size(16,16),Size(8,8),Size(8,8),9);
	int DescriptorDim;

	Mat testImg;
	resize(inImge,testImg,Size(64,640));
	vector<float> descriptor;
	hog.compute(testImg,descriptor,Size(8,8));
	Mat testFeatureMat = Mat::zeros(1,descriptor.size(),CV_32FC1);

	for(int i=0; i<descriptor.size(); i++)
		testFeatureMat.at<float>(0,i) = descriptor[i];
	int result =-1;
	result =  svm_arrow.predict(testFeatureMat);
	if (result==-1)
	{
		return -1;
	}
	else 
	{
		result =  svm_arrow_F.predict(testFeatureMat);
		if (result==-1)
		{
			result =  svm_arrow_L.predict(testFeatureMat);
			if (result==-1)
			{
				result =  svm_arrow_LF.predict(testFeatureMat);
				if (result==-1)
				{
					result =  svm_arrow_R.predict(testFeatureMat);
					if (result==-1)
					{
						result =  svm_arrow_RF.predict(testFeatureMat);
						if (result==-1)
						{
							return 0;
						}
						else
							return 2005;
					}
					else
						return 2004;
				}
				else
					return 2003;
			}
			else
				return 2002;
		}
		else
			return 2001;
	}


}
/*
 * @FUNC
 *     SurfFeature Match bettwen two images ; 
 *     return the numbers of surfeatures  ;      
 * @PARAMS
 *     src1   -  input test image;
 *     src2   -  input model image;
 */
int SurfFeatureMatch(Mat &src1,Mat &src2)
{
	int minHessian = 5;
	SurfFeatureDetector detector(minHessian);
	std::vector<KeyPoint> keyPoint1,keyPoint2;
    detector.detect(src1,keyPoint1);  
	detector.detect(src2,keyPoint2);
	Mat descriptors1,descriptors2;
	SurfDescriptorExtractor extractor;
	extractor.compute(src1,keyPoint1,descriptors1);
	extractor.compute(src2,keyPoint2,descriptors2);
	if (keyPoint1.size()&&keyPoint2.size())
	{
		BruteForceMatcher<L2<float>> matcher;
		std::vector<DMatch> matches;
		matcher.match(descriptors1,descriptors2,matches);
		int size = matches.size();
		int number=0;
		for (int i=0;i<size;i++)
		{
			if (matches[i].distance<0.1)
			{
				number++;
			}
		}
		/*Mat imgMatch;
		drawMatches(src1,keyPoint1,src2,keyPoint2,matches,imgMatch);
		imshow("sss",imgMatch);
		waitKey(10);*/
		return number;
	}
	else
		return 0;
	
}



/* 
  src and dst are grayscale, 8-bit images; 
  Default input value:  
           [low, high] = [0,1];  X-Direction 
           [bottom, top] = [0,1]; Y-Direction 
           gamma ; 
  if adjust successfully, return 0, otherwise, return non-zero. 
*/  
int imageAdjust(Mat &src, Mat &dst,   
	double low, double high,   // Xlow and high are the intensities of src  
	double bottom, double top, // Ymapped to bottom and top of dst  
	double gamma )
{
	if(     low<0 && low>1 && high <0 && high>1&&  
		bottom<0 && bottom>1 && top<0 && top>1 && low>high)  
		return -1;  
	double low2 = low*255;  
	double high2 = high*255;  
	double bottom2 = bottom*255;  
	double top2 = top*255;  
	double err_in = high2 - low2;  
	double err_out = top2 - bottom2;  

	int x,y;  
	double val;  

	// intensity transform  
	for( y = 0; y < src.rows; y++)  
	{  
		for (x = 0; x < src.cols; x++)  
		{  
			val = ((uchar*)(src.data + src.cols*y))[x];   
			val = pow((val - low2)/err_in, gamma) * err_out + bottom2;  
			if(val>255) val=255; if(val<0) val=0; // Make sure src is in the range [low,high]  
			((uchar*)(dst.data + dst.cols*y))[x] = (uchar) val;  
		}  
	}  
	return 0;  
}  
/*
 * @FUNC
 *     process the image with shadow,to enhance the contrast of image;      
 * @PARAMS
 *     src   -  input src image;
 *     src   -  output dst image;
 */
void shadowProcess(Mat &src, Mat &dst)
{
    Mat element1 = getStructuringElement(MORPH_ELLIPSE,Size(10,10));
    Mat openDst1;
    morphologyEx(src,openDst1,MORPH_CLOSE,element1,Point(-1,-1),2);
    
	Mat element = getStructuringElement(MORPH_ELLIPSE,Size(15,15));
	Mat openDst;
	morphologyEx(openDst1,openDst,MORPH_OPEN,element,Point(-1,-1),2);
	
	Mat dst1 = abs(openDst1-openDst);
	int s = imageAdjust(dst1,dst,0,0.3,0,0.7,1);
#ifdef ROAD_SCAN_UT
    imwrite("close.png",openDst1);
    imwrite("open.png",openDst);
#endif

}
/*
 * @FUNC
 *     judge wheather the image has shadow, or needs to enhance the contrast;
 *     return 1, it needs to enhance contrast; return 0, it doesn't need to enhance
 * @PARAMS
 *     src   -  input src image;
 */
bool judgeShadow(Mat &src)
{
	Mat dstImage(256,256,CV_8UC1,Scalar(0));
	int hist=0;

	MatND dstHist;
	int dims=1;
	float hranges[] = {0,255};
	const float *rangs[] = {hranges};
	int channels = 0;
	int size =256;
	calcHist(&src,1,&channels,Mat(),dstHist,dims,&size,rangs);
	int scale=1;

	double minValue = 0;
	double maxValue = 0;
	minMaxLoc(dstHist,&minValue,&maxValue,0,0);

	int hpt = saturate_cast<int>(0.9 * size);
	for (int i=0;i<256;i++)
	{
		float binValue = dstHist.at<float>(i);
		int realValue = saturate_cast<int>(binValue * hpt/maxValue);
		rectangle(dstImage,Point(i*scale,size-1),Point((i+1)*scale-1,size-realValue),Scalar(255));
	}

	int sum = 0;
	for (int i=0;i<256;i++)
	{
		float binValue = dstHist.at<float>(i);
		sum +=binValue;
	}

	double widthPer =0;
	double numWidthPlus=0;
	for (int i=0;i<100;i++)
	{
		float binValue = dstHist.at<float>(i);
		numWidthPlus +=binValue;
		widthPer = numWidthPlus/(sum);
	}
#ifdef ROAD_SCAN_UT
//	cout<<"shadowper="<<widthPer<<endl;
#endif
	if (widthPer>0.30)
	{
//		cout<<"shadowper="<<widthPer<<endl;
		return true;
		
	}
	else
		return false;

}
/*
 * @FUNC
 *     find the interval according to coordinates(x,y) in image, and cal the relGPS;
 * @PARAMS
 *     gpsAndInterval   -  store gps and interva;
 *     point            -  coordinates(x,y) in image;
 *     inImage          -  input image;
 *     inParam          -  config info
 *     pointRel         -  output relative GPS
 */
void findGPSInterval(vector<gpsInformationAndInterval> &gpsAndInterval,Point2d &point,
    Mat &inImage,Parameters inParam, Point2d &pointRel)
{
	int countnow = 0;
	int countnext = 0;
	int H = inImage.rows;
	
	for (int i=0;i<gpsAndInterval.size()-1;i++)
	{
		Point2d GPS_1,GPS_2;
		coordinateChange(gpsAndInterval[i+1].GPS_now,inParam.GPSref,GPS_1);
		coordinateChange(gpsAndInterval[i+1].GPS_next,inParam.GPSref,GPS_2);
		countnow += gpsAndInterval[i].intervalOfInterception;
		countnext  = countnow+gpsAndInterval[i+1].intervalOfInterception;
		if (H-countnow>point.y&&H-countnext<point.y)
		{
			Point2d point2 = Point2d(point.x-inImage.cols/2,H-countnow-point.y);
			getGPSLocationOfEveryPixelInRoadScanImage(GPS_1,GPS_2,
				point2,gpsAndInterval[i+1].intervalOfInterception,pointRel,inParam.distancePerPixel);
		}
	}
}

/*
 * @FUNC
 *     link the lines which are adjacent to each other;
 * @PARAMS
 *     src            -  input image;
 *     dst            -  output image;
 */
void linkInterval(Mat &src, Mat &dst)
{

	int w = src.cols;
	int h = src.rows;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
  
	vector<SLineInfo> vecSlineinfo;//save information of every short line

	for (unsigned int i = 0; i < contours.size(); i++)
	{

		//pixel mean value of contour
		SLineInfo sline;
		Vec4f line;
		vector<Point> pointss = contours[i];
		sline.upoint = pointss[0];
		sline.dpoint = pointss[pointss.size()-1];

		for (int j=0;j<pointss.size();j++)//calculate up and down points of contours
		{
			if (pointss[j].y<sline.upoint.y)
			{
				sline.upoint = pointss[j];
			}
			if (pointss[j].y>sline.dpoint.y)
			{
				sline.dpoint = pointss[j];
			}

		}
		vecSlineinfo.push_back(sline);
        drawContours(src, contours, i, Scalar(255), -1, 8, hierarchy, 0);
	}
	for (int i=0;i<vecSlineinfo.size();i++)
	{
		for(int j=0;j<vecSlineinfo.size();j++)
		{
			if (abs(vecSlineinfo[i].upoint.y-vecSlineinfo[j].dpoint.y)<40&&abs(vecSlineinfo[i].upoint.x-vecSlineinfo[j].dpoint.x)<10)
			{
				double dist = sqrt(double((vecSlineinfo[i].upoint.x-vecSlineinfo[j].dpoint.x)*(vecSlineinfo[i].upoint.x-vecSlineinfo[j].dpoint.x)
					+(vecSlineinfo[i].upoint.y-vecSlineinfo[j].dpoint.y)*(vecSlineinfo[i].upoint.y-vecSlineinfo[j].dpoint.y)));
				if (dist<40)
				{
					line(src,vecSlineinfo[i].upoint,vecSlineinfo[j].dpoint,Scalar(255),1);
				}
			}		
		}		
	}
	threshold(src,dst,5,255,0);

}
/*
 * @FUNC
 *     ride detection by block calculation, in each block, it can cal ridge params
 *     adaptivly;
 * @PARAMS
 *     longLane_cut       -  input image;
 *     inParam            -  config info;
 *     allUnitKapa        -  the image after ridge detection;
 *     allUnitContous     -  contours image;
 *     allKappBin         -  binary image for allUnitKapa;
 */
void blockCalRidge(Mat &longLane_cut, Parameters& inParam, Mat &allUnitKapa,
    Mat &allUnitContous,Mat &allKappBin)
{
    int h = longLane_cut.rows;
    int w = longLane_cut.cols;
    int unitH=1000;
    int unitNum = ceil(1.0*h/unitH);
     for (int uniti=1;uniti<=unitNum;uniti++)
     {

#ifdef ROAD_SCAN_UT
         double startT = static_cast<double>(cv::getTickCount());
         cout<<unitNum<<","<<uniti<<endl;
#endif
         Mat unit;
         if (uniti==unitNum)
             unit = longLane_cut(Rect(0,(uniti-1)*unitH,longLane_cut.cols,longLane_cut.rows-(uniti-1)*unitH));
         else
             unit = longLane_cut(Rect(0,(uniti-1)*unitH,longLane_cut.cols,unitH));       

         bool isShadow = judgeShadow(unit);
         if (isShadow)
         {
//             shadowProcess(unit, unit);
         }			
         Mat unitlongLaneLeft = unit(Rect(0,0,unit.cols/2,unit.rows));
         Mat unitlongLaneRight = unit(Rect(unit.cols/2,0,unit.cols/2,unit.rows));
         int size = 256;
         Mat dstImage1(size,size,CV_8UC1,Scalar(0));
         Mat dstImage2(size,size,CV_8UC1,Scalar(0));
         int unithisLeftTh,unithisRightTh;

         calThread(unitlongLaneLeft,unithisLeftTh,dstImage1);
         calThread(unitlongLaneRight,unithisRightTh,dstImage2);
#ifdef ROAD_SCAN_UT
         double time5 = (static_cast<double>(cv::getTickCount() - startT))/cv::getTickFrequency();
#endif
         Mat unitTH,unitleftTH,unitrightTH;
         threshold(unitlongLaneLeft,unitleftTH,unithisLeftTh,255,0);
         threshold(unitlongLaneRight,unitrightTH,unithisRightTh,255,0);
         int unitridgeLeftPar = calRidgePar(unitleftTH);
         int unitridgeRightPar = calRidgePar(unitrightTH);
#ifdef ROAD_SCAN_UT
         double time6 = (static_cast<double>(cv::getTickCount() - startT))/cv::getTickFrequency();
//         std::cout << "calRidgePar time: "<<time6-time5<< std::endl;
#endif
#ifdef ROAD_SCAN_UT
         cout<<"unitridgeLeftPar="
             <<unitridgeLeftPar<<"unitridgeRightPar="<<unitridgeRightPar<<endl;
#endif


         int unithistThres=0;
         if (unithisLeftTh==0||unithisRightTh==0)
             unithistThres = unithisRightTh+unithisLeftTh;
         else
             unithistThres = (unithisLeftTh + unithisRightTh)/2;
         unithistThres = max(unithisLeftTh,unithisRightTh);
#ifdef ROAD_SCAN_UT
         cout<<"unitleft="<<unithisLeftTh<<"unitright="<<unithisRightTh<<endl;
#endif
         //			cout<<"unitleft="<<unithisLeftTh<<"unitright="<<unithisRightTh<<endl;

         Mat unitimageLeft_32f,unitkapaLeft,unitimageRight_32f,unitkapaRight;
         unitlongLaneLeft.convertTo(unitimageLeft_32f,CV_32F);
         unitlongLaneRight.convertTo(unitimageRight_32f,CV_32F);

         if (unitridgeLeftPar<9)
         {
             unitridgeLeftPar=9;
         }
         if (unitridgeRightPar<9)
         {
             unitridgeRightPar=9;
         }
#ifdef ROAD_SCAN_UT
         double time1 = (static_cast<double>(cv::getTickCount() - startT))/cv::getTickFrequency();

#endif
         ridgeDetect(unitimageLeft_32f,unitkapaLeft,unitridgeLeftPar/3.0,unitridgeLeftPar/3.0);
         ridgeDetect(unitimageRight_32f,unitkapaRight,unitridgeRightPar/3.0,unitridgeRightPar/3.0);
         unitkapaLeft.convertTo(unitkapaLeft,CV_8UC1,1024); 
         unitkapaRight.convertTo(unitkapaRight,CV_8UC1,1024); 
#ifdef ROAD_SCAN_UT
         double time2 = (static_cast<double>(cv::getTickCount() - startT))/cv::getTickFrequency();  
 //        std::cout << "ridge time: "<<time2-time1<< std::endl;
#endif
         Mat unitKapa(unit.size(),CV_8UC1);
         Mat unitkapaRoiLeft = unitKapa(Rect(0,0,unit.cols/2,unit.rows));
         Mat unitkapaRoiRight = unitKapa(Rect(unit.cols/2,0,unit.cols/2,unit.rows));
         unitkapaLeft.copyTo(unitkapaRoiLeft);
         unitkapaRight.copyTo(unitkapaRoiRight);


         Mat unitKapaBin;

         if (isShadow)
             threshold( unitKapa, unitKapaBin, inParam.ridgeThreshold, 255,0 );
         else
             threshold( unitKapa, unitKapaBin, inParam.ridgeThreshold-5, 255,0 );

         vector<vector<Point> > unitcontours;
         vector<Vec4i> unithierarchy;
         findContours(unitKapaBin, unitcontours, unithierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

         // draw contours:   

         Mat unitTSline  = Mat::zeros(unitKapa.size(), CV_8UC1);
         vector<SLineInfo> vecSlineinfo;//save information of every short line

         for (unsigned int i = 0; i < unitcontours.size(); i++)
         {

             //pixel mean value of contour
             double meanvalue=0;;
             SLineInfo sline;
             Vec4f line;
             fitLine(unitcontours[i],line,CV_DIST_L2 ,0,0.01,0.01);//linear fit for short line contour
             sline.vx = line[0];
             sline.vy = line[1];

             double C = (line[3]*line[0]-line[2]*line[1]);
             vector<double> Dist;
             vector<Point> pointss = unitcontours[i];
             sline.upoint = pointss[0];
             sline.dpoint = pointss[pointss.size()-1];
             sline.WeiPoint = Point(0,0);//line center of gravity

             for (int j=0;j<pointss.size();j++)//calculate up and down points of contours
             {
                 if (pointss[j].y<sline.upoint.y)
                 {
                     sline.upoint = pointss[j];
                 }
                 if (pointss[j].y>sline.dpoint.y)
                 {
                     sline.dpoint = pointss[j];
                 }
                 meanvalue += unit.at<uchar>(pointss[j].y,pointss[j].x);
                 sline.WeiPoint.x+=pointss[j].x;
                 sline.WeiPoint.y+=pointss[j].y;
                 double dst = abs(line[1]*pointss[j].x - line[0]*pointss[j].y +C);
                 //			cout<<dst<<endl;
                 Dist.push_back(dst);
             }
             meanvalue = meanvalue/pointss.size();
             double dst = sqrt(double((sline.dpoint.y - sline.upoint.y)*(sline.dpoint.y - sline.upoint.y)
                 +(sline.dpoint.x - sline.upoint.x)*(sline.dpoint.x - sline.upoint.x)));
             int number = Dist.size();
             double sum =0;
             for (int ii=0;ii<number;ii++)
                 sum+=Dist[ii];
             double e=sum/number; 
             double s=0;

             for (int ii=0;ii<number;ii++) 
                 s+=(Dist[ii]-e)*(Dist[ii]-e);

             s=sqrt(s/number);

             if (s>2)
             {
                 if (dst<100)
                 {
                     continue;
                 }		
             }
             sline.WeiPoint = Point(sline.WeiPoint.x/unitcontours[i].size(),sline.WeiPoint.y/unitcontours[i].size());
             sline.vx = line[0];
             sline.vy = line[1];
             double kk = atan(line[1]/(line[0]+0.000001));
             double angle  = abs(180*kk/PI);
             if (sline.WeiPoint.x>(w/2-100)&&sline.WeiPoint.x<(w/2+100))
             {
                 //		continue;
             }
             if (angle<75)
             {
                 continue;
             }
             double kk2 = atan((sline.upoint.y - sline.WeiPoint.y)/(sline.upoint.x - sline.WeiPoint.x+0.000000001));//首和重心点斜率；
             double angle2  = abs(180*kk2/PI);
             if (angle2<75)
             {
                 continue;
             }
            
             if (!isShadow)
             {
                 unithisLeftTh  = 0;
                 unithisRightTh = 0;
             }
             if (sline.upoint.x<w/2&&meanvalue>unithisLeftTh&&dst>100)
             {
                 drawContours(unitTSline, unitcontours, i, Scalar(255), -1, 8, unithierarchy, 0);
             }

             if (sline.upoint.x>=w/2&&meanvalue>unithisRightTh&&dst>100)
             {
                 drawContours(unitTSline, unitcontours, i, Scalar(255), -1, 8, unithierarchy, 0);
             }
         }

         if (uniti==unitNum)
         {
             Mat kaparoi = allUnitKapa(Rect(0,(uniti-1)*unitH,allUnitKapa.cols,allUnitKapa.rows-(uniti-1)*unitH));
             unitKapa.copyTo(kaparoi);

#ifdef DEBUG_ROAD_SCAN
             imwrite( "allUnitKapa.png", allUnitKapa );
#endif


             threshold( allUnitKapa, allKappBin,5, 255,0 );
             //imwrite("allKappBin.png",allKappBin);

             Mat contoursroi = allUnitContous(Rect(0,(uniti-1)*unitH,allUnitKapa.cols,allUnitKapa.rows-(uniti-1)*unitH));

             unitTSline.copyTo(contoursroi);

             //imwrite("allUnitContous.png",allUnitContous);

         }
         else
         {
             Mat kaparoi = allUnitKapa(Rect(0,(uniti-1)*unitH,allUnitKapa.cols,unitH));
             unitKapa.copyTo(kaparoi);

             Mat contoursroi = allUnitContous(Rect(0,(uniti-1)*unitH,allUnitKapa.cols,unitH));
             unitTSline.copyTo(contoursroi);
         }
    }
}
/*
 * @FUNC
 *     link the painting lines, and eliminate the noise,
 * @PARAMS
 *     allUnitContous     -  input contous image;
 *     landmark         -  landmark information;
 *     Tline_link_out     -  output paiting linked image;
 */
void linkPaintLine(Mat allUnitContous,vector<landMark> &landmark,Mat &Tline_link_out)
{

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(allUnitContous, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    Mat TSline  = Mat::zeros(allUnitContous.size(), CV_8UC1);
    Mat TSlinMark = Mat::zeros(allUnitContous.size(), CV_8UC3);
    vector<SLineInfo> vecSlineinfo;//save information of every short line

    for (unsigned int i = 0; i < contours.size(); i++)
    {

        //pixel mean value of contour
        double meanvalue=0;;
        SLineInfo sline;
        Vec4f line;
        fitLine(contours[i],line,CV_DIST_L2 ,0,0.01,0.01);//linear fit for short line contour
        sline.vx = line[0];
        sline.vy = line[1];

        double C = (line[3]*line[0]-line[2]*line[1]);
        vector<double> Dist;
        vector<Point> pointss = contours[i];
        sline.upoint = pointss[0];
        sline.dpoint = pointss[pointss.size()-1];
        sline.WeiPoint = Point(0,0);//line center of gravity

        for (int j=0;j<pointss.size();j++)//calculate up and down points of contours
        {
            if (pointss[j].y<sline.upoint.y)
            {
                sline.upoint = pointss[j];
            }
            if (pointss[j].y>sline.dpoint.y)
            {
                sline.dpoint = pointss[j];
            }
            sline.WeiPoint.x+=pointss[j].x;
            sline.WeiPoint.y+=pointss[j].y;

        }
        sline.WeiPoint = Point(sline.WeiPoint.x/contours[i].size(),sline.WeiPoint.y/contours[i].size());
        bool isdelete = false;

        if (landmark.size())
        {
            for(int j=0;j<landmark.size();j++)
            {
                if (sline.WeiPoint.x>landmark[j].lPoint.x&&sline.WeiPoint.x<landmark[j].rPoint.x
                    &&sline.WeiPoint.y<landmark[j].dPoint.y&&sline.WeiPoint.y>landmark[j].uPoint.y)
                {
                    isdelete = true;
                    break;
                }

            }
            if (isdelete==true)
            {
                isdelete = false;
                continue;
            }		
        }


        //			if (sline.upoint.x<w/2&&meanvalue>hisLeftTh&&dst>100)
        {
            drawContours(TSline, contours, i, Scalar(255), -1, 8, hierarchy, 0);
            drawContours(TSlinMark, contours, i, Scalar(255,255,255), -1, 8, hierarchy, 0);
            circle(TSlinMark,sline.WeiPoint,0,Scalar(255,0,0),4);
            circle(TSlinMark,sline.upoint,0,Scalar(0,255,0),4);
            circle(TSlinMark,sline.dpoint,0,Scalar(0,0,255),4);
            vecSlineinfo.push_back(sline);

        }
    }
#ifdef DEBUG_ROAD_SCAN
    imwrite( "TSlinMark.png", TSlinMark );
#endif

    /////one up point only links to one down points
    vector<PairPoints> vecPairPoints;//save up and down point pair;
    int *objptr = new int[vecSlineinfo.size()];
    int *srcptr = new int[vecSlineinfo.size()];
    for(int i=0;i<vecSlineinfo.size();i++)
    {
        objptr[i]=0;
        srcptr[i]=0;
    }

    for(int i = 0;i<vecSlineinfo.size();i++)
    {
        ///	int ddddd = vecSlineinfo[i].WeiPoint.y;
        for (int j = i+1;j<vecSlineinfo.size();j++)
        {			
            //double dsty = vecSlineinfo[i].WeiPoint.y-vecSlineinfo[j].WeiPoint.y;
            double dsty = vecSlineinfo[i].upoint.y-vecSlineinfo[j].dpoint.y;
            double dstx = vecSlineinfo[i].upoint.x-vecSlineinfo[j].dpoint.x;
            Point2d diffw = Point2d(vecSlineinfo[i].WeiPoint.x-vecSlineinfo[j].WeiPoint.x,
                vecSlineinfo[i].WeiPoint.y-vecSlineinfo[j].WeiPoint.y);
            double cosvalue = abs(vecSlineinfo[i].vx*vecSlineinfo[j].vx+vecSlineinfo[i].vy*vecSlineinfo[j].vy);

            double cosvalue2 = abs(dstx*vecSlineinfo[j].vx+dsty*vecSlineinfo[j].vy)/
                sqrt((dstx*dstx+dsty*dsty)*(vecSlineinfo[j].vx*vecSlineinfo[j].vx+vecSlineinfo[j].vy*vecSlineinfo[j].vy));

            double cosvalue3 = abs(dstx*vecSlineinfo[j].vx+dsty*vecSlineinfo[j].vy)/
                sqrt((dstx*dstx+dsty*dsty)*(vecSlineinfo[j].vx*vecSlineinfo[j].vx+vecSlineinfo[j].vy*vecSlineinfo[j].vy));
        
            double tanangle = atan(abs(diffw.y/(diffw.x+0.0000000001)))*180/PI;

            //if (abs(dsty)>1500||tanangle<75||cosvalue<0.8||abs(dstx)>20||dsty<0)//Threshold condition
            if (abs(dsty)>2000||cosvalue3<0.985||cosvalue2<0.985||cosvalue<0.985||dsty<0||abs(dstx)>allUnitContous.cols/3)
            {
                continue;
            }
            objptr[j] +=1; 
            srcptr[i] +=1;

            PairPoints str_pairpoints;
            str_pairpoints.up = vecSlineinfo[i].upoint;
            str_pairpoints.down = vecSlineinfo[j].dpoint;
            str_pairpoints.objindex =j;//object line index；
            str_pairpoints.srcindex =i;//source line index；
            circle(TSlinMark,vecSlineinfo[i].WeiPoint,0,Scalar(255,0,0),4);
            circle(TSlinMark,vecSlineinfo[i].upoint,0,Scalar(0,255,0),4);
            circle(TSlinMark,vecSlineinfo[i].dpoint,0,Scalar(0,0,255),4);
            vecPairPoints.push_back(str_pairpoints);

        }
    }
#ifdef DEBUG_ROAD_SCAN
    imwrite("TSlinMark.png",TSlinMark);
#endif
    //if more than one up points link to the same down point
    for (int i = 0;i<vecSlineinfo.size();i++)
    {
        if (objptr[i]>1)
        {
            int num = objptr[i];
            Point *TemUPoints = new Point[num];
            Point TemDPoints;
            int n=0;
            int *index = new int[num];
            for (int i1 = 0;i1<vecPairPoints.size();i1++)
            {
                if (vecPairPoints[i1].objindex==i)
                {
                    TemUPoints[n] = vecPairPoints[i1].up;
                    index[n] = i1;
                    n++;
                    TemDPoints = vecPairPoints[i1].down;						
                }
            }
            int valuemin = abs(TemUPoints[0].y-TemDPoints.y);
            int reindext  = 0;
            reindext = index[0];
            int tempreindext = index[0];

            for (int i2=1;i2<n;i2++)
            {	
                double dx = vecPairPoints[index[i2]].up.x-vecPairPoints[index[i2]].down.x;
                double dy = vecPairPoints[index[i2]].up.y-vecPairPoints[index[i2]].down.y;
                double dxx = dx/sqrt(dx*dx+dy*dy);
                double dyy = dy/sqrt(dx*dx+dy*dy);
                double cosvalue0 = vecSlineinfo[i].vx*dxx + vecSlineinfo[i].vy*dyy;
                double cosvalue = abs(vecSlineinfo[i].vx*dx + vecSlineinfo[i].vy*dy)/
                    sqrt((dx*dx+dy*dy)*(vecSlineinfo[i].vx*vecSlineinfo[i].vx+vecSlineinfo[i].vy*vecSlineinfo[i].vy));
                double tanangle = atan(abs(dy/(dx+0.0000000001)))*180/PI;
                double cosangle = acos(cosvalue)*180/PI;
                if (cosvalue<0.98)
                {
                    reindext = index[i2];
                    vecPairPoints[reindext].up=Point(0,0);
                    vecPairPoints[reindext].down=Point(0,0);
                    continue;
                }
                if (valuemin<abs(TemUPoints[i2].y-TemDPoints.y))
                {
                    reindext = index[i2];
                    vecPairPoints[reindext].up=Point(0,0);
                    vecPairPoints[reindext].down=Point(0,0);					
                }
                else
                {
                    valuemin=abs(TemUPoints[i2].y-TemDPoints.y);						
                    vecPairPoints[tempreindext].up=Point(0,0);
                    vecPairPoints[tempreindext].down=Point(0,0);
                    tempreindext = index[i2];
                    reindext = index[i2];
                }

            }
            delete TemUPoints;
            delete index;
        }

    }

    //		if one up point links to more than one down points
    for (int i = 0;i<vecSlineinfo.size();i++)
    {
        if (srcptr[i]>1)
        {
            int num = srcptr[i];
            Point *TemDPoints = new Point[num];
            Point TemUPoints;
            int n=0;
            int *index = new int[num];
            for (int i1 = 0;i1<vecPairPoints.size();i1++)
            {

                if (vecPairPoints[i1].srcindex==i)
                {
                    if (vecPairPoints[i1].up.x)
                    {
                        TemDPoints[n] = vecPairPoints[i1].down;
                        index[n] = i1;
                        n++;
                        TemUPoints = vecPairPoints[i1].up;		
                    }										
                }
            }
            int valuemin = abs(TemDPoints[0].y-TemUPoints.y);
            int reindext  = 0;
            reindext = index[0];
            int tempreindext = index[0];
            Point minvalupoint,minvaldpoint;

            for (int i2=1;i2<n;i2++)
            {					
                if (valuemin<abs(TemDPoints[i2].y-TemUPoints.y))
                {
                    reindext = index[i2];
                    vecPairPoints[reindext].up=Point(0,0);
                    vecPairPoints[reindext].down=Point(0,0);					
                }
                else
                {
                    valuemin=abs(TemDPoints[i2].y-TemUPoints.y);						
                    vecPairPoints[tempreindext].up=Point(0,0);
                    vecPairPoints[tempreindext].down=Point(0,0);
                    tempreindext = index[i2];
                    reindext = index[i2];
                }
            }
            delete TemDPoints;
            delete index;
        }
    }

    delete objptr;
    delete srcptr;

    for (int i = 0;i<vecPairPoints.size();i++)
    {
        int dx = abs(vecPairPoints[i].up.x - vecPairPoints[i].down.x);
        if (vecPairPoints[i].up!=Point(0,0)&&(vecPairPoints[i].up.y>vecPairPoints[i].down.y)&&dx<150)
        {
            line(TSline,vecPairPoints[i].up,vecPairPoints[i].down,Scalar(255,255,0),1);
        }

    }

    Tline_link_out = Mat::zeros(allUnitContous.size(), CV_8UC1);
    threshold( TSline, TSline, 20, 255,0 );
#ifdef DEBUG_ROAD_SCAN
    imwrite("TSline.png",TSline);
#endif
    vector<vector<Point>> contours4;
    vector<Vec4i> hierarchy4;

    findContours(TSline, contours4, hierarchy4, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    for (unsigned int i = 0; i < contours4.size(); i++)
    {
        vector<Point> pointss = contours4[i];
        Point upoint = pointss[0];
        Point dpoint = pointss[pointss.size()-1];
        for (int j=0;j<pointss.size();j++)
        {
            if (pointss[j].y<upoint.y)
            {
                upoint = pointss[j];
            }
            if (pointss[j].y>dpoint.y)
            {
                dpoint = pointss[j];
            }
        }
        int dst = abs(dpoint.y - upoint.y);

        if (abs(upoint.x-dpoint.x)<50&&abs((upoint.x+dpoint.x)/2-Tline_link_out.cols/2)<50)
        {
            continue;
        }

        if (dst>600)//delete the hight less than 600
        {	
            drawContours(Tline_link_out, contours4, i, Scalar(255), 1, 8, hierarchy4, 0);
        }
    }

#ifdef DEBUG_ROAD_SCAN
    imwrite("Tline_link_out.png",Tline_link_out);
#endif

}

/*
 * @FUNC
 *     detection arrows, cal arrows' location(x,y) in image
 * @PARAMS
 *     longLane    -  input src gray image;
 *     longLaneBW  -  input binary image;
 *     arrows      -  arrows weight points and boundary;
 *     laneBW      -  image, which doesn't contain arrows;
 */
void arrowDetection(Mat &longLane,Mat &longLaneBW,vector<landMark> &arrows,Mat &laneBW)
{
    Mat src;
    longLaneBW.copyTo(src);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    // draw contours:   

    Mat T = Mat::zeros(src.size(), CV_8UC1);
    laneBW = Mat::zeros(src.size(), CV_8UC1);
    int w = src.cols/4;
    int meanValue=0;
    for (unsigned int i = 0; i < contours.size(); i++)
    {			
        vector<Point> pointss = contours[i];
        Point upoint = pointss[0];
        Point dpoint = pointss[pointss.size()-1];
        Point lpoint = pointss[0];
        Point rpoint = pointss[pointss.size()-1];
        Point2d weightPoint = Point2d(-1,-1);
        for (int j=0;j<pointss.size();j++)
        {
            if (pointss[j].y<upoint.y)
            {
                upoint = pointss[j];
            }
            if (pointss[j].y>dpoint.y)
            {
                dpoint = pointss[j];
            }			

            if (pointss[j].x<lpoint.x)
            {
                lpoint = pointss[j];
            }
            if (pointss[j].x>rpoint.x)
            {
                rpoint = pointss[j];
            }
            weightPoint.x += pointss[j].x;
            weightPoint.y += pointss[j].y;

            meanValue += src.at<uchar>(pointss[j].y,pointss[j].x);
        }
        weightPoint.x = weightPoint.x/pointss.size();
        weightPoint.y = weightPoint.y/pointss.size();
        meanValue = meanValue/contours.size();

        int dx = rpoint.x - lpoint.x;
        int dy = dpoint.y - upoint.y;

        double values = dy/(dx+0.0000000001);
        if (abs(values-1)<0.3)
        {
            drawContours(laneBW, contours, i, Scalar(255), -1, 8, hierarchy, 0);
            continue;
        }
//      if (rpoint.x<w*3&&lpoint.x>w)
        {
            if (contourArea(contours[i])>2000&&dy<1000)
            {
                Mat roi = src(Rect(lpoint.x,upoint.y,dx,dy));

                int result = arrowClassify(roi);

#ifdef ROAD_SCAN_UT
                //				cout<<"result："<<result<<endl;
#endif							

                if (result !=-1&&result!=0)
                {
                    //		cout<<"matchnum="<<numMatchTemp<<"model="<<index<<endl;					
                    RotatedRect rect = minAreaRect(contours[i]);
                    Point2f vertex[4];
                    rect.points(vertex);
                    for (int j =0;j<4;j++)
                    {
                        line(T,vertex[j],vertex[(j+1)%4],Scalar(255,0,0),2,8);
                    }				
                    drawContours(T, contours, i, Scalar(255), -1, 8, hierarchy, 0);	
                    imwrite("arrows.png",T);
                    

#ifdef DEBUG_ROAD_SCAN
                    char pName[100];
                    sprintf(pName,"./landmark/landmark_%d_classify_%d.png",landmarkSquence,result);
                    imwrite(pName,roi);
                    landmarkSquence++;
#endif				


                    landMark str_landmark;
                    str_landmark.boundary = contours[i];
                    str_landmark.landMarkWeight = weightPoint;
                    str_landmark.uPoint = upoint;
                    str_landmark.dPoint = dpoint;
                    str_landmark.lPoint = lpoint;
                    str_landmark.rPoint = rpoint;
                    arrows.push_back(str_landmark);

                }	
                else
                {
                    drawContours(laneBW, contours, i, Scalar(255), -1, 8, hierarchy, 0);
                }
            }
            else
            {
                drawContours(laneBW, contours, i, Scalar(255), -1, 8, hierarchy, 0);
            }
        }
    }
}
/*
 * @FUNC
 *     detect stop lines.
 * @PARAMS
 *     src           -  input src gray image;
 *     stopLine      -  stop lines weight points and boundary;
 *     dst           -  image, which doesn't contain stop line;
 */
void stopLineDetection(Mat &src, vector<landMark> &stopLine, Mat &dst)
{
    Mat binaryImg;
    gray2BW(src,binaryImg);

}



/*
 * @FUNC
 *     cal H matrix and invertH,
 * @PARAMS
 *     inParam     -  config info;
 *     H           -  H matrix;
 *     invertH     -  cal svd for H;
 */
void calHAndInvertH(Parameters &inParam, Mat &H, Mat &invertH)
{
	int w = inParam.imageCols * inParam.imageScaleWidth;
	int h = inParam.imageRows * inParam.imageScaleHeight;

	Point A = Point(0, h-1);
	Point B = Point(w-1, h-1);

	Point2f objPts[4], imgPts[4];

	imgPts[0].x=(inParam.centerPoint.x+A.x)/2-inParam.distanceOfSlantLeft;
	imgPts[0].y=(inParam.centerPoint.y+A.y)/2;

	imgPts[1].x= (inParam.centerPoint.x+B.x)/2+inParam.distanceOfSlantRight;
	imgPts[1].y= (inParam.centerPoint.y+B.y)/2;

	imgPts[2].x= A.x;
	imgPts[2].y= A.y;

	imgPts[3].x= B.x;
	imgPts[3].y= B.y;

	double scale = 1;
	double xoff = 2*w/10;
	double hh = scale*(A.y -imgPts[0].y)/2;

	objPts[0].x = 6*scale*A.x/10+xoff-inParam.distanceOfLeft;
	objPts[0].y = A.y*scale+inParam.distanceOfUpMove;

	objPts[1].x = 6*scale*B.x/10+xoff+inParam.distanceOfRight;
	objPts[1].y = A.y*scale+inParam.distanceOfUpMove;

	objPts[2].x = 6*scale*A.x/10+xoff-inParam.distanceOfLeft;
	objPts[2].y = scale*A.y+hh*(inParam.lengthRate+1)+inParam.distanceOfUpMove;

	objPts[3].x = 6*scale*B.x/10+xoff+inParam.distanceOfRight;
	objPts[3].y = scale*B.y+hh*(inParam.lengthRate+1)+inParam.distanceOfUpMove;

	H = getPerspectiveTransform( objPts, imgPts);

	// cal invert of H
	invert(H, invertH, CV_SVD);
}

/*
 * @FUNC
 *     gray image to binary image, delete the area that less than threshold, 
 * 
 * @PARAMS
 *     src     -  input gray image;
 *     dst     -  output binary image;
 */
void gray2BW(Mat src, Mat &dst)
{
    int H = src.rows;
    int W = src.cols;
    int nchanel = src.channels();
    if (nchanel >1)
    {
        cvtColor(src,src,COLOR_RGB2GRAY);
    }
    Size size = src.size();
    src.convertTo(src,CV_32F);
    int wize = 50;
    Mat Kenel = Mat::ones(Size(wize,wize),CV_32F); 
    Mat src2;
    src.copyTo(src2);  
    Mat I = Mat::zeros(size,CV_32F);
    filter2D(src,src,CV_32F,Kenel);
    src = src/(wize*wize);
    int h = src.rows;
    int w = src.cols;
    for (int i=0;i<h;i++)
    {
        for (int j=0;j<w;j++)
        {
            if (src2.at<float>(i,j)-src.at<float>(i,j)>20)
            {
                 I.at<float>(i,j) = 255;
            }
            else
                 I.at<float>(i,j) = 0;
        }
    }
    I.convertTo( I,CV_8UC1);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( I, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    dst  = Mat::zeros( I.size(), CV_8UC1);
    for (unsigned int i = 0; i < contours.size(); i++)
    {   
        if (contourArea(contours[i])>1000)
        {
            drawContours(dst, contours, i, Scalar(255), -1, 8, hierarchy, 0);
        }
    }
    threshold(dst,dst,5,255,0);

}
/*
 * @FUNC
 *     detect the car whether it is change lanes,
 *     return true: change lanes;
 *     return false: don't change lanes;
 * @PARAMS
 *     src     -  input gray image;
 */
bool changeLaneDection(Mat &src)
{
    if (src.rows ==0 ||src.cols ==0)
    {
        return false;
    }
    int startPos = 170;
    int endPos   = 430;
    int H = src.rows;
    int W = src.cols;
    //step 1: cal binary image
    Mat srcbw;
    gray2BW(src,srcbw);   
    //step 2: delete the noise and the part in middle image
    srcbw.convertTo(srcbw,CV_8UC1);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(srcbw, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    Mat I  = Mat::zeros(srcbw.size(), CV_8UC1);
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        vector<Point> pointss = contours[i];
        Point2d weiPoint = Point(0,0);//line center of gravity
        for (int j=0;j<pointss.size();j++)//calculate up and down points of contours
        {
            weiPoint.x+=pointss[j].x;
            weiPoint.y+=pointss[j].y;

        }
        weiPoint = Point(weiPoint.x/contours[i].size(),weiPoint.y/contours[i].size());

        if (abs(weiPoint.x - W/2)<50)
        {
//            continue;
        }

        if (contourArea(contours[i])>100)
        {
            drawContours(I, contours, i, Scalar(255), -1, 8, hierarchy, 0);
        }
    }
    threshold(I,I,5,1,0);

    //step 3: add all rows, and cal the numbers range(20 - 300)
    vector<int> rowData;
    for (int i=0;i<W;i++)
    {
        rowData.push_back(0);
    }
    for (int i=0;i<H;i++)
    {
        for (int j=0;j<W;j++)
        {
            int val = I.at<uchar>(i,j);
            if (val >0)
            {
                rowData[j] +=1;
            }
        }
    }
    for (int i=0;i<W;i++)
    {
        if (rowData[i]<20 || rowData[i]>400) 
        {
            rowData[i] = 0;
        }
    }

    // step 4: cal the percent from startPos to endPos
    int count = 0;
    for (int i=startPos;i<endPos;i++)
    {
        if (rowData[i])
        {
            count++;
        }            
    }
    double rate = (1.0*count)/(endPos - startPos);
     cout<<"count= "<<count<<endl;
     cout<<"rate= "<<rate<<endl;
    if (rate >0.4)
    {
        cout<<"rate= "<<rate<<endl;
        return true;
    }
    else
        return false;
}

/*
 * @FUNC
 *     detect the change lane point, delete 30 frames data before change lanes point,
 *            delete 30 frames after after change lanes point,
 * @PARAMS
 *     roi                -  input gray image;
 *     gpsAndInterval     -  input gray image;
 *     currentFrame       -  current frame number;
 *     gps_points         -  GPS data;
 *     inParam            -  config Info;
 *     deleterows          -  the number of rows to delete in current roadScan image; 
 */
void changeLaneProcess(Mat &roi, vector<gpsInformationAndInterval> &GPSAndInterval,int currentFrame,
                        vector<Point2d> &gps_points,Parameters inParam,int &deleterows)
{
    int h = roi.rows;
    int numberframe = 40;
    int row = GPSAndInterval.size();
    int countRows = 0;
    if (row >40)
    {
        for (int i = numberframe-1; i>0;i--)
        {
            countRows += GPSAndInterval[i].intervalOfInterception;
        }
    }
    else
    {
        for (int i = row-1; i>0;i--)
        {
            countRows += GPSAndInterval[i].intervalOfInterception;
        }

    }
    deleterows = countRows;
}
/*
 * @FUNC
 *     delete the area that in middle part of image,such as arrow
 * 
 * @PARAMS
 *     src     -  input gray image;
 *     dst     -  output binary image;
 */
void delMiddlePart(Mat &src,Mat &dst)
{
    int W = src.cols;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    dst  = Mat::zeros(src.size(), CV_8UC1);
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        vector<Point> pointss = contours[i];
        Point2d weiPoint = Point(0,0);//line center of gravity
        for (int j=0;j<pointss.size();j++)//calculate up and down points of contours
        {
            weiPoint.x+=pointss[j].x;
            weiPoint.y+=pointss[j].y;

        }
        weiPoint = Point(weiPoint.x/contours[i].size(),weiPoint.y/contours[i].size());

        if (abs(weiPoint.x - W/2)<50)
        {
            continue;
        }

        if (contourArea(contours[i])>100)
        {
            drawContours(dst, contours, i, Scalar(255), -1, 8, hierarchy, 0);
        }
    }
    threshold(dst,dst,5,1,0);
}



/*
 * @FUNC
 *     curve fitting the points, and delete the points that far from the lines.
 *
 * @PARAMS
 *     longLane     -  input binary image;
 *     fittingImg   -  output curve fitting image.
 */
void polyFitting(Mat &longLane, Mat &fittingImg)
{
    int H = longLane.rows;
    int W = longLane.cols;
    //step 1 : find nonzero points coordinate(x,y),vx,vy.
    vector<int> vx,vy;
    int num = 0;
    for (int i = 0;i<H;i++)
    {
        for (int j = 0;j<W;j++)
        {
            int val = longLane.at<uchar>(i,j);
            if (val>0)
            {
                vx.push_back(j);
                vy.push_back(i);
                num++;
            }
        }
    }
    double *dx = new double [num];
    double *dy = new double [num];
    double *dy2 = new double [num];
    int luo = num*sizeof(double);
    memset(dx,0,num*sizeof(double));
    memset(dy,0,num*sizeof(double));
    memset(dy2,0,num*sizeof(double));
    for (int i = 0;i<num;i++)
    {
        dx[i] = vx[i];
        dy[i] = vy[i];
        dy2[i] = vy[i];
    }
    //step 2: x coordinate normalizatipn, reverse x and y axis;
    Point2d normalization;
    parametersNormalized(dy2, num, normalization);
    //step 3: points curve polyFitting
    double coefficient[21];
    memset(coefficient,0,21*sizeof(double));
    EMatrix(dy2, dx, num, 2, normalization,coefficient);
    //step 4: delete outlier points
    vector<int> val;
    for (int i = 0;i<num;i++)
    {
        double y = dy[i];
        
        double x = calValue(y, coefficient,normalization);
      //  cout<<"y = "<<y<<"   "<<"x = "<<x<<endl;
        double dst = abs(vx[i] - x);
        if (dst>50)
        {
            val.push_back(-1);
            vy[i]  = -1;
        }
        else
        {
            val.push_back(floor(x));
        }
    }
    fittingImg = Mat::zeros(H,W,CV_8UC3);
    for (int i = 0;i<num;i++)
    {
        if (val[i]>=0 && dy[i] >=-1)
        {
            circle(fittingImg,Point(val[i],dy[i]),0,Scalar(0,0,255),1);
        }
    }
    delete dx;
    delete dy;   
    delete dy2;
}


/*
 * @FUNC
 *     link the lines after curve fitting
 * @PARAMS
 *     fittingImage     -  input contous image;
 *     Tline_link_out     -  output paiting linked image;
 */
void linkFittingLine(Mat fittingImage,Mat &Tline_link_out)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(fittingImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    Mat TSline  = Mat::zeros(fittingImage.size(), CV_8UC1);
    Mat TSlinMark = Mat::zeros(fittingImage.size(), CV_8UC3);
    vector<SLineInfo> vecSlineinfo;//save information of every short line

    for (unsigned int i = 0; i < contours.size(); i++)
    {
        SLineInfo sline;
        Vec4f line;
        fitLine(contours[i],line,CV_DIST_L2 ,0,0.01,0.01);//linear fit for short line contour
        sline.vx = line[0];
        sline.vy = line[1];
        double C = (line[3]*line[0]-line[2]*line[1]);
        vector<double> Dist;
        vector<Point> pointss = contours[i];
        sline.upoint = pointss[0];
        sline.dpoint = pointss[pointss.size()-1];
        sline.WeiPoint = Point(0,0);//line center of gravity

        for (int j=0;j<pointss.size();j++)//calculate up and down points of contours
        {
            if (pointss[j].y<sline.upoint.y)
            {
                sline.upoint = pointss[j];
            }
            if (pointss[j].y>sline.dpoint.y)
            {
                sline.dpoint = pointss[j];
            }
            sline.WeiPoint.x+=pointss[j].x;
            sline.WeiPoint.y+=pointss[j].y;

        }
        sline.WeiPoint = Point(sline.WeiPoint.x/contours[i].size(),sline.WeiPoint.y/contours[i].size());        
        drawContours(TSline, contours, i, Scalar(255), 1, 8, hierarchy, 0);
        drawContours(TSlinMark, contours, i, Scalar(255,255,255), 1, 8, hierarchy, 0);
        circle(TSlinMark,sline.WeiPoint,0,Scalar(255,0,0),4);
        circle(TSlinMark,sline.upoint,0,Scalar(0,255,0),4);
        circle(TSlinMark,sline.dpoint,0,Scalar(0,0,255),4);
        vecSlineinfo.push_back(sline);
    }
#ifdef DEBUG_ROAD_SCAN
    imwrite( "TSlinMark.png", TSlinMark );
#endif

    /////one up point only links to one down points
    vector<PairPoints> vecPairPoints;//save up and down point pair;
    int *objptr = new int[vecSlineinfo.size()];
    int *srcptr = new int[vecSlineinfo.size()];
    for(int i=0;i<vecSlineinfo.size();i++)
    {
        objptr[i]=0;
        srcptr[i]=0;
    }

    for(int i = 0;i<vecSlineinfo.size();i++)
    {
        ///	int ddddd = vecSlineinfo[i].WeiPoint.y;
        for (int j = i+1;j<vecSlineinfo.size();j++)
        {			
            //double dsty = vecSlineinfo[i].WeiPoint.y-vecSlineinfo[j].WeiPoint.y;
            double dsty = vecSlineinfo[i].upoint.y-vecSlineinfo[j].dpoint.y;
            double dstx = vecSlineinfo[i].upoint.x-vecSlineinfo[j].dpoint.x;
            Point2d diffw = Point2d(vecSlineinfo[i].WeiPoint.x-vecSlineinfo[j].WeiPoint.x,
                vecSlineinfo[i].WeiPoint.y-vecSlineinfo[j].WeiPoint.y);
            double cosvalue = abs(vecSlineinfo[i].vx*vecSlineinfo[j].vx+vecSlineinfo[i].vy*vecSlineinfo[j].vy);

            double cosvalue2 = abs(dstx*vecSlineinfo[j].vx+dsty*vecSlineinfo[j].vy)/
                sqrt((dstx*dstx+dsty*dsty)*(vecSlineinfo[j].vx*vecSlineinfo[j].vx+vecSlineinfo[j].vy*vecSlineinfo[j].vy));

            double cosvalue3 = abs(dstx*vecSlineinfo[j].vx+dsty*vecSlineinfo[j].vy)/
                sqrt((dstx*dstx+dsty*dsty)*(vecSlineinfo[j].vx*vecSlineinfo[j].vx+vecSlineinfo[j].vy*vecSlineinfo[j].vy));
        
            double tanangle = atan(abs(diffw.y/(diffw.x+0.0000000001)))*180/PI;

            //if (abs(dsty)>1500||tanangle<75||cosvalue<0.8||abs(dstx)>20||dsty<0)//Threshold condition
            if (abs(dsty)>2000||cosvalue3<0.8||cosvalue2<0.8||cosvalue<0.8||dsty<0||abs(dstx)>fittingImage.cols/3)
            {
                continue;
            }
            objptr[j] +=1; 
            srcptr[i] +=1;

            PairPoints str_pairpoints;
            str_pairpoints.up = vecSlineinfo[i].upoint;
            str_pairpoints.down = vecSlineinfo[j].dpoint;
            str_pairpoints.objindex =j;//object line index；
            str_pairpoints.srcindex =i;//source line index；
            circle(TSlinMark,vecSlineinfo[i].WeiPoint,0,Scalar(255,0,0),4);
            circle(TSlinMark,vecSlineinfo[i].upoint,0,Scalar(0,255,0),4);
            circle(TSlinMark,vecSlineinfo[i].dpoint,0,Scalar(0,0,255),4);
            vecPairPoints.push_back(str_pairpoints);
        }
    }
#ifdef DEBUG_ROAD_SCAN
    imwrite("TSlinMark.png",TSlinMark);
#endif
    //if more than one up points link to the same down point
    for (int i = 0;i<vecSlineinfo.size();i++)
    {
        if (objptr[i]>1)
        {
            int num = objptr[i];
            Point *TemUPoints = new Point[num];
            Point TemDPoints;
            int n=0;
            int *index = new int[num];
            for (int i1 = 0;i1<vecPairPoints.size();i1++)
            {
                if (vecPairPoints[i1].objindex==i)
                {
                    TemUPoints[n] = vecPairPoints[i1].up;
                    index[n] = i1;
                    n++;
                    TemDPoints = vecPairPoints[i1].down;						
                }
            }
            int valuemin = abs(TemUPoints[0].y-TemDPoints.y);
            int reindext  = 0;
            reindext = index[0];
            int tempreindext = index[0];

            for (int i2=1;i2<n;i2++)
            {	
                double dx = vecPairPoints[index[i2]].up.x-vecPairPoints[index[i2]].down.x;
                double dy = vecPairPoints[index[i2]].up.y-vecPairPoints[index[i2]].down.y;
                double dxx = dx/sqrt(dx*dx+dy*dy);
                double dyy = dy/sqrt(dx*dx+dy*dy);
                double cosvalue0 = vecSlineinfo[i].vx*dxx + vecSlineinfo[i].vy*dyy;
                double cosvalue = abs(vecSlineinfo[i].vx*dx + vecSlineinfo[i].vy*dy)/
                    sqrt((dx*dx+dy*dy)*(vecSlineinfo[i].vx*vecSlineinfo[i].vx+vecSlineinfo[i].vy*vecSlineinfo[i].vy));
                double tanangle = atan(abs(dy/(dx+0.0000000001)))*180/PI;
                double cosangle = acos(cosvalue)*180/PI;
                if (cosvalue<0.8)
                {
                    reindext = index[i2];
                    vecPairPoints[reindext].up=Point(0,0);
                    vecPairPoints[reindext].down=Point(0,0);
                    continue;
                }
                if (valuemin<abs(TemUPoints[i2].y-TemDPoints.y))
                {
                    reindext = index[i2];
                    vecPairPoints[reindext].up=Point(0,0);
                    vecPairPoints[reindext].down=Point(0,0);					
                }
                else
                {
                    valuemin=abs(TemUPoints[i2].y-TemDPoints.y);						
                    vecPairPoints[tempreindext].up=Point(0,0);
                    vecPairPoints[tempreindext].down=Point(0,0);
                    tempreindext = index[i2];
                    reindext = index[i2];
                }
            }
            delete TemUPoints;
            delete index;
        }

    }
    //if one up point links to more than one down points
    for (int i = 0;i<vecSlineinfo.size();i++)
    {
        if (srcptr[i]>1)
        {
            int num = srcptr[i];
            Point *TemDPoints = new Point[num];
            Point TemUPoints;
            int n=0;
            int *index = new int[num];
            for (int i1 = 0;i1<vecPairPoints.size();i1++)
            {

                if (vecPairPoints[i1].srcindex==i)
                {
                    if (vecPairPoints[i1].up.x)
                    {
                        TemDPoints[n] = vecPairPoints[i1].down;
                        index[n] = i1;
                        n++;
                        TemUPoints = vecPairPoints[i1].up;		
                    }										
                }
            }
            int valuemin = abs(TemDPoints[0].y-TemUPoints.y);
            int reindext  = 0;
            reindext = index[0];
            int tempreindext = index[0];
            Point minvalupoint,minvaldpoint;

            for (int i2=1;i2<n;i2++)
            {					
                if (valuemin<abs(TemDPoints[i2].y-TemUPoints.y))
                {
                    reindext = index[i2];
                    vecPairPoints[reindext].up=Point(0,0);
                    vecPairPoints[reindext].down=Point(0,0);					
                }
                else
                {
                    valuemin=abs(TemDPoints[i2].y-TemUPoints.y);						
                    vecPairPoints[tempreindext].up=Point(0,0);
                    vecPairPoints[tempreindext].down=Point(0,0);
                    tempreindext = index[i2];
                    reindext = index[i2];
                }
            }
            delete TemDPoints;
            delete index;
        }
    }
    delete objptr;
    delete srcptr;
    for (int i = 0;i<vecPairPoints.size();i++)
    {
        int dx = abs(vecPairPoints[i].up.x - vecPairPoints[i].down.x);
        if (vecPairPoints[i].up!=Point(0,0)&&(vecPairPoints[i].up.y>vecPairPoints[i].down.y)&&dx<150)
        {
            line(TSline,vecPairPoints[i].up,vecPairPoints[i].down,Scalar(255,255,0),1);
        }
    }
    Tline_link_out = Mat::zeros(fittingImage.size(), CV_8UC1);
    threshold( TSline, TSline, 20, 255,0 );
#ifdef DEBUG_ROAD_SCAN
    imwrite("TSline.png",TSline);
#endif
    vector<vector<Point>> contours4;
    vector<Vec4i> hierarchy4;
    findContours(TSline, contours4, hierarchy4, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    for (unsigned int i = 0; i < contours4.size(); i++)
    {
        vector<Point> pointss = contours4[i];
        Point upoint = pointss[0];
        Point dpoint = pointss[pointss.size()-1];
        for (int j=0;j<pointss.size();j++)
        {
            if (pointss[j].y<upoint.y)
            {
                upoint = pointss[j];
            }
            if (pointss[j].y>dpoint.y)
            {
                dpoint = pointss[j];
            }
        }
        int dst = abs(dpoint.y - upoint.y);
        if (abs(upoint.x-dpoint.x)<50&&abs((upoint.x+dpoint.x)/2-Tline_link_out.cols/2)<50)
        {
            continue;
        }
        if (dst>600)//delete the hight less than 300
        {	
            drawContours(Tline_link_out, contours4, i, Scalar(255), 1, 8, hierarchy4, 0);
        }
    }

#ifdef DEBUG_ROAD_SCAN
    imwrite("Tline_link_out.png",Tline_link_out);
#endif

}
/*
 * @FUNC
 *     translate RGB to Gray, max(R,G.B) = Gray/per pixel;
 * @PARAMS
 *     longLaneRGB     -  input RGB image;
 *     longLaneGray    -  output gray image;
 */
void cvtRGB2Gray(Mat &longLaneRGB, Mat &longLaneGray)
{
    Mat RI,GI,BI; //R,G,B channel image
    int H = longLaneRGB.rows;
    int W = longLaneRGB.cols;
    longLaneGray = Mat::zeros(H,W,CV_8UC1);
    for (int i=0;i<H;i++)
    {
        for (int j=0;j<W;j++ )
        {
            int r,g,b,v;
            b = longLaneRGB.at<Vec3b>(i,j)[0];
            g = longLaneRGB.at<Vec3b>(i,j)[1];
            r = longLaneRGB.at<Vec3b>(i,j)[2];
            v = max(max(b,g),r);
            longLaneGray.at<uchar>(i,j) = v;
        }
    }
}

void resetGPSOffset(double offsetDistance, Point2d &GPS_1, Point2d &GPS_2)
{
    Point2d gpsVector = Point2d(GPS_2.x - GPS_1.x, GPS_2.y - GPS_1.y);

    double distanceGPS = sqrt(pow(gpsVector.x, 2.0) + pow(gpsVector.y, 2.0));

    //unit vector
    gpsVector.x /= distanceGPS;
    gpsVector.y /= distanceGPS;

    //offset
    gpsVector.x *= offsetDistance;
    gpsVector.y *= offsetDistance;

    //reset the input GPS
    GPS_1.x += gpsVector.x;
    GPS_2.x += gpsVector.x;

    GPS_1.y += gpsVector.y;
    GPS_2.y += gpsVector.y;
}


}
