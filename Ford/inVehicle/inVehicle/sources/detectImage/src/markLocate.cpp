
#include "markLocate.h"
#include "configure.h"
#include "utils.h"
void determineBirdViewLocation(Mat &invertH, Point &pixelLocationOriginalImage, Point &pixelLocationBirdView)
{
    double invertHArray[3][3];
	for (int index1 = 0; index1 < 3; index1++)
	{
		for (int index2 = 0; index2 < 3; index2++)
		{
			double middleValue = invertH.at<double>(index2, index1);
			invertHArray[index1][index2] = middleValue;
		}
	}

	double xx = pixelLocationOriginalImage.x * invertHArray[0][0] + pixelLocationOriginalImage.y * invertHArray[1][0] + invertHArray[2][0];
	double yy = pixelLocationOriginalImage.x * invertHArray[0][1] + pixelLocationOriginalImage.y * invertHArray[1][1] + invertHArray[2][1];
	double ww = pixelLocationOriginalImage.x * invertHArray[0][2] + pixelLocationOriginalImage.y * invertHArray[1][2] + invertHArray[2][2];

	pixelLocationBirdView = Point(int((xx+0.5)/ww), int((yy+0.5)/ww));
}

void getRefGPSLocationOfEveryPixelInRoadScanImage(Mat &imageIn, int stretchRate, Point2d GPS_current, Point2d GPS_next, Point2d GPS_reference, Point pixelLocationBirdView, double distancePerPixel, Point2d &refGPSOriginalImage)
{

	Point2d GPS1, GPS2;
	coordinateChange(GPS_current, GPS_reference, GPS1);
	coordinateChange(GPS_next, GPS_reference, GPS2);

	refGPSOriginalImage = Point2d(-1.0, -1.0);

	Point2d pixel = Point2d((imageIn.cols/2 - pixelLocationBirdView.x)*(1.0), imageIn.rows*stretchRate - pixelLocationBirdView.y);

	int scopeOfScanImage = ceil(sqrt(pow(GPS1.x - GPS2.x, 2.0) + pow(GPS1.y - GPS2.y, 2.0))*100/distancePerPixel);

	//GPS angle
	double angle = atan((GPS2.y-GPS1.y)/(GPS2.x-GPS1.x));

	double threshold = 0.00000000000001;

	if((abs(GPS2.x-GPS1.x)>=threshold)&&(abs(GPS2.y-GPS1.y)>=threshold))
	{
		if(pixel.x>=0)
		{
			//the first quadrant
			refGPSOriginalImage.x = (pixel.x*tan(angle)+pixel.y)/scopeOfScanImage*(GPS2.x-GPS1.x)+GPS1.x;

			double middleValue = pixel.y-tan(PI/2-angle)*pixel.x;

			if(middleValue>=0)
			{
				refGPSOriginalImage.y = (pixel.y-pixel.x/tan(angle))/scopeOfScanImage*(GPS2.y-GPS1.y)+GPS1.y;
			}
			else
			{
				refGPSOriginalImage.y = GPS1.y-(pixel.x/tan(angle)-pixel.y)/scopeOfScanImage*(GPS2.y-GPS1.y);
			}
		}
		else
		{
			//the second quadrant
			double middleValue = pixel.y-tan(PI-angle)*pixel.x;

			if(middleValue>=0)
			{
				refGPSOriginalImage.x = (pixel.y+pixel.x*tan(angle))/scopeOfScanImage*(GPS2.x-GPS1.x)+GPS1.x;
			}
			else
			{
				refGPSOriginalImage.x = GPS1.x+(pixel.x/tan(PI/2-angle)+pixel.y)/scopeOfScanImage*(GPS2.x-GPS1.x);
			}

			refGPSOriginalImage.y = (pixel.y-pixel.x/tan(angle))/scopeOfScanImage*(GPS2.y-GPS1.y)+GPS1.y;

		}
	}
	else if((abs(GPS2.x-GPS1.x)>=threshold)&&(abs(GPS2.y-GPS1.y)<threshold))
	{
		// horizontal
		refGPSOriginalImage.x = pixel.y/scopeOfScanImage*(GPS2.x-GPS1.x)+GPS1.x;

		if(GPS2.x>GPS1.x)
		{
			refGPSOriginalImage.y = GPS1.y-pixel.x*distancePerPixel/100;
		}
		else
		{
			refGPSOriginalImage.y = GPS1.y+pixel.x*distancePerPixel/100;
		}
	}
	else if((abs(GPS2.x-GPS1.x)<threshold)&&(abs(GPS2.y-GPS1.y)>=threshold))
	{
		//vertical
		refGPSOriginalImage.y = pixel.y/scopeOfScanImage*(GPS2.y-GPS1.y)+GPS1.y;

		if(GPS2.y>GPS1.y)
		{
			refGPSOriginalImage.x = GPS1.x+pixel.x*distancePerPixel/100;
		}
		else
		{
			refGPSOriginalImage.x = GPS1.x-pixel.x*distancePerPixel/100;
		}
	}
	else
	{
		refGPSOriginalImage.x = 0.0;
		refGPSOriginalImage.y = 0.0;
	}
}
void calActualGPSFromRef(Point2d location, Point2d referenceGPS, ns_database::point3D_t &actualGPS)
{
    actualGPS.lat = location.x/COEFF_DD2METER + referenceGPS.x;

	if(referenceGPS.x != 90)
	{
		double latitude = (referenceGPS.x)*PI/180;
        actualGPS.lon = location.y/(111413*cos(latitude) - 94*cos(3*latitude)) + referenceGPS.y;
	}
}
