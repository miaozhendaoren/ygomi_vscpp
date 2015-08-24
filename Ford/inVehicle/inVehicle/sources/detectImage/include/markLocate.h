#ifndef _MARKLOCATE_H
#define _MARKLOCATE_H

//#include <iostream>

//#include <iomanip>

//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/nonfree/features2d.hpp"
//#include <opencv2/legacy/legacy.hpp>
#include <opencv2/opencv.hpp>

#include "database.h"
#include "roadScan.h"

using namespace ns_roadScan;

using namespace std;
using namespace cv;

#define COEFF_DD2METER (111320.0)
#define PI (3.14159265359)

void determineBirdViewLocation(Mat &invertH, Point &pixelLocationOriginalImage, Point &pixelLocationBirdView);

void getRefGPSLocationOfEveryPixelInRoadScanImage(Mat &imageIn, int stretchRate, Point2d GPS_current, Point2d GPS_next, Point2d GPS_reference, Point pixelLocationBirdView, double distancePerPixel, Point2d &refGPSOriginalImage);

void coordinateChange(Point2d in, Point2d ref, Point2d &out);

void calActualGPSFromRef(Point2d location, Point2d referenceGPS, ns_database::point3D_t &actualGPS);

#endif