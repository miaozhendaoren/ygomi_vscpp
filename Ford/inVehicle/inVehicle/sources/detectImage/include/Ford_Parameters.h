#ifndef Ford_Parameters_H_
#define Ford_Parameters_H_

//original GPS location
//different for every video

//if VW video, the image size should be resize

//center point for view bird image
const int centerPoint[2] = {319,239};

//lengthRate - initial horizontal and vertical distance per pixel rate
//const double lengthRate = 12.875;
const double lengthRate = 11.7647;
//stretchRate - stretch rate for view bird
const int stretchRate = 2;

//initial point of GPS
//const long double initialPointOfGPS[2] = {42.296853333333331,-83.213250000000002};

//distance per pixel (cm) in horizontal direction
const double distancePerPixel = 0.85;

#endif