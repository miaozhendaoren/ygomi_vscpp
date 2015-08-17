/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  polynomialFit.h
* @brief Header file for polynomial curve fitting function. Given a series of
*        x and y at same number, get a polynomial relationship of x and y.
*            y = coef[1] * x^0 + coef[2] * x^1 + coef[3] * x^2 + ...
*
*        The maximum of fit times is 19, which is ... + coef[20] * x^19
*
* Change Log:
*      Date                Who             What
*      2015/08/17       Feng Liang        Create
*******************************************************************************
*/


#ifndef _POLYNOMIALFIT_H
#define _POLYNOMIALFIT_H

#include <iostream>
#include <vector>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;


#define MAX_DEGREE            19
#define FIT_VARIANCE 0.000000001


//polynomial curve fitting functions
double sum(double *dNumarry, int n);
double MutilSum(double *dX, double *dY, int n);
double RelatePow(double *dx, int n, int ex);
double RelateMutiXY(double *dx, double *dy, int n, int ex);
double EMatrix(double *dx, double *dy, int n, int ex, Point2d normalization, double coefficient[]);
void CalEquation(int exp, double coefficient[]);
double F(double c[], int l, int m);

double calMeanSquareError(double *dx, double *dy, int n, double coefficient[], Point2d normalization);
double calValue(double x, double coefficient[], Point2d normalization);

void parametersNormalized(double *dx, int number, Point2d &normalization);

#endif
