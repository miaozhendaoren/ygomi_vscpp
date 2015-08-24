/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  polynomialFit.cpp
* @brief Implementation of polynomial curve fitting functions. Given a series of
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


#include "polynomialFit.h"

double Em[MAX_DEGREE + 1][MAX_DEGREE + 1];

double sum(double *dNumarry, int n)
{
    double dSum = 0;

    if (nullptr != dNumarry && 0 < n)
    {
        for(int i = 0; i < n; i++)
        {
            dSum += dNumarry[i];
        }
    }

    return dSum;
}

double MutilSum(double *dX, double *dY, int n)
{
    double dMultiSum = 0;

    if (nullptr != dX && nullptr != dY && 0 < n)
    {
        for(int i = 0; i < n; i++)
        {
            dMultiSum += dX[i] * dY[i];
        }
    }

    return dMultiSum;
}

double RelatePow(double *dx, int n, int ex)
{
    double ReSum = 0;

    if (nullptr != dx && 0 < n)
    {
        for(int j = 0; j < n; j++)
        {
            ReSum += pow(dx[j], ex);
        }
    }

    return ReSum;
}

double RelateMutiXY(double *dx, double *dy, int n, int ex)
{
    double dReMultiSum = 0;

    if (nullptr != dx && nullptr != dy && 0 < n)
    {
        for(int i = 0; i < n; i++)
        {
            dReMultiSum += pow(dx[i], ex) * dy[i];
        }
    }

    return dReMultiSum;
}

double EMatrix(double *dx, double *dy, int n, int ex, Point2d normalization, double coefficient[])
{
    double meanSquareError = 0.0;

    if (nullptr != dx && nullptr != dy && 0 < n)
    {
        for(int i = 1; i <= ex; i++)
        {
            for(int j = 1; j <= ex; j++)
            {
                Em[i][j] = RelatePow(dx, n, i + j - 2);
            }

            Em[i][ex + 1] = RelateMutiXY(dx, dy, n, i - 1);
        }
        Em[1][1] = n;

        CalEquation(ex, coefficient);

        meanSquareError = calMeanSquareError(dx, dy, n, coefficient, normalization);
    }

    return meanSquareError;
}

void CalEquation(int exp, double coefficient[])
{
    if (nullptr != coefficient)
    {
        for(int k = 1; k < exp; k++)
        {
            for(int i = (k + 1); i < (exp + 1); i++)
            {
                double p1 = 0;

                if(Em[k][k] != 0)
                {
                    p1 = Em[i][k] / Em[k][k];
                }

                for(int j = k; j < (exp + 2); j++)
                {
                    Em[i][j] = Em[i][j] - Em[k][j]*p1;
                }
            }
        }

        coefficient[exp] = Em[exp][exp + 1] / Em[exp][exp];

        for(int i = (exp - 1); i >= 1; i--)
        {
            coefficient[i] = (Em[i][exp + 1] - F(coefficient, (i + 1), exp)) / Em[i][i];
        }
    }
}

double F(double c[], int l, int m)
{
    double sum = 0;

    if (nullptr != c)
    {
        for(int i = l; i <= m; i++)
        {
            sum += Em[l - 1][i] * c[i];
        }
    }

    return sum;
}

double calMeanSquareError(double *dx, double *dy, int n, double coefficient[], Point2d normalization)
{
    double meanSquareError = 0.0;

    if (nullptr != dx && nullptr != dy && 0 < n && nullptr != coefficient)
    {
        for(int index = 0; index < n; index++)
        {
            double y = coefficient[ 1] * pow(dx[index], 0.0) +
                       coefficient[ 2] * pow(dx[index], 1.0) +
                       coefficient[ 3] * pow(dx[index], 2.0) +
                       coefficient[ 4] * pow(dx[index], 3.0) +
                       coefficient[ 5] * pow(dx[index], 4.0) +
                       coefficient[ 6] * pow(dx[index], 5.0) +
                       coefficient[ 7] * pow(dx[index], 6.0) +
                       coefficient[ 8] * pow(dx[index], 7.0) +
                       coefficient[ 9] * pow(dx[index], 8.0) +
                       coefficient[10] * pow(dx[index], 9.0) +
                       coefficient[11] * pow(dx[index], 10.0) +
                       coefficient[12] * pow(dx[index], 11.0) +
                       coefficient[13] * pow(dx[index], 12.0) +
                       coefficient[14] * pow(dx[index], 13.0) +
                       coefficient[15] * pow(dx[index], 14.0) +
                       coefficient[16] * pow(dx[index], 15.0) +
                       coefficient[17] * pow(dx[index], 16.0) +
                       coefficient[18] * pow(dx[index], 17.0) +
                       coefficient[19] * pow(dx[index], 18.0) +
                       coefficient[20] * pow(dx[index], 19.0);

            meanSquareError += pow(y - dy[index], 2.0);
        }

        if(n == 1)
        {
            meanSquareError = 0.0;
        }
        else
        {
            meanSquareError = sqrt(meanSquareError / (n - 1));
        }
    }

    return meanSquareError;
}

double calValue(double x, double coefficient[], Point2d normalization)
{
    double y = 0.0;

    x = (x - normalization.x) / normalization.y;

    if (nullptr != coefficient)
    {
        y = coefficient[ 1] * pow(x, 0.0) +
            coefficient[ 2] * pow(x, 1.0) +
            coefficient[ 3] * pow(x, 2.0) +
            coefficient[ 4] * pow(x, 3.0) +
            coefficient[ 5] * pow(x, 4.0) +
            coefficient[ 6] * pow(x, 5.0) +
            coefficient[ 7] * pow(x, 6.0) +
            coefficient[ 8] * pow(x, 7.0) +
            coefficient[ 9] * pow(x, 8.0) +
            coefficient[10] * pow(x, 9.0) +
            coefficient[11] * pow(x, 10.0) +
            coefficient[12] * pow(x, 11.0) +
            coefficient[13] * pow(x, 12.0) +
            coefficient[14] * pow(x, 13.0) +
            coefficient[15] * pow(x, 14.0) +
            coefficient[16] * pow(x, 15.0) +
            coefficient[17] * pow(x, 16.0) +
            coefficient[18] * pow(x, 17.0) +
            coefficient[19] * pow(x, 18.0) +
            coefficient[20] * pow(x, 19.0);
    }

    return y;
}

void parametersNormalized(double *dx, int number, Point2d &normalization)
{
    if (nullptr != dx)
    {
        //mean
        double sumDx = 0.0;

        for(int index = 0; index<number; index++)
        {
            sumDx += dx[index];
        }

        normalization.x = sumDx / number;

        //std
        double sumStd = 0.0;
        for(int index = 0; index < number; index++)
        {
            sumStd += pow(dx[index] - normalization.x, 2.0);
        }

        if(number == 1)
        {
            normalization.y = 0;
        }
        else
        {
            normalization.y = sqrt(sumStd / (number - 1));
        }

        for(int index = 0; index < number; index++)
        {
            dx[index] = (dx[index] - normalization.x) / (normalization.y + FIT_VARIANCE);
        }
    }

    return;
}