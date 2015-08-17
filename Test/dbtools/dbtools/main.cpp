/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  main.cpp
* @brief Source file for database, common definitions for vehichle and server
*
* Change Log:
*      Date                       Who                   What
*      2015/7/10           jiaolong liu/lin deng       Create
*******************************************************************************
*/

#pragma once

#include <Windows.h>
#include <vector>
#include <iostream>
#include<conio.h> 
#include "dbtools.h"

using namespace ns_dbmaster;
using std::cout;
using std::endl;
using std::vector;

void test_getAllPoints(DBTools* dbtest)
{
	vector<Point3D> vPoint;
	int count=0;
	//test getAllPoints
	if(dbtest->getAllPoints(vPoint))
	{
		vector<Point3D>::iterator vPointIter =  vPoint.begin();
		while(vPointIter != vPoint.end())
		{
			count++;
			cout.precision(10);
			cout<<vPointIter->getLatitude()<<",,,"<<count<<",,,"<<vPointIter->getLongitude()<<endl;
			vPointIter++;
		}
	}
}
void test_insertPoint(DBTools* dbtest)
{
	Point3D pTemp;
	pTemp.setLatitude(48.39974539);
	pTemp.setLongitude(11.72941915);
	pTemp.setAltitude(0);
	int id = dbtest->insertPoint(pTemp);
	cout<<"point id: "<<id<<endl;

}

void test_getIdByPoint(DBTools* dbtest)
{
	Point3D pTemp;
	pTemp.setLatitude(48.39974539);
	pTemp.setLongitude(11.72941915);
	pTemp.setAltitude(0);
	int id = dbtest->getIdByPoint(pTemp);
	cout<<"point id: "<<id<<endl;
}
void test_getPointById(DBTools* dbtest)
{
	Point3D pTemp;
	int id = 2;
	dbtest->getPointById(id,pTemp);
	cout.precision(9);
	cout<<pTemp.getId()<<","<<pTemp.getLatitude()<<","<<pTemp.getLongitude()<<","<<pTemp.getAltitude()<<endl;
}
void test_getAllLines(DBTools* dbtest)
{
	vector<Line> vLineTemp;
	dbtest->getAllLines(vLineTemp);
	vector<Line>::iterator vLineIter = vLineTemp.begin();
	while(vLineIter != vLineTemp.end())
	{
		cout<<vLineIter->getId()<<",";
		vector<Point3D> vPoint = vLineIter->getPoints();
		vector<Point3D>::iterator vPointIter =  vPoint.begin();
		while(vPointIter != vPoint.end())
		{
			cout.precision(9);
			cout<<vPointIter->getLatitude()<<","<<vPointIter->getLongitude()<<endl;
			vPointIter++;
		}
		vLineIter++;
	}
}
void test_insertLine(DBTools* dbtest)
{
	vector<Point3D> vPointTemp;
	Point3D pTemp;
	
	dbtest->getPointById(635,pTemp);
	vPointTemp.push_back(pTemp);
	dbtest->getPointById(634,pTemp);
	vPointTemp.push_back(pTemp);
	dbtest->getPointById(633,pTemp);
	vPointTemp.push_back(pTemp);
	dbtest->getPointById(632,pTemp);
	vPointTemp.push_back(pTemp);
	dbtest->getPointById(631,pTemp);
	vPointTemp.push_back(pTemp);

	Line* lineTemp = new Line(vPointTemp);
	int id = dbtest->insertLine(*lineTemp);
	cout<<"line id: "<<id<<endl;
}

void test_insertPaint(DBTools* dbtest)
{
	Line lineTemp;
	dbtest->getLineById(7,lineTemp);
	SurfaceBoundary* sfBoundTemp = new SurfaceBoundary(false,false,lineTemp);
	vector<SurfaceBoundary> vsfTemp;
	vsfTemp.push_back(*sfBoundTemp);
	Surface* sfTemp = new Surface(vsfTemp);
	Color_t cc;
	cc.b = 1;
	cc.g = 1;
	cc.r = 0;
	Point3D landmark;
	dbtest->getPointById(601,landmark);
	Paint* paintTemp = new Paint(landmark,cc,*sfTemp);

	dbtest->insertPaint(*paintTemp);
}

int main(int argc, char* argv[])
{
	DBTools* dbtest = DBTools::getInstance();
	
	//test_getAllPoints(dbtest);
	//test_insertPoint(dbtest);
	//test_getIdByPoint(dbtest);
	//test_getPointById(dbtest);
	//test_getAllLines(dbtest);
	//test_insertLine(dbtest);
	test_insertPaint(dbtest);

	//dbtest->importdatafromNewcoToMaster();

	cout<<"Game Over.";
	getch();

	
}