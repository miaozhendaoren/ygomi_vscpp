/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  dbtools.h
* @brief Source file for database, common definitions for vehichle and server
*
* Change Log:
*      Date                       Who                   What
*      2015/7/10           jiaolong liu/lin deng       Create
*******************************************************************************
*/

#pragma once


#include <vector>
#include "Point3D.h"
#include "Line.h"
#include "Sign.h"
#include "Paint.h"
#include "Lane.h"
//#include "mysql.h"

using namespace ns_database;

namespace ns_dbmaster
{
	class DBTools
	{
	public:
        static DBTools* getInstance(void) {return m_instance;}

		bool getAllPoints(OUT std::vector<Point3D> &) {return false;}
		int insertPoint(INOUT Point3D& point) {return -1;}
		bool insertPoints(OUT std::vector<Point3D> &) {return false;}
		//bool deletePointById(int Id);
		bool deleteAllPoint() {return false;}//truncate table

		bool getAllLines(OUT std::vector<Line> &) {return false;}
		bool getLineById(int id, OUT Line &) {return false;}
		bool getLineByGps(Point3D &,OUT std::vector<Line> &) {return false;}
		bool insertLine(Line &) {return false;}
		bool insertLines(std::vector<Line> &) {return false;}
		//bool deleteLineById(int id);
		bool deleteAllLine() {return false;}

		bool getAllSigns(OUT std::vector<Sign> &) {return false;}
		bool getSignById(int id, OUT Sign &) {return false;}
		bool getSignByGps(Point3D &) {return false;}//from_point
		int insertSign(Sign &) {return -1;}
		bool insertSigns(std::vector<Sign> &) {return false;}
		bool deleteSignById(int id) {return false;}
		bool deleteAllSigns() {return false;}

		bool getAllPaints(std::vector<Paint> & ) {return false;}
		bool getPaintById(int id, OUT Paint &) {return false;}
		int insertPaint(Paint &) {return false;}
		bool insertPaints(std::vector<Paint> &) {return false;}
		bool deletePaintById(int id) {return false;}
		bool deleteAllPaints() {return false;}

		bool getAllLanes(OUT std::vector<Lane> &) {return false;}
		bool getLaneById(int id, OUT Lane &) {return false;}
		bool getNextlane (int id, std::vector<Lane> &) {return false;}
		bool getAdjlane(int id, std::vector<Lane> &) {return false;}
		int insertLane(Lane &) {return -1;}
		bool insertLanes(std::vector<Lane> &) {return false;}
		bool deleteLaneById(int id) {return false;}
		bool deleteAllLanes() {return false;}

	private:
        DBTools(void) {}
        static bool initServerDB() {return false;}

    private:
        static DBTools* m_instance;
        //static MYSQL myCont;
	};

}
