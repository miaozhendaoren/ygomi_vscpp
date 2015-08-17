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
#include "mysql.h"

using namespace ns_database;

namespace ns_dbmaster
{
    
	class DBTools
	{
	public:
		static DBTools* getInstance(void);

		bool getAllPoints(OUT std::vector<Point3D> &);
		int getIdByPoint(IN Point3D& point);
		bool getPointById(IN int id,OUT Point3D &);
		int insertPoint(INOUT Point3D& point);
		bool insertPoints(INOUT std::vector<Point3D> &);
		
		bool getAllLines(OUT std::vector<Line> &);
		bool getLineById(IN int id, OUT Line &);
		int  getIdByLine(IN Line &);
		bool getLineByGps(IN Point3D &,OUT std::vector<Line> &);
		int insertLine(INOUT Line &);
		bool insertLines(INOUT std::vector<Line> &);
		
		bool getAllSigns(OUT std::vector<Sign> &);
		bool getSignByGpsRadius(IN Point3D & , IN int radius , OUT std::vector<Sign> &);
		bool getSignByGps(IN Point3D &,OUT Sign &);//from_point
		int insertSign(INOUT Sign &);
		bool insertSigns(IN std::vector<Sign> &);
		bool deleteSignById(IN int id);

		int insertSurface(INOUT Surface &);

		bool getAllPaints(std::vector<Paint> & );
		bool getPaintById(int id, OUT Paint &);
		int insertPaint(INOUT Paint &);
		bool insertPaints(std::vector<Paint> &);
		bool deletePaintById(int id);

		bool getAllLanes(OUT std::vector<Lane> &);
		bool getLaneById(int id, OUT Lane &);
		bool getNextlane (int id, std::vector<Lane> &);
		bool getAdjlane(int id, std::vector<Lane> &);
		int insertLane(Lane &);
		bool insertLanes(std::vector<Lane> &);
		bool deleteLaneById(int id);

		bool deleteAllTable(std::string tablename);
		int floatToInt(float var);
		float intToFloat(int var);

		//test
		void importdatafromNewcoToMaster();

	private:
		static DBTools* m_instance;
		DBTools(void);
		static bool initServerDB();
		static MYSQL myCont;
		//static double precision;
	};

}
