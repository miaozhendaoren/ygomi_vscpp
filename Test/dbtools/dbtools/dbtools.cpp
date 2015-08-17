/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  dbtools.cpp
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
#include <sstream>
#include "Point3D.h"
#include "Line.h"
#include "Sign.h"
#include "Paint.h"
#include "Lane.h"
#include "mysql.h"
#include "dbtools.h"

using namespace ns_database;

namespace ns_dbmaster
{
	MYSQL DBTools::myCont;
	DBTools* DBTools::m_instance = NULL;
	//double DBTools::precision = 0.0000001;

	DBTools::DBTools(void)
	{
		
		initServerDB();
		
	}
	DBTools* DBTools::getInstance(void)
	{
		if(NULL == m_instance)
		{
			//Lock();
			if(NULL == m_instance)
	 	    {
				m_instance = new DBTools();
			}
			//UnLock();
		}
		return m_instance;
	}
	bool DBTools::initServerDB()
	{
		//connect mysql
		const char user[] = "server";
		const char pswd[] = "server"; 
		//const char host[] = "localhost"; 
		const char host[] = "10.69.2.116";
		const char table[] = "masterdb";
		unsigned int port = 3306;

		mysql_init(&myCont);

		if(mysql_real_connect(&myCont,host,user,pswd,table,port,NULL,0))
		{
			std::cout<<"connect succeed for MySQL."<<std::endl;
			//logPrintf(logLevelNotice_e, "DATABASE", "connect succeed for sql.", FOREGROUND_RED);
			return true;
		}
		else
		{
			std::cout<<"connect failed for MySQL."<<std::endl;
			//logPrintf(logLevelError_e, "DATABASE", "connect failed for sql.", FOREGROUND_RED);
			return false;
		}
	}
	
	bool DBTools::getAllPoints(OUT std::vector<Point3D>& vPoint)
	{
		std::vector<Point3D> vPointTemp;
		std::string sql="select * from point;";
		int res=mysql_query(&myCont,sql.c_str());
		if(!res)
		{
			MYSQL_RES *result = mysql_store_result(&myCont);
			if(mysql_num_rows(result))
			{
				MYSQL_ROW sql_row;
				Point3D pTemp;
				while(sql_row=mysql_fetch_row(result))
				{
					pTemp.setId(atoi(sql_row[0]));
					pTemp.setLatitude(intToFloat(atoi(sql_row[1])));
					pTemp.setLongitude(intToFloat(atoi(sql_row[2])));
					if(atoi(sql_row[3])>0)
					    pTemp.setAltitude(intToFloat(atoi(sql_row[3])));
					else
						pTemp.setAltitude(0);
					vPointTemp.push_back(pTemp);	
				}
				vPoint = vPointTemp;
			}
			mysql_free_result(result);
		}
		return true;
	}
	
	bool DBTools::getPointById(IN int id,OUT Point3D & point)
	{
		std::stringstream ss_pid;
		ss_pid<<id;
		std::string sql="select x,y,z from point where point_id="+ss_pid.str()+";";
		int res=mysql_query(&myCont,sql.c_str());
		if(!res)
		{
			MYSQL_RES *result = mysql_store_result(&myCont);
			if(mysql_num_rows(result))
			{
				MYSQL_ROW sql_row;
				Point3D pTemp;
				sql_row=mysql_fetch_row(result);

				pTemp.setId(id);
				pTemp.setLatitude(intToFloat(atoi(sql_row[0])));
				pTemp.setLongitude(intToFloat(atoi(sql_row[1])));
				if(atoi(sql_row[3])>0)
					pTemp.setAltitude(intToFloat(atoi(sql_row[2])));
				else
					pTemp.setAltitude(0);

				point = pTemp;
			}
			mysql_free_result(result);
			return true;
		}
		return false;
	}

	int DBTools::getIdByPoint(IN Point3D& point)
	{
		int pointid = 0;
		Point3D pTemp = point;

		std::stringstream ss_x;
		std::stringstream ss_y;
		std::stringstream ss_z;
		ss_x.precision(9);
		ss_y.precision(9);
		ss_z.precision(9);
		ss_x<<floatToInt(pTemp.getLatitude());
		ss_y<<floatToInt(pTemp.getLongitude());
		if(pTemp.getAltitude()>0)
			ss_z<<floatToInt(pTemp.getAltitude());
		else
			ss_z<<0;
		std::string sql="select point_id from point where x="+ss_x.str()+" and y="+ss_y.str()+" and z="+ss_z.str()+";";
		int res=mysql_query(&myCont,sql.c_str());
		//std::cout<<ss_x.str()<<std::endl;
		if(!res)
		{
			MYSQL_RES *result = mysql_store_result(&myCont);
			if(mysql_num_rows(result))
			{
				MYSQL_ROW sql_row;
				sql_row=mysql_fetch_row(result);
				pointid = atoi(sql_row[0]);
			}
			mysql_free_result(result);
		}
		return pointid;
	}

	int DBTools::insertPoint(INOUT Point3D& point)
	{
		Point3D pTemp = point;
		int pointid = getIdByPoint(pTemp);
		if( pointid == 0)
		{
			std::stringstream ss_x;
			std::stringstream ss_y;
			std::stringstream ss_z; 
			ss_x.precision(9);
		    ss_y.precision(9);
		    ss_z.precision(9);
			ss_x<<floatToInt(pTemp.getLatitude());
			ss_y<<floatToInt(pTemp.getLongitude());
			
			if(pTemp.getAltitude()>0)
			    ss_z<<floatToInt(pTemp.getAltitude());
			else
				ss_z<<0;

			std::string sql="insert into point(x,y,z) values("+ss_x.str()+"," + ss_y.str() + " ," + ss_z.str()+ ");";
			mysql_query(&myCont,sql.c_str());
			 //std::cout<<ss_x.str()<<std::endl;
			pointid = mysql_insert_id(&myCont);//get last inserted id.
			pTemp.setId(pointid);
			point = pTemp;
		}
		point.setId(pointid);
		return pointid;
	}

	bool DBTools::insertPoints(INOUT std::vector<Point3D>& vPoint)
	{
		int insertid  = 0;
		std::vector<Point3D> vPointTemp = vPoint;
		std::vector<Point3D>::iterator vPointIter = vPointTemp.begin();

		while(vPointIter != vPointTemp.end())
		{
			insertid = insertPoint(*vPointIter);
			if(insertid == 0)
				return false;

			vPointIter++;
		}

		return true;
	}

	bool DBTools::deleteAllTable(std::string tablename)
	{
		std::stringstream ss_tn;
		ss_tn<<tablename;
		std::string sql="truncate table " +ss_tn.str() + ";";
		int res=mysql_query(&myCont,sql.c_str());
		if(!res)
			return true;
		else
			return false;
	}

	bool DBTools::getAllLines(OUT std::vector<Line>& vLine)
	{
		std::vector<Line> vLineTemp;
		std::string sql="select a.line_id,a.from_point,a.to_point,b.point_id from line a,shape_points b where a.line_id = b.line_id ";
		int res=mysql_query(&myCont,sql.c_str());
		if(!res)
		{
			MYSQL_RES *result = mysql_store_result(&myCont);
			if(mysql_num_rows(result))
			{
				MYSQL_ROW sql_row;
				Line lineTemp;
				Point3D pTemp;
				std::vector<Point3D> vPointTemp;
				int lineid = 0;
				while(sql_row=mysql_fetch_row(result))
				{
					if(lineid != atoi(sql_row[0]))
					{
						if(vPointTemp.size() != 0)
						{
							lineTemp.setPoints(vPointTemp);
							vLine.push_back(lineTemp);
							vPointTemp.clear();
						}
						lineTemp.setId(atoi(sql_row[0]));
					    getPointById(atoi(sql_row[1]),pTemp);
					    vPointTemp.push_back(pTemp);
					    getPointById(atoi(sql_row[2]),pTemp);
						vPointTemp.push_back(pTemp);
					}
					
					getPointById(atoi(sql_row[3]),pTemp);
					vPointTemp.push_back(pTemp);

					lineid = atoi(sql_row[0]);
				}
			}
			mysql_free_result(result);
			return true;
		}
		return false;
	}

	bool DBTools::getLineById(IN int id, OUT Line& line)
	{
		std::stringstream ss_id;
		ss_id<<id;
		std::string sql="select a.line_id,a.from_point,a.to_point,b.point_id from line a,shape_points b where a.line_id ="+ss_id.str()+" and b.line_id="+ss_id.str()+" order by b.sequence_number;";
		int res=mysql_query(&myCont,sql.c_str());
		if(!res)
		{
			MYSQL_RES *result = mysql_store_result(&myCont);
			if(mysql_num_rows(result))
			{
				MYSQL_ROW sql_row;
				Point3D pTemp;
				std::vector<Point3D> vPointTemp;
				bool flag = false;
				while(sql_row=mysql_fetch_row(result))
				{
					if(!flag)
					{
						line.setId(atoi(sql_row[0]));
					    getPointById(atoi(sql_row[1]),pTemp);
					    vPointTemp.push_back(pTemp);
					    getPointById(atoi(sql_row[2]),pTemp);
						vPointTemp.push_back(pTemp);

						flag = true;
					}
					getPointById(atoi(sql_row[3]),pTemp);
					vPointTemp.push_back(pTemp);
				}

				line.setPoints(vPointTemp);
				vPointTemp.clear();
			}
			mysql_free_result(result);
			return true;
		}
		return false;
	}

	int  DBTools::getIdByLine(IN Line& line)
	{
		int lineid = 0;
		bool flag = true;

		std::vector<Point3D> vPointTemp = line.getPoints();
		int frompointID = getIdByPoint(vPointTemp[0]);
		int topointID = getIdByPoint(vPointTemp[1]);

		std::stringstream ss_from;
		std::stringstream ss_to;
		ss_from<<frompointID;
		ss_to<<topointID;
		std::string sql="select line_id from line where from_point ="+ss_from.str()+" and to_point ="+ss_to.str()+";";
		int res=mysql_query(&myCont,sql.c_str());
		if(!res)
		{
			MYSQL_RES *result = mysql_store_result(&myCont);
			if(mysql_num_rows(result))
			{
				MYSQL_ROW sql_row;
				while(sql_row=mysql_fetch_row(result))
				{
					lineid = atoi(sql_row[0]);
					
					std::stringstream ss_id;
					ss_id<<lineid;
					std::string sql="select point_id,sequence_number from shape_points where line_id ="+ss_id.str()+" order by sequence_number;";
					int res=mysql_query(&myCont,sql.c_str());
					if(!res)
					{
						MYSQL_RES *result = mysql_store_result(&myCont);
						if(mysql_num_rows(result))
						{
							MYSQL_ROW sql_row_in;
							int index = 0;
							//int endWhile = 1;
							while(sql_row_in=mysql_fetch_row(result))
							{
								index = atoi(sql_row_in[1])+1;
								if(atoi(sql_row_in[0]) != getIdByPoint(vPointTemp[index]))
								{
									lineid = 0;
									flag = false;
								}		
							}
							if(flag)
							{
								mysql_free_result(result);
								break;
							}
						}
						else if(vPointTemp.size() == 2)
						{
							mysql_free_result(result);
							break;
						}
						mysql_free_result(result);
					}
					
				}
			}
			else
			{
				lineid = 0;
			}
			mysql_free_result(result);
		}
		return lineid;
	}

	bool DBTools::getLineByGps(Point3D & point,OUT std::vector<Line>& vLine)
	{
		int pointid = getIdByPoint(point);
		if(pointid == 0)
			return false;
		std::stringstream ss_id;
		ss_id<<pointid;

		std::string sql="select a.line_id from line a,shape_points b where a.line_id = b.line_id and ( a.from_point="+ss_id.str()+" or a.to_point="+ss_id.str()+" or b.point_id="+ss_id.str()+" );";
		int res=mysql_query(&myCont,sql.c_str());
		if(!res)
		{
			MYSQL_RES *result = mysql_store_result(&myCont);
			if(mysql_num_rows(result))
			{
				MYSQL_ROW sql_row;
				Line lineTemp;
				while(sql_row=mysql_fetch_row(result))
				{
					getLineById(atoi(sql_row[0]),lineTemp);
					vLine.push_back(lineTemp);
				}
			}
			mysql_free_result(result);
			return true;
		}
		return false;
	}

	int DBTools::insertLine(INOUT Line& line)
	{
		int lineid = getIdByLine(line);
		if(lineid != 0)
			return lineid;
		int from_point,to_point;
		std::vector<Point3D> vPointTemp;
		vPointTemp = line.getPoints();
		//line
		{
			from_point = insertPoint(vPointTemp[0]);
			to_point = insertPoint(vPointTemp[1]);
			std::stringstream ss_from;
			std::stringstream ss_to;
			ss_from<<from_point;
			ss_to<<to_point;

			std::string sql="insert into line(from_point,to_point) values("+ss_from.str()+"," + ss_to.str()+ ");";
			mysql_query(&myCont,sql.c_str());
			lineid = mysql_insert_id(&myCont);
		}
		//shape points
		for(int pcount = 1; pcount<vPointTemp.size()-1;pcount++)
		{
			std::stringstream ss_lineid;
			std::stringstream ss_pointid;
			std::stringstream ss_number;
			ss_lineid<<lineid;
			ss_pointid<<insertPoint(vPointTemp[pcount+1]);
			ss_number<<pcount;

			std::string sql="insert into shape_points(line_id,point_id,sequence_number) values("+ss_lineid.str()+"," + ss_pointid.str() + " ," + ss_number.str()+ ");";
			mysql_query(&myCont,sql.c_str());
		}
			
		line.setId(lineid);
		return lineid;
	}

	bool DBTools::insertLines(INOUT std::vector<Line>& vLine)
	{
		int lineid  = 0;
		std::vector<Line> vLineTemp = vLine;
		std::vector<Line>::iterator vLineIter = vLineTemp.begin();

		while(vLineIter != vLineTemp.end())
		{
			lineid = insertLine(*vLineIter);
			if(lineid == 0)
				return false;

			vLineIter++;
		}

		return true;
	}

	int DBTools::floatToInt(float var)
	{
		return (var-0.5)/0.0000001;
	}
	float DBTools::intToFloat(int var)
	{
		return var*0.0000001+0.5;
	}

	int DBTools::insertSurface(INOUT Surface& surface)
	{
		std::string sql="insert into surface values();";
		mysql_query(&myCont,sql.c_str());
		int surfaceId = mysql_insert_id(&myCont);

		std::vector<SurfaceBoundary> vSurBoundTemp = surface.getSurfaceBoundaries();
		std::vector<SurfaceBoundary>::iterator vsfbIter = vSurBoundTemp.begin();
		while(vsfbIter != vSurBoundTemp.end())
		{
			int lineid = insertLine(*vsfbIter);

			std::stringstream ss_lineid;
			std::stringstream ss_surfaceid;
			std::stringstream ss_imput;
			std::stringstream ss_closure;
			ss_lineid<<lineid;
			ss_surfaceid<<surfaceId;
			ss_imput<<vsfbIter->isImputed();
			ss_closure<<vsfbIter->isClosure();

			std::string sql="insert into surface_boundaries values("+ss_lineid.str()+ " , " + ss_surfaceid.str()+ " , " + ss_imput.str()+ " ," + ss_closure.str()+ ");";
			mysql_query(&myCont,sql.c_str());

			std::vector<int> vOrgTemp = vsfbIter->getOrigin();
			std::vector<int>::iterator vOrgItor = vOrgTemp.begin();
			while(vOrgItor != vOrgTemp.end())
			{
				std::stringstream ss_lineid;
			    std::stringstream ss_paintid;
				ss_lineid<<lineid;
				ss_paintid<<*vOrgItor;

				std::string sql="insert into line_origin values("+ss_lineid.str()+"," + ss_paintid.str()+ ");";
				mysql_query(&myCont,sql.c_str());

				vOrgItor++;
			}
			vsfbIter++;
		}

		surface.setId(surfaceId);
		return surfaceId;
	}

	int DBTools::insertPaint(INOUT Paint& paint)
	{
		Surface sfTemp = paint.getSurface();
		int sfId = insertSurface(sfTemp);

		Point3D pTemp = paint.getLandMark();
		int landmarkId = insertPoint(pTemp);

		std::stringstream ss_surfaceid;
		std::stringstream ss_landmark;
		std::stringstream ss_R;
		std::stringstream ss_G;
		std::stringstream ss_B;
		ss_surfaceid<<sfId;
		ss_landmark<<landmarkId;
		ss_R<<(uint32)paint.getColor().r;
		ss_G<<(uint32)paint.getColor().g;
		ss_B<<(uint32)paint.getColor().b;
		std::string sql="insert into paint(surface_id,landmark_point,color_r,color_g,color_b) values("+ss_surfaceid.str()+ " , " + ss_landmark.str()+ " , " + ss_R.str()+ " , " + ss_G.str()+ " ," + ss_B.str()+ ");";
		mysql_query(&myCont,sql.c_str());
		int paintId = mysql_insert_id(&myCont);
		paint.setId(paintId);
		return paintId;
	}


	void DBTools::importdatafromNewcoToMaster()
	{
		std::string sql="select * from newco.points;";
		int res=mysql_query(&myCont,sql.c_str());
		if(!res)
		{
			MYSQL_RES *result = mysql_store_result(&myCont);
			if(mysql_num_rows(result))
			{
				MYSQL_ROW sql_row;
				Point3D pTemp;
				while(sql_row=mysql_fetch_row(result))
				{
					pTemp.setId(atoi(sql_row[0]));
					pTemp.setLatitude(atof(sql_row[1]));
					pTemp.setLongitude(atof(sql_row[2]));
					pTemp.setAltitude(atof(sql_row[3]));
					
					insertPoint(pTemp);	
				}
			}
			mysql_free_result(result);
		}
	}


}