#pragma once
#include <vector>
#include "typeDefine.h"
#include "database.h"
#include "databaseInVehicle.h"

namespace ns_historyLine{

	using std::list;
	using std::vector;
	using ns_database::databaseInVehicle;
	using ns_database::point3D_t;
	using ns_database::lineAttributes_t;

	struct lineInfoPerVector_t{
	uint8 lineStyle;
	int32 pointNum;
	vector<point3D_t> allGps;
	};

	//extern list<lineInfoPerVector_t> historyLine;
	class saveLinePointInSafe
	{
		private:
				list<lineInfoPerVector_t> historyLine;
				list<point3D_t>		  gpsBuffer;
				HANDLE _lineMutex;
				HANDLE _gpsMutex;
				int lineNum;
				int gpsNum;

		public:
			saveLinePointInSafe(int numberLine,int numberGps);
			~saveLinePointInSafe(void);
			void saveHistoryLine(databaseInVehicle* database_gp);
			void saveCurrentGps(point3D_t gpsPoint);
			void getHistoryBuffer(list<lineInfoPerVector_t> &historyList);
			void getGpsBuffer(list<point3D_t> &gpsList);
	};
}