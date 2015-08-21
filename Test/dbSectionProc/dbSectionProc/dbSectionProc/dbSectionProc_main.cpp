#include "apiDataStruct.h"
#include "RoadVecGen.h"
#include "VisualizationApis.h"

list<list<vector<point3D_t>>>       newDataVec;
list<list<vector<point3D_t>>>       fgData;

CRoadVecGen g_RoadVecGen;

void main()
{
    // set configuration path
    g_RoadVecGen.setSectionConfigPath(".\\config\\manualSeg.txt");

    // road new reported data from file
    readReportData(".\\config\\pathlist.txt", newDataVec);

#ifdef VISUALIZATION_ON
    char winname[MAX_PATH];
    uint32 grpNum = 1;
    list<list<vector<point3D_t>>>::iterator grpItor = newDataVec.begin();
    while (grpItor != newDataVec.end())
    {
        sprintf_s(winname, MAX_PATH - 1, "group_%d.png", grpNum++);
        showImage(*grpItor, Scalar(255, 0, 0), winname);

        grpItor++;
    }
#endif

    g_RoadVecGen.roadSectionsGen(newDataVec, fgData);

    return;
}

