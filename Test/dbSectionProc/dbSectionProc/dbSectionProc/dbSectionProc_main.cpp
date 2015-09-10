#include "apiDataStruct.h"
#include "RoadVecGen2.h"
#include "VisualizationApis.h"

list<list<vector<point3D_t>>>       newDataVec;
list<list<vector<point3D_t>>>       fgData;
list<segAttributes_t>               segConfigList;

ns_database::CRoadVecGen2 g_RoadVecGen;

ns_database::CRoadVecGen2 *roadVecGen2_gp;

void main()
{
    // set configuration path
    g_RoadVecGen.setSectionConfigPath(".\\config\\manualSeg.txt", segConfigList);

    // road new reported data from file
    readReportData(".\\config\\pathlist.txt", newDataVec);

#if VISUALIZATION_ON
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

#if SAVE_DATA_ON
    char fgname[MAX_PATH];
    uint32 fgNum = 1;
    list<list<vector<point3D_t>>>::iterator fgItor = fgData.begin();
    while (fgItor != fgData.end())
    {
        sprintf_s(fgname, MAX_PATH - 1, "fgroup_%d.txt", fgNum++);
        saveListVec(*fgItor, fgname);

        fgItor++;
    }
#endif

    return;
}

