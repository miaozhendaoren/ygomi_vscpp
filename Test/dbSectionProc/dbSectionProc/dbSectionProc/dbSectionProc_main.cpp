#include "apiDataStruct.h"
#include "RoadVecGen.h"

list<vector<point3D_t>>       newDataVec;
list<list<vector<point3D_t>>> fgData;

CRoadVecGen g_RoadVecGen;

void readReportData(char *filename, list<vector<point3D_t>> &newData);

void main()
{
    // set configuration path
    g_RoadVecGen.setSectionConfigPath("C:\\Projects\\manualSeg.txt");

    // road new reported data from file
    readReportData("C:\\Projects\\dataStruct_0.txt", newDataVec);

    g_RoadVecGen.roadSectionsGen(newDataVec, fgData);

    return;
}


void readReportData(char *filename, list<vector<point3D_t>> &newData)
{
    if (nullptr == filename)
    {
        printf("filename is not correct\n");
        return;
    }

    // release previous data
    if (!newData.empty())
    {
        list<vector<point3D_t>>::iterator it;
        for (it = newData.begin(); it != newData.end(); it++)
        {
            it->clear();
        }
        newData.clear();
    }

    FILE *fp = nullptr;
    errno_t err = fopen_s(&fp, filename, "rt");
    if (0 != err)
    {
        printf("failed to open file %s\n", filename);
        return;
    }

    double tempData[19];
    point3D_t currentPoint = { 0 };
    vector<point3D_t> leftLine;
    vector<point3D_t> rightLine;

    // initialize tempData
    memset(tempData, 0, 19);

    while (!feof(fp))
    {
        // scan row data
        fscanf_s(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \
                      %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                     &tempData[ 0], &tempData[ 1], &tempData[ 2], &tempData[ 3],
                     &tempData[ 4], &tempData[ 5], &tempData[ 6], &tempData[ 7],
                     &tempData[ 8], &tempData[ 9], &tempData[10], &tempData[11],
                     &tempData[12], &tempData[13], &tempData[14], &tempData[15],
                     &tempData[16], &tempData[17], &tempData[18]);

        // left line point
        currentPoint.lat       = tempData[0];
        currentPoint.lon       = tempData[1];
        currentPoint.paintFlag = (float)tempData[3];

        leftLine.push_back(currentPoint);

        // right line point
        currentPoint.lat       = tempData[12];
        currentPoint.lon       = tempData[13];
        currentPoint.paintFlag = (float)tempData[14];

        rightLine.push_back(currentPoint);
    }

    // add lane data
    newData.push_back(leftLine);
    newData.push_back(rightLine);
}
