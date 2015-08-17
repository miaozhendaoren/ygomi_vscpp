/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  roadVectorGen.cpp
* @brief Source file for calculating principal curve and road vector generation
*
* Change Log:
*      Date                Who             What
*      2015/5/12           Linkun Xu       Create
*******************************************************************************
*/

#include "roadVectorGen.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;
using namespace laneSpace;
using namespace ns_database;

laneQueueClass  newDataQueues;
databaseServer* database_gp;

void main()
{
    ifstream fin("E:\\Newco\\demo\\code\\newco_demo04\\Demo\\Ford\\server\\server\\log\\newData2p3_C_bad.txt");
    string line;

    database_gp = new ns_database::databaseServer();

    vector<vector<double>> valVecVec;
    double val;
    int dataIdx = 1;

    while(1)
    {
        for(int idx = 0; idx < 6; ++idx)
        {
            if(getline(fin,line))
            {
                istringstream sstr(line);
                vector<double> valVec;
                while(sstr >> val)
                {
                    valVec.push_back(val);
                }
                valVecVec.push_back(valVec);
            }else
            {
                // File end
                int stop = 1;
            }
        }
        getline(fin,line);

        printf("dataIdx = %d\n", dataIdx++);

        for(int idx = 0; idx < valVecVec[0].size(); ++idx)
        {
            laneType_t linePoint;
            linePoint.gpsL.lat = valVecVec[0][idx];
            linePoint.gpsR.lat = valVecVec[1][idx];
            linePoint.gpsL.lon = valVecVec[2][idx];
            linePoint.gpsR.lon = valVecVec[3][idx];
            linePoint.gpsL.alt = 0;
            linePoint.gpsR.alt = 0;
            linePoint.linePaintFlagL = valVecVec[4][idx];
            linePoint.linePaintFlagR = valVecVec[5][idx];

            newDataQueues.addLanePoint(0, linePoint);
        }

        valVecVec.clear();

        vector<int> storeLane;
        storeLane.push_back(0);
        roadVectorGen(newDataQueues, storeLane, dashAlign_e);

    }

    fin.close();
}
