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
#include "roadVectorGenPushAllVec.h"
#include "roadVectorGenSimpleAlign.h"
#include "roadVectorGenDashAlign.h"

using std::vector;
using std::queue;
using std::list;
using namespace laneSpace;
using namespace ns_database;

extern databaseServer* database_gp;

extern bool doubleDataValid(double dataIn);

bool roadVectorGen(laneQueueClass &newDataQueues, vector<int> &storeLane, roadVecGenEnum alg_e)
{
    bool returnFlag;

    switch(alg_e)
    {
        case pushAllVec_e:
        {
            returnFlag = roadVectorGenPushAllVec(newDataQueues, storeLane);
            break;
        }
        case simpleAlign_e:
        {
            returnFlag = roadVectorGenSimpleAlign(newDataQueues, storeLane);
            break;
        }
        case dashAlign_e:
        {
            returnFlag = roadVectorGenDashAlign(newDataQueues, storeLane);
            break;
        }
        default:
        {
            //error;
        }
    }


    return returnFlag;
}

