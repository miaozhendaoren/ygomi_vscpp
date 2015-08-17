/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  testMain.cpp
* @brief test main function for database
*
* Change Log:
*      Date                Who             What
*      2015/1/7         Linkun Xu        Create
*******************************************************************************
*/

#include "database.h"

using namespace ns_database;

void main()
{
    while(1)
    {
        int a = 0;

        {
            database database_1("../../DHD/road_data_germ1.dhd");

            void* input = 0;

            database_1.readDb(&input, file_e);

            /***************************************************************
             * getAllVectors
             ***************************************************************/
            {
                std::list<std::list<std::vector<point3D_t>>> allLines; // segment list / vector list / point list / point
                std::list<std::list<lineAttributes_t>> lineAttr; // segment list / vector list / attributes
                database_1.getAllVectors(allLines, lineAttr);
                database_1.getAllVectors_clear(allLines, lineAttr);
            }

            /***************************************************************
             * getSegmentByGps
             ***************************************************************/
            {
                int length;
                uint8 outBuff[50000];
                int numSeg;

                numSeg = database_1.getSegmentNum();

                std::list<std::list<std::vector<point3D_t>>> allLines; // segment list / vector list / point list / point
                std::list<std::list<lineAttributes_t>> lineAttr; // segment list / vector list / attributes
                
                database_1.getAllVectors(allLines, lineAttr);

                std::list<std::list<std::vector<point3D_t>>>::iterator segListIter = allLines.begin();
                std::list<std::list<lineAttributes_t>>::iterator lineAttrIter = lineAttr.begin();
                // For each segment
                while(segListIter != allLines.end())
                {
                    std::list<std::vector<point3D_t>>::iterator lineListIter = (*segListIter).begin();
                    std::list<lineAttributes_t>::iterator lineAttrInSegIter = (*lineAttrIter).begin();

                    while(lineListIter != (*segListIter).end())
                    {
                        // middle line
                        if((*lineAttrInSegIter).lineStyle == 1)
                        {
                            int numPoints = (*lineListIter).size();

                            for(int pointIdx = 0; pointIdx < numPoints; pointIdx++)
                            {
                                point3D_t gps = (*lineListIter)[pointIdx];
                                uint8 existFlag;
                                segAttributes_t segAttr;

                                database_1.getSegmentByGps(&gps, &existFlag, &segAttr);

                                if(existFlag)
                                {
                                    printf("segment found, segmentID = %u.          ", segAttr.segId);
                                }
                                else
                                {
                                    printf("\n\nsegment not found!\n\n");
                                }
                            }
                        }

                        lineAttrInSegIter++;
                        lineListIter++;
                    }

                    lineAttrIter++;
                    segListIter++;
                }

                database_1.getAllVectors_clear(allLines, lineAttr);
            }
            
            /***************************************************************
             * addFurniture
             ***************************************************************/
            {
                furAttributes_t furAttrSrc, furAttrDst;
                uint8 furExist;
                uint8 outBuff[50000];
                void* outBuffP = outBuff;
                int buffLen;
                int segId = 1;
                
                std::list<std::list<std::vector<point3D_t>>> allLines; // segment list / vector list / point list / point
                std::list<std::list<lineAttributes_t>> lineAttr; // segment list / vector list / attributes
                
                database_1.getAllVectors(allLines, lineAttr);

                std::list<std::list<std::vector<point3D_t>>>::iterator segIter = allLines.begin();
                
                while(segIter != allLines.end())
                {
                    uint32 furId = 1;
                    std::list<std::vector<point3D_t>>::iterator lineIter = (*segIter).begin();

                    for(int pointIdx = 0; pointIdx < (*lineIter).size(); pointIdx++)
                    {
                        point3D_t location = (*lineIter)[pointIdx];

                        if(1)
                        // Indicate the segment of the furniture by ID
                        {
                            furAttrSrc.segId_used = 1;
                            furAttrSrc.segId = segId;
                        }
                        else
                        // Use GPS to locate the segment of the furniture
                        {
                            furAttrSrc.segId_used = 0;
                        }
                        furAttrSrc.furId_used = 1;
                        furAttrSrc.furId = furId++;
                        furAttrSrc.segVersion_used = 1;
                        furAttrSrc.segVersion = 1;
                        furAttrSrc.location_used = 1;
                        furAttrSrc.location = location;
                        furAttrSrc.angle_used = 0;
                        furAttrSrc.type_used = 0;
                        furAttrSrc.side_used = 0;
                        furAttrSrc.sideFlag_used = 0;
                        furAttrSrc.offset_used = 0;
                        furAttrSrc.reliabRating_used = 1;
                        furAttrSrc.reliabRating = 1;

                        database_1.convFurnitureToTlv(&furAttrSrc, memory_e, &outBuffP, &buffLen);

                        outBuffP = outBuff;
                        database_1.addFurnitureTlv(outBuff, buffLen);

                        database_1.getFurnitureByGps(&location, &furExist, &furAttrDst);

                        database_1.getFurnitureById(furAttrSrc.segId, furAttrSrc.furId, &furExist, &furAttrDst);


                    }

                    segId++;
                    segIter++;
                }

                database_1.getAllVectors_clear(allLines, lineAttr);
            }

            /***************************************************************
             * getSegmentById / convSegmentToTlv / addSegmentTlv / getVectorsInSegTlv / addAllVectorsInSegTlv
             ***************************************************************/
            {
                int length;
                uint8 outBuff[50000];
                int numSeg;

                FILE* fidTmpSeg = fopen("seg.bin", "wb");
                FILE* fidTmpVec = fopen("vec.bin", "wb");

                numSeg = database_1.getSegmentNum();

                for(int segIdx = 0; segIdx < numSeg; segIdx++)
                {
                    // Segment
                    uint8 existFlag;
                    segAttributes_t segAttr;
                    void* outBuffP = outBuff;

                    database_1.getSegmentById(segIdx+1, &existFlag, &segAttr);
                    database_1.convSegmentToTlv(&segAttr, memory_e, &outBuffP, &length);

                    fwrite(outBuff, 1, length, fidTmpSeg);

                    database_1.addSegmentTlv(outBuff, length);

                    // Vector
                    outBuffP = outBuff;
                    database_1.getVectorsInSegTlv(segIdx+1, memory_e, &outBuffP, &length);
                    
                    fwrite(outBuff, 1, length, fidTmpVec);

                    database_1.addAllVectorsInSegTlv(outBuff, length);
                }

                fclose(fidTmpSeg);
                fclose(fidTmpVec);

                numSeg = database_1.getSegmentNum();
            }

        }
    }

}
