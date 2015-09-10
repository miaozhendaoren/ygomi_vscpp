/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  database.cpp
* @brief Source file for database, common definitions for vehichle and server
*
* Change Log:
*      Date                Who             What
*      2015/1/7           Linkun Xu       Create
*******************************************************************************
*/

#include "database.h"

#include <stdio.h>    // FILE
#include <windows.h>  // CreateMutex, FOREGROUND_RED

#include "LogInfo.h"  // logPrintf

using std::list;
using std::vector;
using std::string;

namespace ns_database
{
    furAttributes_t::furAttributes_t()
    {
        format();

        // Create mutexes
        //_hMutexFile = CreateMutex(NULL,FALSE,NULL);
        //_hMutexMemory = CreateMutex(NULL,FALSE,NULL);
        //ReleaseMutex(_hMutexFile);
        //ReleaseMutex(_hMutexMemory);
    }

    furAttributes_t::~furAttributes_t()
    {
        //CloseHandle(_hMutexFile);
        //CloseHandle(_hMutexMemory);
        
        boundary.clear();
    }

    void furAttributes_t::format()
    {
        segId_used = 0;
        furId_used = 0;
        segVersion_used = 0;
        location_used = 0;
        angle_used = 0;
        type_used = 0;
        side_used = 0;
        sideFlag_used = 0;
        offset_used = 0;
        reliabRating_used = 0;
        boundary_used = 0;
    }

    database::database()
    {
        // Get TLV configurations
        // todo: get configurations according to database revision
        initTlvCfg();

        initParams();

        // Assign memory to header with empty TLVs
        tlvCommon_t tlvTmp;
        tlvTmp.typeId = null_e;
        tlvTmp.usedFlag = 0;
        _header.assign(header_max_e - header_base_e, tlvTmp);

        // Create mutexes
        _hMutexFile = CreateMutex(NULL,FALSE,NULL);
        _hMutexMemory = CreateMutex(NULL,FALSE,NULL);
        ReleaseMutex(_hMutexFile);
        ReleaseMutex(_hMutexMemory);
    }

    database::database(string inFileStr)
    {
        // Prepare database file name string
        _dbFileName = inFileStr;

        // Get TLV configurations
        // TODO: get configurations according to database revision
        initTlvCfg();

        initParams();

        // Create mutexes
        _hMutexFile = CreateMutex(NULL,FALSE,NULL);
        _hMutexMemory = CreateMutex(NULL,FALSE,NULL);
        ReleaseMutex(_hMutexFile);
        ReleaseMutex(_hMutexMemory);
    }

    database::~database()
    {
        _header.clear();
        _segmentList.clear();
        _vectorList.clear();
        _lineAttrList.clear();

        CloseHandle(_hMutexFile);
        CloseHandle(_hMutexMemory);
    }

    /******************** Memory operation methods *****************/
    void database::initParams()
    {
        _distThreshFarFar = 40.0;// 40m
        _distThreshFar  = 25.0;
        _distThreshMid  = 25.0;
        _distThreshNear = 12.0;

        _angleThresh = PI;// FIXME: PI means not filter on angle
    }

    void database::initTlvCfg()
    {
        setTvlCfg(&_tlvCfg_sec_a[headerSec_e - secBase_e], headerSec_e, tlv_complexV_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_sec_a[segmentSec_e - secBase_e], segmentSec_e, tlv_complexV_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_sec_a[vectorSec_e - secBase_e], vectorSec_e, tlv_complexV_e, 0, 1, 0);

        setTvlCfg(&_tlvCfg_header_a[header_revision_e - header_base_e], header_revision_e, tvUint16_e, 2, 1, 0);
        setTvlCfg(&_tlvCfg_header_a[header_countryId_e - header_base_e], header_countryId_e, tvUint16_e, 2, 1, 0);
        setTvlCfg(&_tlvCfg_header_a[header_dateTime_e - header_base_e], header_dateTime_e, tvDateTime_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_header_a[header_segIdList_e - header_base_e], header_segIdList_e, tlvUint32s_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_header_a[header_fileId_e - header_base_e], header_fileId_e, tvUint32_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_header_a[header_vendor_e - header_base_e], header_vendor_e, tlvString_e, 0, 1, 0);

        setTvlCfg(&_tlvCfg_seg_a[seg_segId_e - seg_base_e], seg_segId_e, tvUint32_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_seg_a[seg_version_e - seg_base_e], seg_version_e, tvUint32_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_seg_a[seg_segLen_e - seg_base_e], seg_segLen_e, tvUint32_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_seg_a[seg_segType_e - seg_base_e], seg_segType_e, tvUint8_e, 1, 1, 0);
        setTvlCfg(&_tlvCfg_seg_a[seg_segPorts_e - seg_base_e], seg_segPorts_e, tlvPoints_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_seg_a[seg_segLinks_e - seg_base_e], seg_segLinks_e, tlvUint32s_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_seg_a[seg_roadLen_e - seg_base_e], seg_roadLen_e, tlvSingles_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_seg_a[seg_bridge_e - seg_base_e], seg_bridge_e, tvUint8_e, 1, 1, 0);
        setTvlCfg(&_tlvCfg_seg_a[seg_tunnel_e - seg_base_e], seg_tunnel_e, tvUint8_e, 1, 1, 0);
        setTvlCfg(&_tlvCfg_seg_a[seg_furList_e - seg_base_e], seg_furList_e, tlv_complexV_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_seg_a[seg_dynList_e - seg_base_e], seg_dynList_e, tlv_complexV_e, 0, 1, 0);

        setTvlCfg(&_tlvCfg_vec_a[vec_segId_e - vec_base_e], vec_segId_e, tvUint32_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_vec_a[vec_vecList_e - vec_base_e], vec_vecList_e, tlv_complexV_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_vec_a[vec_laneNum_e - vec_base_e], vec_laneNum_e, tvUint32_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_vec_a[vec_segNum_e - vec_base_e], vec_segNum_e, tvUint32_e, 4, 1, 0);
        
        setTvlCfg(&_tlvCfg_fur_a[fur_furId_e - fur_base_e], fur_furId_e, tvUint32_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_fur_a[fur_segVer_e - fur_base_e], fur_segVer_e, tvUint32_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_fur_a[fur_location_e - fur_base_e], fur_location_e, tlv_complexV_e, 1, 1, 0);
        setTvlCfg(&_tlvCfg_fur_a[fur_angle_e - fur_base_e], fur_angle_e, tvSingle_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_fur_a[fur_type_e - fur_base_e], fur_type_e, tvUint16_e, 2, 1, 0);
        setTvlCfg(&_tlvCfg_fur_a[fur_side_e - fur_base_e], fur_side_e, tlvUint32s_e, 2, 1, 0);
        setTvlCfg(&_tlvCfg_fur_a[fur_sideFlag_e - fur_base_e], fur_sideFlag_e, tvUint8_e, 1, 1, 0);
        setTvlCfg(&_tlvCfg_fur_a[fur_offset_e - fur_base_e], fur_offset_e, tvSingle_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_fur_a[fur_reliabRating_e - fur_base_e], fur_reliabRating_e, tvUint8_e, 1, 1, 0);
        setTvlCfg(&_tlvCfg_fur_a[fur_segId_e - fur_base_e], fur_segId_e, tvUint32_e, 4, 1, 0);
        
        setTvlCfg(&_tlvCfg_dataLine_a[data_lineId_e - data_lineBase_e], data_lineId_e, tvUint32_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_dataLine_a[data_lineWidth_e - data_lineBase_e], data_lineWidth_e, tvSingle_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_dataLine_a[data_lineStyle_e - data_lineBase_e], data_lineStyle_e, tvUint8_e, 1, 1, 0);
        setTvlCfg(&_tlvCfg_dataLine_a[data_lineSegVersion_e - data_lineBase_e], data_lineSegVersion_e, tvUint32_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_dataLine_a[data_linePointList_e - data_lineBase_e], data_linePointList_e, tlv_complexV_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_dataLine_a[data_lineLaneId_e - data_lineBase_e], data_lineLaneId_e, tlvUint32s_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_dataLine_a[data_lineLaneWidth_e - data_lineBase_e], data_lineLaneWidth_e, tvSingle_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_dataLine_a[data_lineLaneChFlag_e - data_lineBase_e], data_lineLaneChFlag_e, tvUint8_e, 0, 1, 0);
        setTvlCfg(&_tlvCfg_dataLine_a[data_linePaintFlagL_e - data_lineBase_e], data_linePaintFlagL_e, tvUint8_e, 1, 1, 0);
        setTvlCfg(&_tlvCfg_dataLine_a[data_linePaintFlagR_e - data_lineBase_e], data_linePaintFlagR_e, tvUint8_e, 1, 1, 0);


        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointLatitude_e - data_pointBase_e], data_pointLatitude_e, tvDouble_e, 8, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointLongitude_e - data_pointBase_e], data_pointLongitude_e, tvDouble_e, 8, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointAltitude_e - data_pointBase_e], data_pointAltitude_e, tvDouble_e, 8, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointDistance1_e - data_pointBase_e], data_pointDistance1_e, tvSingle_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointDistance2_e - data_pointBase_e], data_pointDistance2_e, tvSingle_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointDistance3_e - data_pointBase_e], data_pointDistance3_e, tvSingle_e, 4, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointDeviation1_e - data_pointBase_e], data_pointDeviation1_e, tvHalf_e, 2, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointDeviation2_e - data_pointBase_e], data_pointDeviation2_e, tvHalf_e, 2, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointDeviation3_e - data_pointBase_e], data_pointDeviation3_e, tvHalf_e, 2, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointReliabRating_e - data_pointBase_e], data_pointReliabRating_e, tvUint8_e, 1, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointPaintFlag_e - data_pointBase_e], data_pointPaintFlag_e, tvSingle_e, 1, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_mergeCounter_e - data_pointBase_e], data_mergeCounter_e, tvUint32_e, 4, 1, 0);

        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointLatitudeL_e - data_pointBase_e], data_pointLatitudeL_e, tvDouble_e, 8, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointLongitudeL_e - data_pointBase_e], data_pointLongitudeL_e, tvDouble_e, 8, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointAltitudeL_e - data_pointBase_e], data_pointAltitudeL_e, tvDouble_e, 8, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointLatitudeR_e - data_pointBase_e], data_pointLatitudeR_e, tvDouble_e, 8, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointLongitudeR_e - data_pointBase_e], data_pointLongitudeR_e, tvDouble_e, 8, 1, 0);
        setTvlCfg(&_tlvCfg_dataPoint_a[data_pointAltitudeR_e - data_pointBase_e], data_pointAltitudeR_e, tvDouble_e, 8, 1, 0);
    }

    void database::getTlvCfgbyId(IN  typeId_e typeId, 
                                 OUT tlvCfg_t* tlvCfg, 
                                 OUT typeId_e* idBase)
    {
        int itemIdx;

        if((typeId >= secBase_e) && (typeId < secMax_e))
        {
            itemIdx = typeId - secBase_e;
            *tlvCfg = _tlvCfg_sec_a[itemIdx];
            *idBase = secBase_e;
        }else if((typeId >= data_pointBase_e) && (typeId < data_pointMax_e))
        {
            itemIdx = typeId - data_pointBase_e;
            *tlvCfg = _tlvCfg_dataPoint_a[itemIdx];
            *idBase = data_pointBase_e;
        }else if((typeId >= data_lineBase_e) && (typeId < data_lineMax_e))
        {
            itemIdx = typeId - data_lineBase_e;
            *tlvCfg = _tlvCfg_dataLine_a[itemIdx];
            *idBase = data_lineBase_e;
        }else if((typeId >= header_base_e) && (typeId < header_max_e))
        {
            itemIdx = typeId - header_base_e;
            *tlvCfg = _tlvCfg_header_a[itemIdx];
            *idBase = header_base_e;
        }else if((typeId >= seg_base_e) && (typeId < seg_max_e))
        {
            itemIdx = typeId - seg_base_e;
            *tlvCfg = _tlvCfg_seg_a[itemIdx];
            *idBase = seg_base_e;
        }else if((typeId >= vec_base_e) && (typeId < vec_max_e))
        {
            itemIdx = typeId - vec_base_e;
            *tlvCfg = _tlvCfg_vec_a[itemIdx];
            *idBase = vec_base_e;
        }else if((typeId >= fur_base_e) && (typeId < fur_max_e))
        {
            itemIdx = typeId - fur_base_e;
            *tlvCfg = _tlvCfg_fur_a[itemIdx];
            *idBase = fur_base_e;
        }else
        {
            tlvCfg->typeId = null_e;
            *idBase = null_e;
        }

    }

    void database::setTvlCfg(OUT tlvCfg_t* tlv,
                             IN typeId_e typeId,
                             IN tlvType_e tlvType,
                             IN uint32 length,
                             IN uint16 startRev,
                             IN uint16 endRev)
    {
        tlv->typeId = typeId;
        tlv->tlvType = tlvType;
        tlv->length = length;
        tlv->startRev = startRev;
        tlv->endRev = endRev;
    }

    void database::setTvlCommon(OUT tlvCommon_t* tlv,
                                IN typeId_e typeId,
                                IN uint8 usedFlag,
                                IN tlvType_e tlvType,
                                IN uint32 length,
                                IN uint32 value)
    {
        tlv->typeId = typeId;
        tlv->usedFlag = usedFlag;
        tlv->tlvType = tlvType;
        tlv->length = length;
        tlv->value = value;
    }

    void database::initDb()
    {
        // Free memory before reset
        _header.clear();
        _segmentList.clear();
        _vectorList.clear();
        _lineAttrList.clear();

        // Reset
        setTvlCommon(&_headerSecHeader, headerSec_e, 1, tlv_complexV_e, 0, 0);
        setTvlCommon(&_segmentSecHeader, segmentSec_e, 1, tlv_complexV_e, 0, 0);
        setTvlCommon(&_vectorSecHeader, vectorSec_e, 1, tlv_complexV_e, 0, 0);

        // Assign memory to header with empty TLVs
        tlvCommon_t tlvTmp;
        tlvTmp.typeId = null_e;
        tlvTmp.usedFlag = 0;
        _header.assign(header_max_e - header_base_e, tlvTmp);

        for(uint32 itemIdx = 0; itemIdx < _header.size(); itemIdx++)
        {
            setTvlCommon(&tlvTmp, _tlvCfg_header_a[itemIdx].typeId, 0, _tlvCfg_header_a[itemIdx].tlvType, _tlvCfg_header_a[itemIdx].length, 0);
            _header[itemIdx] = tlvTmp;
        }
    }

    void database::convSegmentToTlv(IN segAttributes_t* segmentAttr, 
                                    IN resource_e sourceFlag,
                                    OUT void** output, 
                                    OUT int32* length)
    {
        if(sourceFlag == file_e)
        {
            WaitForSingleObject(_hMutexFile,INFINITE);
        }
        else
        {
            WaitForSingleObject(_hMutexMemory,INFINITE);
        }

        tlvCommon_t tlvTmp;
        tlvCfg_t* tlvCfgP;
        int byteNum = 0;

        tlvCfgP = &_tlvCfg_seg_a[seg_segId_e - seg_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, segmentAttr->segId_used, tlvCfgP->tlvType, tlvCfgP->length, segmentAttr->segId);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);
        
        tlvCfgP = &_tlvCfg_seg_a[seg_version_e - seg_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, segmentAttr->version_used, tlvCfgP->tlvType, tlvCfgP->length, segmentAttr->version);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_seg_a[seg_segType_e - seg_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, segmentAttr->type_used, tlvCfgP->tlvType, tlvCfgP->length, segmentAttr->type);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_seg_a[seg_segPorts_e - seg_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, segmentAttr->ports_used, tlvCfgP->tlvType, segmentAttr->numPort, 0);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        for(int portIdx = 0; portIdx < segmentAttr->numPort; portIdx++)
        {
            tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLatitude_e - data_pointBase_e];
            setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&segmentAttr->ports[portIdx].lat);
            byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

            tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLongitude_e - data_pointBase_e];
            setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&segmentAttr->ports[portIdx].lon);
            byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

            tlvCfgP = &_tlvCfg_dataPoint_a[data_pointAltitude_e - data_pointBase_e];
            setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&segmentAttr->ports[portIdx].alt);
            byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);
        }

        tlvCfgP = &_tlvCfg_seg_a[seg_segLinks_e - seg_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, segmentAttr->links_used, tlvCfgP->tlvType, segmentAttr->numPort, (uint32)segmentAttr->links);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_seg_a[seg_roadLen_e - seg_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, segmentAttr->roadLength_used, tlvCfgP->tlvType, tlvCfgP->length, (uint32)segmentAttr->roadLength);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_seg_a[seg_bridge_e - seg_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, segmentAttr->bridgeFlag_used, tlvCfgP->tlvType, tlvCfgP->length, segmentAttr->bridgeFlag);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_seg_a[seg_tunnel_e - seg_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, segmentAttr->tunnelFlag_used, tlvCfgP->tlvType, tlvCfgP->length, segmentAttr->tunnelFlag);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_seg_a[seg_furList_e - seg_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, segmentAttr->numFurniture_used, tlvCfgP->tlvType, segmentAttr->numFurniture, 0);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_seg_a[seg_dynList_e - seg_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, segmentAttr->numDynamicData_used, tlvCfgP->tlvType, segmentAttr->numDynamicData, 0);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        *length = byteNum;

        if(sourceFlag == file_e)
        {
            ReleaseMutex(_hMutexFile);
        }
        else
        {
            ReleaseMutex(_hMutexMemory);
        }
    }

    void database::convFurnitureToTlv(IN furAttributes_t* furnitureAttr, 
                                      IN resource_e sourceFlag,
                                      OUT void** output, 
                                      OUT int32* length)
    {
        if(sourceFlag == file_e)
        {
            WaitForSingleObject(_hMutexFile,INFINITE);
        }
        else
        {
            WaitForSingleObject(_hMutexMemory,INFINITE);
        }

        tlvCommon_t tlvTmp;
        tlvCfg_t* tlvCfgP;
        int byteNum = 0;

        // Keep furniture ID as the first TLV of one furniture!
        tlvCfgP = &_tlvCfg_fur_a[fur_furId_e - fur_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, furnitureAttr->furId_used, tlvCfgP->tlvType, tlvCfgP->length, furnitureAttr->furId);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_fur_a[fur_segVer_e - fur_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, furnitureAttr->segVersion_used, tlvCfgP->tlvType, tlvCfgP->length, furnitureAttr->segVersion);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_fur_a[fur_location_e - fur_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, furnitureAttr->location_used, tlvCfgP->tlvType, tlvCfgP->length, 0);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        if(furnitureAttr->location_used == 1)
        {
            tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLatitude_e - data_pointBase_e];
            setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&furnitureAttr->location.lat);
            byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

            tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLongitude_e - data_pointBase_e];
            setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&furnitureAttr->location.lon);
            byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

            tlvCfgP = &_tlvCfg_dataPoint_a[data_pointAltitude_e - data_pointBase_e];
            setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&furnitureAttr->location.alt);
            byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);
        }

        tlvCfgP = &_tlvCfg_fur_a[fur_angle_e - fur_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, furnitureAttr->angle_used, tlvCfgP->tlvType, tlvCfgP->length, *(uint32*)(&furnitureAttr->angle));
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_fur_a[fur_type_e - fur_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, furnitureAttr->type_used, tlvCfgP->tlvType, tlvCfgP->length, (uint32)furnitureAttr->type);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_fur_a[fur_side_e - fur_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, furnitureAttr->side_used, tlvCfgP->tlvType, tlvCfgP->length, (uint32)furnitureAttr->side);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_fur_a[fur_sideFlag_e - fur_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, furnitureAttr->sideFlag_used, tlvCfgP->tlvType, tlvCfgP->length, (uint32)furnitureAttr->sideFlag);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_fur_a[fur_offset_e - fur_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, furnitureAttr->offset_used, tlvCfgP->tlvType, tlvCfgP->length, *(uint32*)(&furnitureAttr->offset));
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_fur_a[fur_reliabRating_e - fur_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, furnitureAttr->reliabRating_used, tlvCfgP->tlvType, tlvCfgP->length, (uint32)furnitureAttr->reliabRating);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        tlvCfgP = &_tlvCfg_fur_a[fur_segId_e - fur_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, furnitureAttr->segId_used, tlvCfgP->tlvType, tlvCfgP->length, furnitureAttr->segId);
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        *length = byteNum;

        if(sourceFlag == file_e)
        {
            ReleaseMutex(_hMutexFile);
        }
        else
        {
            ReleaseMutex(_hMutexMemory);
        }
    }

    void database::calcFurnitureRoadSideLoc(INOUT furAttributes_t* furnitureAttr)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);
        
        uint8  segId_usedIn = furnitureAttr->segId_used;
        if(0 == segId_usedIn)
        {
            return;
        }
        uint32 segIdIn = furnitureAttr->segId;
        
        list<list<vector<point3D_t>>>::iterator vecListIter = _vectorList.begin();
        list<segAttributes_t>::iterator segAttrListIter = _segmentList.begin();
        
        vector<point3D_t> pointListPolygon;

        if(0 == furnitureAttr->location_used)
        {
            return;
        }
        point3D_t furLocation = furnitureAttr->location;
        point3D_t nearFurLocation = furLocation;
        uint8  furSideFlag_used = furnitureAttr->sideFlag_used;
        uint8  furSideFlag = furnitureAttr->sideFlag; // 1: right side, 2: left side, 3: both sides, 4: on the road
        if(1 != furSideFlag_used)
        {
            furSideFlag = 4;
        }

        // For each segment
        while(vecListIter != _vectorList.end())
        {
            if((*vecListIter).empty())
            // Segment with no vector
            {
                ++vecListIter;
                ++segAttrListIter;
                continue;
            }

            uint32 segmentIdDb = segAttrListIter->segId;
            double minDist = DBL_MAX;
            
            // Segment ID match
            if(segmentIdDb == segIdIn)
            {                
                int vectorNumInSeg = (*vecListIter).size();
                if(2 > vectorNumInSeg)
                {
                    continue;
                }
                
                list<vector<point3D_t>>::iterator pointListIter = vecListIter->begin();

                // For each vector
                //while(pointListIter != vecListIter->end())
                {
                    int numPoints = (*pointListIter).size();
                    // For each point
                    for(int pointIdx = 0; pointIdx < numPoints; pointIdx++)
                    {
                        point3D_t pointTmp = (*pointListIter)[pointIdx];

                        double distance = (furLocation.lat - pointTmp.lat) * (furLocation.lat - pointTmp.lat) + (furLocation.lon - pointTmp.lon) * (furLocation.lon - pointTmp.lon);

                        //2: left side, or  4: on the road
                        if(1 != furSideFlag)
                        {
                            if(distance < minDist)
                            {
    							minDist = distance;
                                nearFurLocation = pointTmp;
                            }
                        }
                            
                        pointListPolygon.push_back(pointTmp);
                    }

                    for(int pointListIdx = 1; pointListIdx < vectorNumInSeg; pointListIdx++)
                    {
                        ++pointListIter;
                    }
                    
                    numPoints = (*pointListIter).size();
                    // For each point
                    for(int pointIdx = numPoints-1; pointIdx >= 0; pointIdx--)
                    {
                        point3D_t pointTmp = (*pointListIter)[pointIdx];

                        double distance = (furLocation.lat - pointTmp.lat) * (furLocation.lat - pointTmp.lat) + (furLocation.lon - pointTmp.lon) * (furLocation.lon - pointTmp.lon);

                        //1: right side, or  4: on the road
                        if(2 != furSideFlag)
                        {
                            if(distance < minDist)
                            {
    							minDist = distance;
                                nearFurLocation = pointTmp;
                            }
                        }
                        
                        pointListPolygon.push_back(pointTmp);
                    }

                    bool pointInFlag = pointInPolygon(furLocation, pointListPolygon);
                    if(true == pointInFlag)
                    {
                        furnitureAttr->location = nearFurLocation;
                        furnitureAttr->location.alt = furLocation.alt;
                        furnitureAttr->location.count = furLocation.count;
                        furnitureAttr->location.paintFlag = furLocation.paintFlag;
                    }
                }
                break;
            }
            ++vecListIter;
            ++segAttrListIter;
        }
        
        pointListPolygon.clear();

        ReleaseMutex(_hMutexMemory);
    }

    void database::getAllVectorsTlv(IN resource_e sourceFlag, 
                                      OUT void** output, 
                                      OUT int32* length)
    {
        if(sourceFlag == file_e)
        {
            WaitForSingleObject(_hMutexFile,INFINITE);
        }
        else
        {
            WaitForSingleObject(_hMutexMemory,INFINITE);
        }

        tlvCommon_t tlvTmp;
        tlvCfg_t* tlvCfgP;
        int byteNum = 0;

        list<list<vector<point3D_t>>>::iterator vecListIter = _vectorList.begin();
        list<segAttributes_t>::iterator segAttrListIter = _segmentList.begin();

        // For each segment
        while(vecListIter != _vectorList.end())
        {
            if((*vecListIter).empty())
            // Segment with no vector
            {
                ++vecListIter;
                ++segAttrListIter;
                continue;
            }

            uint32 segmentIdDb = segAttrListIter->segId;

            //if(segmentIdDb == segmentIdIn)
            // Segment ID match
            {
                list<vector<point3D_t>>::iterator pointListIter = vecListIter->begin();

                // Segment ID
                tlvCfgP = &_tlvCfg_vec_a[vec_segId_e - vec_base_e];
                setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, segmentIdDb);
                byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);
                
                int vectorNumInSeg = (*vecListIter).size();

                // Vector list
                tlvCfgP = &_tlvCfg_vec_a[vec_vecList_e - vec_base_e];
                setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, vectorNumInSeg, 0);
                byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                // For each vector
                while(pointListIter != vecListIter->end())
                {
                    // Get vector info from the first element
                    /*
                    tlvCfgP = &_tlvCfg_dataLine_a[data_lineId_e - data_lineBase_e];
                    setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, attrIter->lineId);
                    byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                    tlvCfgP = &_tlvCfg_dataLine_a[data_lineWidth_e - data_lineBase_e];
                    setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, attrIter->width);
                    byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                    tlvCfgP = &_tlvCfg_dataLine_a[data_lineStyle_e - data_lineBase_e];
                    setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, attrIter->lineStyle);
                    byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                    tlvCfgP = &_tlvCfg_dataLine_a[data_lineSegVersion_e - data_lineBase_e];
                    setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, attrIter->segVersion);
                    byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);
                    */
                    int numPoints = (*pointListIter).size();

                    tlvCfgP = &_tlvCfg_dataLine_a[data_linePointList_e - data_lineBase_e];
                    setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, numPoints, 0);
                    byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);
                    
                    // For each point
                    for(int pointIdx = 0; pointIdx < numPoints; pointIdx++)
                    {
                        point3D_t pointTmp = (*pointListIter)[pointIdx];

                        tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLatitude_e - data_pointBase_e];
                        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)(&(pointTmp.lat)));
                        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                        tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLongitude_e - data_pointBase_e];
                        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)(&(pointTmp.lon)));
                        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                        tlvCfgP = &_tlvCfg_dataPoint_a[data_pointAltitude_e - data_pointBase_e];
                        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)(&(pointTmp.alt)));
                        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                        tlvCfgP = &_tlvCfg_dataPoint_a[data_pointPaintFlag_e - data_pointBase_e];
                        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, *(uint32*)(&pointTmp.paintFlag));
                        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                        tlvCfgP = &_tlvCfg_dataPoint_a[data_mergeCounter_e - data_pointBase_e];
                        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, pointTmp.count);
                        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);
                    }

                    ++pointListIter;
                }

                //break;
            }

            ++vecListIter;
            ++segAttrListIter;
        }

        *length = byteNum;

        if(sourceFlag == file_e)
        {
            ReleaseMutex(_hMutexFile);
        }
        else
        {
            ReleaseMutex(_hMutexMemory);
        }
    }

    void database::getAllVectors(list<list<vector<point3D_t>>>& allLines, list<list<lineAttributes_t>>& lineAttr)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        allLines = _vectorList;
        lineAttr = _lineAttrList;

        ReleaseMutex(_hMutexMemory);
    }

    void database::getAllVectors_clear(list<list<vector<point3D_t>>>& allLines, list<list<lineAttributes_t>>& lineAttr)
    {
        allLines.clear();
        lineAttr.clear();
    }

    void database::resetAllVectors(IN list<list<vector<point3D_t>>>& allLines, IN list<list<lineAttributes_t>>& lineAttr)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);
        
        // Clear vectors in current database
        _lineAttrList.clear();
        _vectorList.clear();

        // Push new data into database
        _vectorList = allLines;
        _lineAttrList = lineAttr;

        ReleaseMutex(_hMutexMemory);
    }

    void database::resetSegCfg(IN list<segAttributes_t> &segConfigList)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        _segmentList = segConfigList;

        ReleaseMutex(_hMutexMemory);
    }

    void database::getSegmentByGps(IN  point3D_t*       gps, 
                                   OUT uint8*           existFlag, 
                                   OUT segAttributes_t* segmentAttr)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        *existFlag = 0;
        list<segAttributes_t>::iterator segListIter = _segmentList.begin();

        // For each segment
        while(segListIter != _segmentList.end())
        {
            // TODO: locate the segment of the GPS
            if((*segListIter).numPort == 2)
            {
                double lat1 = (*segListIter).ports[0].lat;
                double lat2 = (*segListIter).ports[1].lat;
                double err = _distThreshNear;

                if(lat1 > lat2) // lat1 on north of lat2
                {
                    lat1 += err;
                    lat2 -= err;
                }
                else
                {
                    lat1 -= err;
                    lat2 += err;

                    double temp = lat1;
                    lat1 = lat2;
                    lat2 = temp;
                }

                double lon1 = (*segListIter).ports[0].lon;
                double lon2 = (*segListIter).ports[1].lon;

                if(lon1 > lon2) // lat1 on east of lat2
                {
                    lon1 += err;
                    lon2 -= err;
                }
                else
                {
                    lon1 -= err;
                    lon2 += err;

                    double temp = lon1;
                    lon1 = lon2;
                    lon2 = temp;
                }

                if((gps->lat > lat2) && (gps->lat < lat1) &&
                   (gps->lon > lon2) && (gps->lon < lon1))
                {
                    *existFlag = 1;

                    *segmentAttr = (*segListIter);
                    
                    break;
                }

            }
            else
            {
                // not supported now
                logPrintf(logLevelError_e, "DATABASE", "Not supported segment type", FOREGROUND_RED);
            }

            segListIter++;
        }

        ReleaseMutex(_hMutexMemory);
    }

	void database::getSegmentIdByGps(IN  point3D_t*       gps, 
                                     OUT uint8*           existFlag, 
                                     OUT segAttributes_t* segmentAttr)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);
		double minH = DBL_MAX;
        *existFlag = 0;
        list<segAttributes_t>::iterator segListIter = _segmentList.begin();

        // For each segment
        while(segListIter != _segmentList.end())
        {
            // TODO: locate the segment of the GPS
            double lat1 = (*segListIter).ports[0].lat;
            double lat2 = (*segListIter).ports[1].lat;

            double lon1 = (*segListIter).ports[0].lon;
            double lon2 = (*segListIter).ports[1].lon;

			// calculate angle
			double aPower2 = (gps->lat - lat1) * (gps->lat - lat1) + (gps->lon - lon1) * (gps->lon - lon1);
			double bPower2 = (gps->lat - lat2) * (gps->lat - lat2) + (gps->lon - lon2) * (gps->lon - lon2);
			double cPower2 = (lat1 - lat2) * (lat1 - lat2) + (lon1 - lon2) * (lon1 - lon2);
			double a = sqrt(aPower2); double b = sqrt(bPower2); double c = sqrt(cPower2);

			//cosA = (a^2+b^2-c^2)/2ab
			double cosA = (aPower2 + cPower2 - bPower2);
			double cosB = (bPower2 + cPower2 - aPower2);

			if( (cosA >= 0) && (cosB >= 0) ) // acute angle
			{
				// S=¡Ìp(p-a)(p-b)(p-c), p = (a+b+c)/2
				double p  = (a + b + c) / 2;
				double S = sqrt(p*(p-a)*(p-b)*(p-c));

				double h = 2 * S / c;

				if(minH > h)
				{
					minH = h;

					*existFlag = 1;

					*segmentAttr = (*segListIter);
				}
			}
			else
			{
				double minDistan = a < b ? a : b;
                if(minH > minDistan)
				{
					minH = minDistan;

					*existFlag = 1;

					*segmentAttr = (*segListIter);
				}
			}

            segListIter++;
        }

        ReleaseMutex(_hMutexMemory);
    }

    void database::getSegmentById(IN  uint32 segmentIdIn, 
                                  OUT uint8* existFlag, 
                                  OUT segAttributes_t* segmentAttr)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        *existFlag = 0;
        list<segAttributes_t>::iterator segListIter = _segmentList.begin();

        // For each segment
        while(segListIter != _segmentList.end())
        {
            // ID found
            if(((*segListIter).segId_used == 1) && 
                ((*segListIter).segId == segmentIdIn))
            {
                *existFlag = 1;
                *segmentAttr = (*segListIter);

                break;
            }

            segListIter++;
        }

        ReleaseMutex(_hMutexMemory);
    }

    void database::addSegmentTlv(IN uint8* tlvBuff, IN uint32 buffLen)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);
        
        void*  inputLoc = tlvBuff;
        void** input = &inputLoc;
        
        _mEndPosition = tlvBuff + buffLen;

        readTlvToSegment(input, memory_e);

        ReleaseMutex(_hMutexMemory);
    }

    void database::addAllVectorsInSegTlv(IN uint8* tlvBuff, IN uint32 buffLen)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        void*  inputLoc = tlvBuff;
        void** input = &inputLoc;

        _mEndPosition = tlvBuff + buffLen;

        readTlvToVector(input, memory_e);
        
        ReleaseMutex(_hMutexMemory);
    }

    /******************** File read/write methods ******************/

    int database::read(OUT void* dstP, IN int size, IN int count, IN void** srcP, IN resource_e sourceFlag)
    {
        if(sourceFlag == file_e)
        // read from file
        {
            WaitForSingleObject(_hMutexFile,INFINITE);

            FILE* fid = (FILE*)*srcP;

            int byteNum = (size * fread(dstP, size, count, fid)); // fread return count if success

            ReleaseMutex(_hMutexFile);
            return byteNum;
        }
        else if(sourceFlag == memory_e)
        // read from memory
        {
            WaitForSingleObject(_hMutexMemory,INFINITE);

            int byteNum = size * count;
            uint8* srcPTmp = (uint8*)(*srcP);

            if((uint32)(srcPTmp + byteNum) > (uint32)_mEndPosition)
            // Read to end of buffer
            {
                ReleaseMutex(_hMutexMemory);
                return 0;
            }

            memcpy(dstP, srcPTmp, byteNum);
            
            *srcP = srcPTmp + byteNum;

            ReleaseMutex(_hMutexMemory);
            return byteNum;
        }
        else
        {
            // error
            logPrintf(logLevelError_e, "DATABASE", "Unsupported source flag", FOREGROUND_RED);
            return 0;
        }
    }

    int database::write(OUT void** dstP, IN int size, IN int count, IN void* srcP, IN resource_e sourceFlag)
    {
        if(sourceFlag == file_e)
        // write to file
        {
            WaitForSingleObject(_hMutexFile,INFINITE);

            FILE* fid = (FILE*)*dstP;

            int byteNum = (size * fwrite(srcP, size, count, fid)); // fread return count if success

            ReleaseMutex(_hMutexFile);

            return byteNum;
        }
        else if(sourceFlag == memory_e)
        // write to memory
        {
            WaitForSingleObject(_hMutexMemory,INFINITE);

            int byteNum = size * count;
            uint8* dstPTmp = (uint8*)(*dstP);

            // TODO: need to assign _mEndPosition before writing to memory
            //if((uint32)(dstPTmp + byteNum) > (uint32)_mEndPosition)
            //// Write to end of buffer
            //{
            //    logPrintf(logLevelError_e, "DATABASE", "Write to end of buffer", FOREGROUND_RED);
            //    return 0;
            //}

            memcpy(dstPTmp, srcP, byteNum);
            
            *dstP = dstPTmp + byteNum;

            ReleaseMutex(_hMutexMemory);
            return byteNum;
        }
        else
        {
            // error
            logPrintf(logLevelError_e, "DATABASE", "Unsupported source flag", FOREGROUND_RED);
            return 0;
        }
    }

    void database::logPosition(IN void** input, IN resource_e sourceFlag)
    {
        if(sourceFlag == file_e)
        {
            FILE* fid = (FILE*)*input;

            fgetpos(fid, &_fPosition);
        }
        else if(sourceFlag == memory_e)
        {
            _mPosition = *input;
        }
        else
        {
            // error
            logPrintf(logLevelError_e, "DATABASE", "Unsupported source flag", FOREGROUND_RED);
            return;
        }
    }

    void database::resetPosition(IN void** input, IN resource_e sourceFlag)
    {
        if(sourceFlag == file_e)
        {
            FILE* fid = (FILE*)*input;

            fsetpos(fid, &_fPosition);
        }
        else if(sourceFlag == memory_e)
        {
            *input = _mPosition;
        }
        else
        {
            // error
            logPrintf(logLevelError_e, "DATABASE", "Unsupported source flag", FOREGROUND_RED);
            return;
        }
    }

    int32 database::writeTlvCommon(OUT tlvCommon_t* tlv,
                                   IN  void** output,
                                   IN  resource_e sourceFlag)
    {
        int numByte = 0;

        if(tlv->usedFlag != 0)
        {
            numByte += write(output, 2, 1, &(tlv->typeId), sourceFlag);

            switch(tlv->tlvType)
            {
                case tvUint8_e:
                {
                    uint8 value = (uint8)tlv->value;
                    numByte += write(output, 1, 1, &value, sourceFlag);

                    break;
                }
                case tvUint16_e:
                {
                    uint16 value = (uint16)tlv->value;
                    numByte += write(output, 2, 1, &value, sourceFlag);

                    break;
                }
                case tvUint32_e:
                {
                    numByte += write(output, 4, 1, &(tlv->value), sourceFlag);

                    break;
                }
                case tv2xUint32_e:
                {
                    logPrintf(logLevelError_e, "DATABASE", "TLV type not supported yet", FOREGROUND_RED);
                    break;
                }
                case tvHalf_e:
                {
                    logPrintf(logLevelError_e, "DATABASE", "TLV type not supported yet", FOREGROUND_RED);
                    break;
                }
                case tvSingle_e:
                {
                    numByte += write(output, 4, 1, &(tlv->value), sourceFlag);

                    break;
                }
                case tvDouble_e:
                {
                    double* valueP = (double*)tlv->value;
                    numByte += write(output, 8, 1, valueP, sourceFlag);

                    break;
                }
                case tvDateTime_e:
                {
                    logPrintf(logLevelError_e, "DATABASE", "TLV type not supported yet", FOREGROUND_RED);
                    break;
                }
                case tvPoint_e:
                {
                    logPrintf(logLevelError_e, "DATABASE", "TLV type not supported yet", FOREGROUND_RED);
                    break;
                }

                case tlvUint8s_e:
                {
                    logPrintf(logLevelError_e, "DATABASE", "TLV type not supported yet", FOREGROUND_RED);
                    break;
                }
                case tlvUint16s_e:
                {
                    logPrintf(logLevelError_e, "DATABASE", "TLV type not supported yet", FOREGROUND_RED);
                    break;
                }
                case tlvUint32s_e:
                {
                    uint32* valueP = (uint32*)tlv->value;

                    numByte += write(output, 4, 1, &(tlv->length), sourceFlag);

                    for(uint32 idx = 0; idx < tlv->length; idx++)
                    {
                        numByte += write(output, 4, 1, valueP++, sourceFlag);
                    }

                    break;
                }
                case tlvSingles_e:
                {
                    float* valueP = (float*)tlv->value;

                    numByte += write(output, 4, 1, &(tlv->length), sourceFlag);

                    for(uint32 idx = 0; idx < tlv->length; idx++)
                    {
                        numByte += write(output, 4, 1, valueP++, sourceFlag);
                    }

                    break;
                }
                case tlvPoints_e:
                {
                    numByte += write(output, 4, 1, &(tlv->length), sourceFlag);

                    break;
                }
                case tlvString_e:
                {
                    logPrintf(logLevelError_e, "DATABASE", "TLV type not supported yet", FOREGROUND_RED);
                    break;
                }
                case tlv_complexV_e:
                {
                    numByte += write(output, 4, 1, &(tlv->length), sourceFlag);

                    break;
                }
                default:
                {
                    // error
                    logPrintf(logLevelError_e, "DATABASE", "Unknown TLV type", FOREGROUND_RED);
                }
            }
        }

        return numByte;
    }

    void database::writeDbFile()
    {
        // TODO: add lock on the file when writing
#if 0
        _dbFid = fopen(_dbFileName, "wb");
        if (_dbFid == NULL)
        {
            // File could not be opened
            // error
            return;
        }

        // Write header section
        writeTlvCommon(&_headerSecHeader);

        for(int itemIdx = 0; itemIdx < _header.size(); itemIdx++)
        {
            writeTlvCommon(&_header[itemIdx]);
        }

        // Write segment section
        writeTlvCommon(&_segmentSecHeader);

        // Write vector section
        writeTlvCommon(&_vectorSecHeader);

        for(int itemIdx = 0; itemIdx < (seg_max_e - seg_base_e + 1); itemIdx++)
        {
            writetlvCommon_t(&_segment[itemIdx]);
        }

        for(int itemIdx = 0; itemIdx < (vec_max_e - vec_base_e + 1); itemIdx++)
        {
            writetlvCommon_t(&_vector[itemIdx]);
        }

        fclose(_dbFid);
#endif
    }

    void database::readTlv(IN  void** input,
                           IN  resource_e sourceFlag,
                           OUT tlvCommon_t* tlv, 
                           OUT typeId_e* idBase, 
                           OUT int* numByteRead, 
                           OUT uint8* dataBufP)
    {
        typeId_e tlvId;
        tlvCfg_t tlvCfg;

        *numByteRead = read(&tlvId, 2, 1, input, sourceFlag);

        if(*numByteRead != 0)
        {
            // Get TLV configurations by ID
            getTlvCfgbyId(tlvId, &tlvCfg, idBase);

            readTlvValue(tlvCfg.tlvType, input, sourceFlag, tlv, dataBufP);
            tlv->typeId = tlvId;
            tlv->usedFlag = 1;
            tlv->tlvType = tlvCfg.tlvType;
        }
    }

    void database::readTlvValue(IN tlvType_e tlvType, 
                                IN  void** input,
                                IN  resource_e sourceFlag,
                                OUT tlvCommon_t* tlv, 
                                OUT uint8* dataBufP)
    {
        switch(tlvType)
        {
            case tvUint8_e:
            {
                uint8 value;
                read(&value, 1, 1, input, sourceFlag);

                tlv->length = 1;
                tlv->value = (uint32)value;

                break;
            }
            case tvUint16_e:
            {
                uint16 value;
                read(&value, 2, 1, input, sourceFlag);

                tlv->length = 2;
                tlv->value = (uint32)value;

                break;
            }
            case tvUint32_e:
            {
                uint32 value;
                read(&value, 4, 1, input, sourceFlag);

                tlv->length = 4;
                tlv->value = value;

                break;
            }
            case tv2xUint32_e:
            {


                break;
            }
            case tvHalf_e:
            {

                break;
            }
            case tvSingle_e:
            {
                float value;
                read(&value, 4, 1, input, sourceFlag);

                tlv->length = 4;
                memcpy(&tlv->value, &value, 4);

                break;
            }
            case tvDouble_e:
            {
                double value;
                read(&value, 8, 1, input, sourceFlag);

                tlv->length = 8;

                memcpy(dataBufP, &value, 8);

                tlv->value = (uint32)dataBufP;

                break;
            }
            case tvDateTime_e:
            {
                break;
            }
            case tvPoint_e:
            {
                break;
            }

            case tlvUint8s_e:
            {
                break;
            }
            case tlvUint16s_e:
            {
                break;
            }
            case tlvUint32s_e:
            {
                uint32 numUint32s;
                read(&numUint32s, 4, 1, input, sourceFlag);

                tlv->length = numUint32s;
                tlv->value = (uint32)dataBufP;

                uint32* dataTmp = (uint32*)dataBufP;

                for(uint32 idx = 0; idx < numUint32s; idx++)
                {
                    read(dataTmp++, 4, 1, input, sourceFlag);
                }

                break;
            }
            case tlvSingles_e:
            {
                uint32 numSingles;
                read(&numSingles, 4, 1, input, sourceFlag);

                tlv->length = numSingles;
                tlv->value = (uint32)dataBufP;

                float* dataTmp = (float*)dataBufP;

                for(uint32 idx = 0; idx < numSingles; idx++)
                {
                    read(dataTmp++, 4, 1, input, sourceFlag);
                }
                break;
            }
            case tlvPoints_e:
            {
                uint32 numPoints;
                read(&numPoints, 4, 1, input, sourceFlag);

                tlv->length = numPoints;

                break;
            }
            case tlvString_e:
            {
                break;
            }
            case tlv_complexV_e:
            {
                uint32 length;
                read(&length, 4, 1, input, sourceFlag);

                tlv->length = length;
                tlv->value = 0;
                break;
            }
            default:
            {
                // error
                return;
            }
        }
    }

    void database::readTlvToSegment(IN  void** input,
                                    IN  resource_e sourceFlag)
    {
        // Initialize segment element
        segAttributes_t segmentElement;
        int segmentEnd = 0;
        int firstSeg = 1;

        segmentElement.segId_used = 0;
        segmentElement.version_used = 0;
        segmentElement.type_used = 0;
        segmentElement.numPort_used = 0;
        segmentElement.ports_used = 0;
        segmentElement.links_used = 0;
        segmentElement.roadLength_used = 0;
        segmentElement.bridgeFlag_used = 0;
        segmentElement.tunnelFlag_used = 0;
        segmentElement.numFurniture_used = 0;
        segmentElement.numDynamicData_used = 0;

        // Read TLV from DB file
        tlvCommon_t tlvTmp;
        typeId_e idBase;
        int numByteRead;
        
        logPosition(input, sourceFlag);

        readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);

        // Parse TLVs
        while(numByteRead)
        {
            switch(idBase)
            {
                case seg_base_e:
                {
                    switch(tlvTmp.typeId)
                    {
                        case seg_segId_e:
                        {
                            if(firstSeg == 1)
                            {
                                firstSeg = 0;
                            }
                            else
                            {
                                _segmentList.push_back(segmentElement);
                            }

                            segmentElement.segId_used = 1;
                            segmentElement.segId = tlvTmp.value;

                            break;
                        }
                        case seg_version_e:
                        {
                            segmentElement.version_used = 1;
                            segmentElement.version = tlvTmp.value;

                            break;
                        }
                        case seg_segLen_e:
                        {
                            break;
                        }
                        case seg_segType_e:
                        {
                            segmentElement.type_used = 1;
                            segmentElement.type = tlvTmp.value;

                            break;
                        }
                        case seg_segPorts_e:
                        {
                            segmentElement.numPort_used = 1;
                            segmentElement.numPort = tlvTmp.length;
                            segmentElement.ports_used = 1;

                            for(int portIdx = 0; portIdx < segmentElement.numPort; portIdx++)
                            {
                                int numBytes;
                                point3D_t point3D;

                                readTlvToPoint(input, sourceFlag, point3D, &numBytes);

                                segmentElement.ports[portIdx] = point3D;
                            }

                            break;
                        }
                        case seg_segLinks_e:
                        {
                            segmentElement.numPort_used = 1;
                            segmentElement.numPort = tlvTmp.length;
                            segmentElement.links_used = 1;

                            uint32* dataP = (uint32* )(tlvTmp.value);

                            for(int portIdx = 0; portIdx < segmentElement.numPort; portIdx++)
                            {
                                segmentElement.links[portIdx] = *dataP++;
                            }
        
                            break;
                        }
                        case seg_roadLen_e:
                        {
                            segmentElement.roadLength_used = 1;

                            int numRoadLen = tlvTmp.length;

                            float* dataP = (float* )(tlvTmp.value);

                            for(int portIdx = 0; portIdx < segmentElement.numPort; portIdx++)
                            {
                                segmentElement.roadLength[portIdx] = *dataP++;
                            }
        
                            break;
                        }
                        case seg_bridge_e:
                        {
                            segmentElement.bridgeFlag_used = 1;
                            segmentElement.bridgeFlag = tlvTmp.value;
        
                            break;
                        }
                        case seg_tunnel_e:
                        {
                            segmentElement.tunnelFlag_used = 1;
                            segmentElement.tunnelFlag = tlvTmp.value;
        
                            break;
                        }
                        case seg_furList_e:
                        {
                            segmentElement.numFurniture_used = 1;
                            segmentElement.numFurniture = tlvTmp.length;
        
                            break;
                        }
                        case seg_dynList_e:
                        {
                            segmentElement.numDynamicData_used = 1;
                            segmentElement.numDynamicData = tlvTmp.length;

                            break;
                        }
                        default:
                        {
                            // error
                            logPrintf(logLevelError_e, "DATABASE", "Not supported segment TLV", FOREGROUND_RED);
                            break;
                        }
                    }

                    break;
                }
                default:
                {
                    segmentEnd = 1;

                    break;
                }
            }

            if(segmentEnd == 1)
            {
                resetPosition(input, sourceFlag);
                break; // while
            }

            logPosition(input, sourceFlag);

            readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
        }

        _segmentList.push_back(segmentElement);
    }

    void database::readTlvToVector(IN  void** input,
                           IN  resource_e sourceFlag)
    {
        tlvCommon_t tlvTmp;
        uint32 segmentId = 0;

        vector<point3D_t> vectorElement;

        // Read TLV from DB file
        typeId_e idBase;
        int numByteRead;
        
        logPosition(input, sourceFlag);

        readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);

        // Parse TLVs
        while(numByteRead)
        {
            typeId_e typeId = tlvTmp.typeId;

            switch(idBase)
            {
                case vec_base_e:
                {
                    if(typeId == vec_segId_e)
                    {
                        segmentId = tlvTmp.value;
                    }
                    else if(typeId == vec_vecList_e)
                    {
                        list<vector<point3D_t>> vectorsInSeg;
                        list<lineAttributes_t> attrInSeg;

                        int numVec = tlvTmp.length;
                        int numPoint;

                        for(int vecIdx = 0; vecIdx < numVec; vecIdx++)
                        // Read each vector
                        {
                            int lineEnd = 0;

                            lineAttributes_t attr;

                            // Read Line attributes
                            {
                                attr.segmentId = segmentId;

                                logPosition(input, sourceFlag);
                                readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);

                                while(numByteRead)
                                {
                                    typeId_e typeId = tlvTmp.typeId;

                                    switch(idBase)
                                    {
                                        case data_lineBase_e:
                                        {
                                            if(typeId == data_linePointList_e)
                                            {
                                                attr.numPoints = tlvTmp.length;
                                                numPoint = tlvTmp.length;
                                            }else if(typeId == data_lineId_e)
                                            {
                                                attr.lineId = tlvTmp.value;
                                            }else if(typeId == data_lineWidth_e)
                                            {
                                                attr.width = tlvTmp.value;
                                            }else if(typeId == data_lineStyle_e)
                                            {
                                                attr.lineStyle = tlvTmp.value;
                                            }else if(typeId == data_lineSegVersion_e)
                                            {
                                                attr.segVersion = tlvTmp.value;
                                            }else if(typeId == data_lineSegVersion_e)
                                            {
                                                attr.segVersion = tlvTmp.value;
                                            }

                                            break;
                                        }
                                        default:
                                        {
                                            lineEnd = 1;

                                            break;
                                        }
                                    }

                                    if(lineEnd == 1)
                                    {
                                        resetPosition(input, sourceFlag);
                                        break; // while
                                    }

                                    logPosition(input, sourceFlag);
                                    readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
                                }

                                attrInSeg.push_back(attr);
                            }

                            // Read points
                            {
                                for(uint32 pointIdx = 0; pointIdx < numPoint; pointIdx++)
                                {
                                    int numBytes;
                                    point3D_t point;

                                    readTlvToPoint(input, sourceFlag, point, &numBytes);

                                    vectorElement.push_back(point);
                                }

                            }

                            vectorsInSeg.push_back(vectorElement);
                            vectorElement.clear();
                        }

                        _vectorList.push_back(vectorsInSeg);
                        _lineAttrList.push_back(attrInSeg);
                    }

                    break;
                }
                default:
                {
                    break;
                }
            }
            
            readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
        }
    }

    void database::readTlvToPoint(IN  void** input,
                                IN  resource_e sourceFlag,
                                OUT point3D_t& pointElement, 
                                OUT int* numByteInBuff)
    {
        int pointEnd = 0;
        int firstPoint = 1;

        typeId_e idBase;
        tlvCommon_t tlvTmp;
        int numByteRead;

        *numByteInBuff = 0;

        logPosition(input, sourceFlag);
        readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);

        while(numByteRead)
        {
            typeId_e typeId = tlvTmp.typeId;

            switch(idBase)
            {
                case data_pointBase_e:
                {
                    if((typeId == data_pointLatitude_e) || (typeId == data_pointDistance1_e))
                    {
                        // Break if next point
                        if(firstPoint == 0)
                        {
                            pointEnd = 1;

                            break;
                        }

                        firstPoint = 0;
                    }

                    if(typeId == data_pointAltitude_e)
                    {
                        memcpy(&pointElement.alt, _dataTmpBuf, 8);
                        *numByteInBuff += 8;
                    }else if(typeId == data_pointLatitude_e)
                    {
                        memcpy(&pointElement.lat, _dataTmpBuf, 8);
                        *numByteInBuff += 8;
                    }else if(typeId == data_pointLongitude_e)
                    {
                        memcpy(&pointElement.lon, _dataTmpBuf, 8);
                        *numByteInBuff += 8;
                    }else if(typeId == data_pointPaintFlag_e)
                    {
                        pointElement.paintFlag = *(float*)&tlvTmp.value;
                    }else if(typeId == data_mergeCounter_e)
                    {
                        pointElement.count = tlvTmp.value;
                    }else
                    // disX, disY, disZ, attributes
                    {
                        logPrintf(logLevelWarning_e, "DATABASE", "Type id in point TLV error", FOREGROUND_RED);
                    }

                    break;
                }
                default:
                {
                    // Break if not point item
                    pointEnd = 1;

                    break;
                }
            }

            if(pointEnd == 1)
            {
                resetPosition(input, sourceFlag);
                break; // while
            }

            logPosition(input, sourceFlag);
            readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
        }
    }

    void database::readTlvToFurniturePublic(IN uint8* tlvBuff, 
                                            IN uint32 buffLen,
                                            OUT furAttributes_t &furAttr)
    {
        void*  inputLoc = tlvBuff;
        void** input = &inputLoc;

        _mEndPosition = tlvBuff + buffLen;

        readTlvToFurniture(input, memory_e, &furAttr);
    }

    void database::readTlvToFurniture(IN  void** input,
                                      IN  resource_e sourceFlag,
                                      OUT furAttributes_t* furnitureElement)
    {
        // Initialize segment element
        int furnitureEnd = 0;
        int firstFurniture = 1;

        furnitureElement->segId_used = 0;
        furnitureElement->furId_used = 0;
        furnitureElement->segVersion_used = 0;
        furnitureElement->location_used = 0;
        furnitureElement->angle_used = 0;
        furnitureElement->type_used = 0;
        furnitureElement->side_used = 0;
        furnitureElement->sideFlag_used = 0;
        furnitureElement->offset_used = 0;
        furnitureElement->reliabRating_used = 0;

        // Read TLV from DB file
        tlvCommon_t tlvTmp;
        typeId_e idBase;
        int numByteRead;
        
        logPosition(input, sourceFlag);

        readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);

        // Parse TLVs
        while(numByteRead)
        {
            switch(idBase)
            {
                case fur_base_e:
                {
                    switch(tlvTmp.typeId)
                    {
                        case fur_furId_e:
                        {
                            // furniture ID is the first TLV of one furniture
                            if(firstFurniture == 0)
                            {
                                furnitureEnd = 1;
                                break;
                            }
                            firstFurniture = 0;

                            furnitureElement->furId_used = 1;
                            furnitureElement->furId = tlvTmp.value;

                            break;
                        }
                        case fur_segVer_e:
                        {
                            furnitureElement->segVersion_used = 1;
                            furnitureElement->segVersion = tlvTmp.value;

                            break;
                        }
                        case fur_location_e:
                        {
                            furnitureElement->location_used = 1;

                            int numBytes;
                            point3D_t point3D;

                            readTlvToPoint(input, sourceFlag, point3D, &numBytes);

                            furnitureElement->location = point3D;

                            break;
                        }
                        case fur_angle_e:
                        {
                            furnitureElement->angle_used = 1;
                            furnitureElement->angle = *(float*)&tlvTmp.value;

                            break;
                        }
                        case fur_type_e:
                        {
                            furnitureElement->type_used = 1;
                            furnitureElement->type = (uint16)tlvTmp.value;

                            break;
                        }
                        case fur_side_e:
                        {
                            furnitureElement->side_used = 1;

                            uint32* dataP = (uint32* )(tlvTmp.value);

                            for(int portIdx = 0; portIdx < 2; portIdx++)
                            {
                                furnitureElement->side[portIdx] = *dataP++;
                            }

                            break;
                        }
                        case fur_sideFlag_e:
                        {
                            furnitureElement->sideFlag_used = 1;
                            furnitureElement->sideFlag = (uint8)tlvTmp.value;

                            break;
                        }
                        case fur_offset_e:
                        {
                            furnitureElement->offset_used = 1;
                            furnitureElement->offset = (float)tlvTmp.value;

                            break;
                        }
                        case fur_reliabRating_e:
                        {
                            furnitureElement->reliabRating_used = 1;
                            furnitureElement->reliabRating = (uint8)tlvTmp.value;

                            break;
                        }
                        case fur_segId_e:
                        {
                            furnitureElement->segId_used = 1;
                            furnitureElement->segId = tlvTmp.value;

                            break;
                        }
                        default:
                        {
                            logPrintf(logLevelError_e, "DATABASE", "Not supported furniture TLV", FOREGROUND_RED);
                        }
                    }

                    break;
                }
                default:
                {
                    furnitureEnd = 1;

                    break;
                }
            }

            if(furnitureEnd == 1)
            {
                resetPosition(input, sourceFlag);
                break; // while
            }

            logPosition(input, sourceFlag);

            readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
        }
    }

    void database::readDb(IN  void** input,
                          IN  resource_e sourceFlag,
                          IN  int32 numByte)
    {
        initDb();

        if(sourceFlag == file_e)
        {
            WaitForSingleObject(_hMutexFile,INFINITE);

            _dbFid = fopen(_dbFileName.c_str(), "rb");
            if (_dbFid == NULL)
            {
                // File could not be opened
                logPrintf(logLevelCrit_e, "DATABASE", "DB file could not be found", FOREGROUND_RED);
                ReleaseMutex(_hMutexFile);
                return;
            }

            input = (void**)&_dbFid;
        }
        else
        {
            WaitForSingleObject(_hMutexMemory,INFINITE);

            if(numByte == 0)
            {
                ReleaseMutex(_hMutexMemory);
                return;
            }

            _mEndPosition = ((uint8*)*input) + numByte;
        }

        {
            tlvCommon_t tlvTmp;
            typeId_e idBase;
            int numByteRead;

            typeId_e status = null_e;
        
            // Read TLV from DB file
            readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);

            while(numByteRead)
            {
                typeId_e typeId = tlvTmp.typeId;

                switch(idBase)
                {
                    case secBase_e:
                    // Section start, change status
                    {
                        status = typeId;

                        if(typeId == headerSec_e)
                        {
                            _headerSecHeader = tlvTmp;
                        }
                        else if(typeId == segmentSec_e)
                        {
                            _segmentSecHeader = tlvTmp;

                            readTlvToSegment(input, sourceFlag);
                        }
                        else if(typeId == vectorSec_e)
                        {
                            _vectorSecHeader = tlvTmp;

                            readTlvToVector(input, sourceFlag);
                        }

                        break;
                    }
                    case header_base_e:
                    {
                        if(status != headerSec_e)
                        {
                            // error
                            logPrintf(logLevelWarning_e, "DATABASE", "Not in header sec", FOREGROUND_RED);
                        }

                        _header[typeId - header_base_e] = tlvTmp;

                        break;
                    }
                    default:
                    {
                        // error
                        logPrintf(logLevelWarning_e, "DATABASE", "TLV error", FOREGROUND_RED);
                        break;
                    }
                }

                readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
            }
        }

        if(sourceFlag == file_e)
        {
            fclose(_dbFid);
            ReleaseMutex(_hMutexFile);
        }
        else
        {
            ReleaseMutex(_hMutexMemory);
        }
    }

    void database::formatDbFile()
    {
        initDb();

        writeDbFile();
        
    }

    string ID2Name(int target) 
    {
        string s;
        switch (target)
        {
        case 27452:
            {
                s = "SL20";
                break;
            }
        case 27453:
            {
                s = "SL30";
                break;
            }
        case 27454:
            {
                s = "SL40";
                break;
            }
        case 27455:
            {
                s = "SL50";
                break;
            }
        case 27456:
            {
                s = "SL60";
                break;
            }
        case 28300:
            {
                s = "NO STOP";
                break;
            }
        case 28600:
            {
                s = "NO PARK";
                break;
            }
        case 20600:
            {
                s = "STOP";
                break;
            }
        case 22400:
            {
                s = "BUS STOP";
                break;
            }
        case 99900:
            {
                s = "NO VEHICLE";
                break;
            }
        case 24000:
            {
                s = "SHARE PATH";
                break;
            }
        case 23900:
            {
                s = "PED LANE";
                break;
            }
        case 22220:
            {
                s = "KEEP RIGHT";
                break;
            }
        case 23700:
            {
                s = "BICYCLE LANE";
                break;
            }
        case 31400:
            {
                s = "PARK PLACE";
                break;
            }
        case 35010:
            {
                s = "PED CROSS";
                break;
            }
        case 30600:
            {
                s = "PRI ROAD";
                break;
            }
        case 20500:
            {
                s = "GIVE WAY";
                break;
            }
        case 13100:
            {
                s = "TRA_SIG";
                break;
            }
        case 30100:
            {
                s = "RIGHT WAY";
                break;
            }
        case 12300:
            {
                s = "ROAD WORKS";
                break;
            }
        case 13810:
            {
                s = "BICYCLES";
                break;
            }
        case 10100:
            {
                s = "CAUTION";
                break;
            }
        case 13310:
            {
                s = "PED";
                break;
            }
        default:
            s = "Others";
            break;
        }
        return s;
    }
}
