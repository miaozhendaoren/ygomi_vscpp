/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  databaseDef.h
* @brief Header file for database definitions
*
* Change Log:
*      Date                Who             What
*      2015/1/7           Linkun Xu       Create
*******************************************************************************
*/

#ifndef DATABASEDEF_H
#define DATABASEDEF_H
#include "typeDefine.h"

namespace ns_database
{
    // typedefs.  Need to move to global definition
	#define MAX_NUM_PORT 10
    #define MAX_NUM_ROADLEN 45
    enum tlvType_e : uint8
    {
        tvUint8_e,
        tvUint16_e,
        tvUint32_e,
        tv2xUint32_e,
        tvHalf_e,
        tvSingle_e,
        tvDouble_e,
        tvDateTime_e,
        tvPoint_e,

        tlvUint8s_e,
        tlvUint16s_e,
        tlvUint32s_e,
        tlvHalfs_e,
        tlvSingles_e,
        tlvDoubles_e,
        tlvPoints_e,
        tlvString_e,

        tlv_complexV_e,
    };

    enum typeId_e : uint16
    {
        null_e = 0x0000,

        secBase_e    = 0x0001,
        headerSec_e  = secBase_e,
        segmentSec_e = 0x0002,
        vectorSec_e  = 0x0003,
        secMax_e,

        data_pointBase_e = 0x0051,
        data_pointLatitude_e     = data_pointBase_e,
        data_pointLongitude_e    = 0x0052,
        data_pointAltitude_e     = 0x0053,
        data_pointDistance1_e    = 0x0054,
        data_pointDistance2_e    = 0x0055,
        data_pointDistance3_e    = 0x0056,
        data_pointDeviation1_e   = 0x0057,
        data_pointDeviation2_e   = 0x0058,
        data_pointDeviation3_e   = 0x0059,
        data_pointReliabRating_e = 0x005A,
        data_pointPaintFlag_e    = 0x005B,
        data_pointLatitudeL_e    = 0x005C,
        data_pointLongitudeL_e   = 0x005D,
        data_pointAltitudeL_e    = 0x005E,
        data_pointLatitudeR_e    = 0x005F,
        data_pointLongitudeR_e   = 0x0060,
        data_pointAltitudeR_e    = 0x0061,
        data_mergeCounter_e      = 0x0062,
        data_pointMax_e,

        data_lineBase_e = 0x0101,
        data_lineId_e          = data_lineBase_e,
        data_lineWidth_e       = 0x0102,
        data_lineStyle_e       = 0x0103,
        data_lineSegVersion_e  = 0x0104,
        data_linePointList_e   = 0x0105,
        data_lineLaneId_e      = 0x0106,
        data_lineLaneWidth_e   = 0x0107,
        data_lineLaneChFlag_e  = 0x0108,
        data_linePaintFlagL_e  = 0x0109,
        data_linePaintFlagR_e  = 0x010A,
        data_lineMax_e,

        header_base_e = 0x0601,
        header_revision_e  = header_base_e,
        header_countryId_e = 0x0602,
        header_dateTime_e  = 0x0603,
        header_segIdList_e = 0x0604,
        header_fileId_e    = 0x0605,
        header_vendor_e    = 0x0606,
        header_max_e,

        seg_base_e = 0x0701,
        seg_segId_e    = seg_base_e,
        seg_version_e  = 0x0702,
        seg_segLen_e   = 0x0703,
        seg_segType_e  = 0x0704,
        seg_segPorts_e = 0x0705,
        seg_segLinks_e = 0x0706,
        seg_roadLen_e  = 0x0707,
        seg_bridge_e   = 0x0708,
        seg_tunnel_e   = 0x0709,
        seg_furList_e  = 0x070A,
        seg_dynList_e  = 0x070B,
        seg_max_e,

        vec_base_e = 0x1001,
        vec_segId_e    = vec_base_e,
        vec_vecList_e  = 0x1002,
        vec_laneNum_e  = 0x1003,
        vec_segNum_e   = 0x1004,
        vec_max_e,

        fur_base_e = 0x2001,
        fur_furId_e        = fur_base_e,
        fur_segVer_e       = 0x2002,
        fur_location_e     = 0x2003,
        fur_angle_e        = 0x2004,
        fur_type_e         = 0x2005,
        fur_side_e         = 0x2006,
        fur_sideFlag_e     = 0x2007,
        fur_offset_e       = 0x2008,
        fur_reliabRating_e = 0x2009,
        fur_segId_e        = 0x200A,
        fur_max_e,

        // Dynamic data start from 0x3001


    };

        
}


#endif
