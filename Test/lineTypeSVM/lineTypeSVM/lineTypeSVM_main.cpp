/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  lineTypeSVM.cpp
* @brief This is the main CPP of this project to estimate line type by using
*        SVM.
*
* Change Log:
*      Date                Who             What
*      2015/09/21       Ming Chen         Create
*******************************************************************************
*/

#include "apiDataStruct.h"
#include "SecRptData.h"
#include "VisualizationApis.h"

#include <opencv2/ml/ml.hpp>

#define ARGC_NUM           5
#define SAMPLE_INTERVAL    20
#define DASH_DIST_DIFF     2.5

#define INTERVAL_LEN       5
#define INTERVAL_NUM       20
#define NORM_FACTOR        (INTERVAL_LEN * INTERVAL_NUM)


// global parameters definition
list<segAttributes_t>            segCfgList;
sectionCon                       segConfig;
list<list<vector<point3D_t>>>    rptData;
list<reportSectionData>          segData;
CSecRptData                      g_segObj;


// function headers declaration
void usage(int argc, char* argv[]);

void saveSegData(IN list<reportSectionData> &segData,
    OUT int &numOfLanes);

bool readSegCfg(IN  char *filename,
    OUT list<segAttributes_t> &segCfgList,
    OUT sectionCon            &segConfig);

void dotLineBlockIndex(IN  vector<point3D_t> &lineData,
    OUT vector<int>       &dotBlkIndexSt,
    OUT vector<int>       &dotBlkIndexEd);

void extractFeature(IN list<vector<point3D_t>> &laneData,
    OUT cv::Mat &fMat);

#if 0
int main(int argc, char* argv[])
{
    int TRAIN_CNT = 46;
    int TEST_CNT  = 46;

    cv::Mat tf = cv::Mat::zeros(TRAIN_CNT, 2 * INTERVAL_NUM, CV_32F);
    cv::Mat tl = cv::Mat::zeros(TRAIN_CNT, 1, CV_32F);

    cv::Mat cf = cv::Mat::zeros(TEST_CNT, 2 * INTERVAL_NUM, CV_32F);
    cv::Mat cl = cv::Mat::zeros(TEST_CNT, 1, CV_32F);
    cv::Mat pl = cv::Mat::zeros(TEST_CNT, 1, CV_32F);

    float tmpData = 0.0;
    float tmp[2 * INTERVAL_NUM];
    memset(tmp, 0, sizeof(float) * 2 * INTERVAL_NUM);

    FILE *fp = NULL;
    errno_t err = fopen_s(&fp, "feature_training.txt", "r");
    if (0 == err)
    {
        int index = 0;
        while (!feof(fp))
        {
            fscanf_s(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                         &tmp[  0], &tmp[  1], &tmp[  2], &tmp[  3],
                         &tmp[  4], &tmp[  5], &tmp[  6], &tmp[  7],
                         &tmp[  8], &tmp[  9], &tmp[ 10], &tmp[ 11],
                         &tmp[ 12], &tmp[ 13], &tmp[ 14], &tmp[ 15],
                         &tmp[ 16], &tmp[ 17], &tmp[ 18], &tmp[ 19],
                         &tmp[ 20], &tmp[ 21], &tmp[ 22], &tmp[ 23],
                         &tmp[ 24], &tmp[ 25], &tmp[ 26], &tmp[ 27],
                         &tmp[ 28], &tmp[ 29], &tmp[ 30], &tmp[ 31],
                         &tmp[ 32], &tmp[ 33], &tmp[ 34], &tmp[ 35],
                         &tmp[ 36], &tmp[ 37], &tmp[ 38], &tmp[ 39]);
            //&(tf.at<float>(index,  0)), &(tf.at<float>(index,  1)), &(tf.at<float>(index,  2)), &(tf.at<float>(index,  3)),
            //&(tf.at<float>(index,  4)), &(tf.at<float>(index,  5)), &(tf.at<float>(index,  6)), &(tf.at<float>(index,  7)),
            //&(tf.at<float>(index,  8)), &(tf.at<float>(index,  9)), &(tf.at<float>(index, 10)), &(tf.at<float>(index, 11)),
            //&(tf.at<float>(index, 12)), &(tf.at<float>(index, 13)), &(tf.at<float>(index, 14)), &(tf.at<float>(index, 15)),
            //&(tf.at<float>(index, 16)), &(tf.at<float>(index, 17)), &(tf.at<float>(index, 18)), &(tf.at<float>(index, 19)),
            //&(tf.at<float>(index, 20)), &(tf.at<float>(index, 21)), &(tf.at<float>(index, 22)), &(tf.at<float>(index, 23)),
            //&(tf.at<float>(index, 24)), &(tf.at<float>(index, 25)), &(tf.at<float>(index, 26)), &(tf.at<float>(index, 27)),
            //&(tf.at<float>(index, 28)), &(tf.at<float>(index, 29)), &(tf.at<float>(index, 30)), &(tf.at<float>(index, 31)),
            //&(tf.at<float>(index, 32)), &(tf.at<float>(index, 33)), &(tf.at<float>(index, 34)), &(tf.at<float>(index, 35)),
            //&(tf.at<float>(index, 36)), &(tf.at<float>(index, 37)), &(tf.at<float>(index, 38)), &(tf.at<float>(index, 39)));
            for (int i = 0; i < 2 * INTERVAL_NUM; i++)
            {
                tf.at<float>(index, i) = tmp[i];
            }

            index++;
        }

        fclose(fp);
    }


    err = fopen_s(&fp, "feature_test.txt", "r");
    if (0 == err)
    {
        int index = 0;

        memset(tmp, 0, sizeof(float) * 2 * INTERVAL_NUM);
        while (!feof(fp))
        {
            fscanf_s(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                         &tmp[  0], &tmp[  1], &tmp[  2], &tmp[  3],
                         &tmp[  4], &tmp[  5], &tmp[  6], &tmp[  7],
                         &tmp[  8], &tmp[  9], &tmp[ 10], &tmp[ 11],
                         &tmp[ 12], &tmp[ 13], &tmp[ 14], &tmp[ 15],
                         &tmp[ 16], &tmp[ 17], &tmp[ 18], &tmp[ 19],
                         &tmp[ 20], &tmp[ 21], &tmp[ 22], &tmp[ 23],
                         &tmp[ 24], &tmp[ 25], &tmp[ 26], &tmp[ 27],
                         &tmp[ 28], &tmp[ 29], &tmp[ 30], &tmp[ 31],
                         &tmp[ 32], &tmp[ 33], &tmp[ 34], &tmp[ 35],
                         &tmp[ 36], &tmp[ 37], &tmp[ 38], &tmp[ 39]);
            //&(cf.at<float>(index,  0)), &(cf.at<float>(index,  1)), &(cf.at<float>(index,  2)), &(cf.at<float>(index,  3)),
            //&(cf.at<float>(index,  4)), &(cf.at<float>(index,  5)), &(cf.at<float>(index,  6)), &(cf.at<float>(index,  7)),
            //&(cf.at<float>(index,  8)), &(cf.at<float>(index,  9)), &(cf.at<float>(index, 10)), &(cf.at<float>(index, 11)),
            //&(cf.at<float>(index, 12)), &(cf.at<float>(index, 13)), &(cf.at<float>(index, 14)), &(cf.at<float>(index, 15)),
            //&(cf.at<float>(index, 16)), &(cf.at<float>(index, 17)), &(cf.at<float>(index, 18)), &(cf.at<float>(index, 19)),
            //&(cf.at<float>(index, 20)), &(cf.at<float>(index, 21)), &(cf.at<float>(index, 22)), &(cf.at<float>(index, 23)),
            //&(cf.at<float>(index, 24)), &(cf.at<float>(index, 25)), &(cf.at<float>(index, 26)), &(cf.at<float>(index, 27)),
            //&(cf.at<float>(index, 28)), &(cf.at<float>(index, 29)), &(cf.at<float>(index, 30)), &(cf.at<float>(index, 31)),
            //&(cf.at<float>(index, 32)), &(cf.at<float>(index, 33)), &(cf.at<float>(index, 34)), &(cf.at<float>(index, 35)),
            //&(cf.at<float>(index, 36)), &(cf.at<float>(index, 37)), &(cf.at<float>(index, 38)), &(cf.at<float>(index, 39)));

            for (int i = 0; i < 2 * INTERVAL_NUM; i++)
            {
                cf.at<float>(index, i) = tmp[i];
            }

            index++;
        }

        fclose(fp);
    }


    err = fopen_s(&fp, "labels_training.txt", "r");
    if (0 == err)
    {
        int index = 0;
        while (!feof(fp))
        {
            fscanf_s(fp, "%f\n", &tmpData);

            tl.at<float>(index,  0) = tmpData;
            index++;
        }

        fclose(fp);
    }


    err = fopen_s(&fp, "labels_test.txt", "r");
    if (0 == err)
    {
        int index = 0;
        while (!feof(fp))
        {
            fscanf_s(fp, "%f\n", &tmpData);

            cl.at<float>(index,  0) = tmpData;
            index++;
        }

        fclose(fp);
    }


    //// SVM definition
    CvSVM SVM;
    CvSVMParams param;

    CvSVMParams params;
    params.svm_type    = CvSVM::C_SVC;
    params.kernel_type = CvSVM::LINEAR;
    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

    // SVM training
    SVM.train(tf, tl, Mat(), Mat(), params);

    // predict
    SVM.predict(cf, pl);

    //// print output
    int matchedCnt = 0;
    int nr = pl.rows;
    int nc = pl.cols * pl.channels();
    for (int j = 0; j < nr; ++j)
    {
        float *data_pl = pl.ptr<float>(j);
        float *data_cl = cl.ptr<float>(j);
        for (int i = 0; i < nc; ++i)
        {
            printf("#%d, #%d: %f ---> %f\n", j, i, data_cl[i], data_pl[i]);

            if (abs(data_cl[i] - data_pl[i]) < 0.00005)
            {
                ++matchedCnt;
            }
        }
    }

    printf("matched rate: %d / %d\n", matchedCnt, TEST_CNT);

    return 0;
}

#else

int main(int argc, char* argv[])
{
    if (ARGC_NUM != argc)
    {
        usage(argc, argv);
        return -1;
    }

    // parameter input parser
    int parsedArgc = 0;
    char gpsFilename[_MAX_PATH];
    char cfgFilename[_MAX_PATH];
    while (parsedArgc < argc)
    {
        if (0 == strcmp(argv[parsedArgc], "-t"))
        {
            strcpy_s(gpsFilename, _MAX_PATH, argv[++parsedArgc]);
            ++parsedArgc;
        }
        else if (0 == strcmp(argv[parsedArgc], "-c"))
        {
            strcpy_s(cfgFilename, _MAX_PATH, argv[++parsedArgc]);
            ++parsedArgc;
        }
        else
        {
            ++parsedArgc;
        }
    }

    // read section configuration data
    bool flag = readSegCfg(cfgFilename, segCfgList, segConfig);
    if (false == flag)
    {
        printf("Failed to read section configuration file %s\n", cfgFilename);
        return -1;
    }

    // read GPS list file
    readReportData(gpsFilename, rptData);

    // segment reported data
    g_segObj.segMultiRptData(rptData, SAMPLE_INTERVAL, segCfgList, segData);

    int numOfLanes = 0;
    saveSegData(segData, numOfLanes);

    cv::Mat allfMat = cv::Mat::zeros(2 * numOfLanes, 2 * INTERVAL_NUM, CV_32F);

    // extract feature vector
    int curLaneCnt = 0;
    int numOfRptData = segData.size();
    list<list<list<vector<point3D_t>>>>::iterator grpIt;
    list<list<vector<point3D_t>>>::iterator laneIt;

    list<reportSectionData>::iterator rptIt = segData.begin();
    while (rptIt != segData.end())
    {
        if (!rptIt->rptSecData.empty())
        {
            grpIt = rptIt->rptSecData.begin();
            while (grpIt != rptIt->rptSecData.end())
            {
                if (!grpIt->empty())
                {
                    laneIt = grpIt->begin();
                    while (laneIt != grpIt->end())
                    {
                        if (!laneIt->empty())
                        {
                            cv::Mat fMat = cv::Mat::zeros(2, 2 * INTERVAL_NUM, CV_32F);
                            extractFeature(*laneIt, fMat);
                            cv::Mat tMat = allfMat(cv::Rect(0, curLaneCnt, 2 * INTERVAL_NUM, 2));
                            fMat.copyTo(tMat);

                            curLaneCnt += 2;
                        }

                        laneIt++;
                    }
                }

                grpIt++;
            }
        }

        rptIt++;
    }

    // save feature matrix
    int nr = allfMat.rows;
    int nc = allfMat.cols * allfMat.channels();
    float *data = NULL;

    FILE *fp = NULL;
    errno_t err = fopen_s(&fp, "feature.txt", "w+");
    if (err == 0)
    {
        for (int jj = 0; jj < nr; ++jj)
        {
            data = allfMat.ptr<float>(jj);
            for (int ii = 0; ii < nc - 1; ++ii)
            {
                fprintf(fp, "%.10f, ", data[ii]);
            }

            // separate last element to add new line
            fprintf(fp, "%.10f\n", data[nc - 1]);
        }

        fclose(fp);
    }

    //// SVM definition
    //CvSVM SVM;
    //CvSVMParams param;

    //CvSVMParams params;
    //params.svm_type    = CvSVM::C_SVC;
    //params.kernel_type = CvSVM::LINEAR;
    //params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

    //// training data, label, test data and output Mat definition
    //cv::Mat trainingDataMat = allfMat(cv::Rect(0, 0, 2 * INTERVAL_NUM, 2 * (numOfLanes - 3)));
    //cv::Mat labelsMat = (cv::Mat_<float>(2 * numOfLanes - 6, 1) <<
    //    1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1,
    //    0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    //    0, 1, 0, 1, 0, 1, 0, 1); //, 1, 0, 1, 1, 1, 0);

    //cv::Mat outMat = cv::Mat::zeros(6, 1, CV_32F);
    //cv::Mat preMat = allfMat(cv::Rect(0, 2 * (numOfLanes - 3), 2 * INTERVAL_NUM, 6));

    //// SVM training
    //SVM.train(trainingDataMat, labelsMat, Mat(), Mat(), params);

    //// predict
    //SVM.predict(preMat, outMat);

    //// print output
    //int nr = outMat.rows;
    //int nc = outMat.cols * outMat.channels();
    //for (int j = 0; j < nr; ++j)
    //{
    //    float *data = outMat.ptr<float>(j);
    //    for (int i = 0; i < nc; ++i)
    //    {
    //        printf("#%d, #%d: %f\n", j, i, data[i]);
    //    }
    //}

    return 0;
}

#endif

void usage(int argc, char* argv[])
{
    printf("%d parameter is entered, while %d is needed\n", argc, ARGC_NUM);
    printf("Usage: lineTypeSVM.exe -t <gps file> -c <cfg file>\n");
    printf("       -t <GPS TXT file list>\n", "lineTypeSVM_main.exe");
    printf("       -c <section configuration file>\n");
    printf("\n");
}


bool readSegCfg(IN  char *filename,
    OUT list<segAttributes_t> &segCfgList,
    OUT sectionCon            &segConfig)
{
    if (NULL == filename)
    {
        return false;
    }

    int sectionNum = 0;
    int sectionID  = 0;
    int laneNum = 0;
    int a = 0, b = 0, c = 0, d = 0, e = 0;

    segAttributes_t segmentElement;
    segmentElement.segId_used          = 0;
    segmentElement.version_used        = 0;
    segmentElement.type_used           = 0;
    segmentElement.numPort_used        = 0;
    segmentElement.ports_used          = 0;
    segmentElement.links_used          = 0;
    segmentElement.roadLength_used     = 0;
    segmentElement.bridgeFlag_used     = 0;
    segmentElement.tunnelFlag_used     = 0;
    segmentElement.numFurniture_used   = 0;
    segmentElement.numDynamicData_used = 0;
    segmentElement.uiLaneNum_used      = 0;

    FILE *fp = nullptr;
    errno_t err = fopen_s(&fp, filename, "rt");
    if(0 != err)
    {
        return false;
    }

    while (!feof(fp))
    {
        fscanf_s(fp, "segId: %d,laneNum: %d\n", &sectionID, &laneNum);
        segmentElement.segId_used = 1;
        segmentElement.segId      = sectionID;
        segmentElement.uiLaneNum_used = 1;
        segmentElement.uiLaneNum      = laneNum;

        vector<int> laneType, laneConn;
        switch (laneNum)
        {
        case 1:
            fscanf_s(fp, "lineType: %d,%d\n", &a, &b);
            laneType.push_back(a);
            laneType.push_back(b);
            fscanf_s(fp, "connectInfo: %d\n", &a);
            laneConn.push_back(a);
            break;
        case 2:
            fscanf_s(fp, "lineType: %d,%d,%d\n", &a, &b, &c);
            laneType.push_back(a);
            laneType.push_back(b);
            laneType.push_back(c);
            fscanf_s(fp, "connectInfo: %d,%d\n", &a, &b);
            laneConn.push_back(a);
            laneConn.push_back(b);
            break;
        case 3:
            fscanf_s(fp, "lineType: %d,%d,%d,%d\n", &a, &b, &c, &d);
            laneType.push_back(a);
            laneType.push_back(b);
            laneType.push_back(c);
            laneType.push_back(d);
            fscanf_s(fp, "connectInfo: %d,%d,%d\n", &a, &b, &c);
            laneConn.push_back(a);
            laneConn.push_back(b);
            laneConn.push_back(c);
            break;
        case 4:
            fscanf_s(fp, "lineType: %d,%d,%d,%d,%d\n", &a, &b, &c, &d, &e);
            laneType.push_back(a);
            laneType.push_back(b);
            laneType.push_back(c);
            laneType.push_back(d);
            laneType.push_back(e);
            fscanf_s(fp, "connectInfo: %d,%d,%d,%d\n", &a, &b, &c, &d);
            laneConn.push_back(a);
            laneConn.push_back(b);
            laneConn.push_back(c);
            laneConn.push_back(d);
            break;
        }

        fscanf_s(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\n",
            &segmentElement.ports[0].lon, &segmentElement.ports[0].lat,
            &segmentElement.ports[1].lon, &segmentElement.ports[1].lat,
            &segmentElement.ports[2].lon, &segmentElement.ports[2].lat,
            &segmentElement.ports[3].lon, &segmentElement.ports[3].lat);

        segCfgList.push_back(segmentElement);
    }

    fclose(fp);
    return true;
}


void saveSegData(IN list<reportSectionData> &segData, OUT int &numOfLanes)
{
    if (segData.empty())
    {
        return;
    }

    numOfLanes = 0;

    int grpCnt = 0, laneCnt = 0;
    char filename[_MAX_PATH] = { '\0' };
    list<list<list<vector<point3D_t>>>>::iterator grpIt;
    list<list<vector<point3D_t>>>::iterator laneIt;

    list<reportSectionData>::iterator rptIt = segData.begin();
    while (rptIt != segData.end())
    {
        grpCnt = 0;
        laneCnt = 0;

        if (!rptIt->rptSecData.empty())
        {
            grpIt = rptIt->rptSecData.begin();
            while (grpIt != rptIt->rptSecData.end())
            {
                if (!grpIt->empty())
                {
                    laneIt = grpIt->begin();
                    while (laneIt != grpIt->end())
                    {
                        if (!laneIt->empty())
                        {
                            sprintf_s(filename, "section_%d_group_%d_lane_%d.png",
                                rptIt->sectionId, grpCnt, laneCnt);
                            showImage(*laneIt, Scalar(0, 255, 0), filename);

                            laneCnt++;

                            numOfLanes++;
                        }

                        laneIt++;
                    }

                    grpCnt++;
                }

                grpIt++;
            }
        }

        rptIt++;
    }

}


void dotLineBlockIndex(IN  vector<point3D_t> &lineData,
    OUT vector<int>       &dotBlkIndexSt,
    OUT vector<int>       &dotBlkIndexEd)
{
    // number of points in input line
    int numOfPoints = lineData.size();
    if (0 >= numOfPoints)
    {
        dotBlkIndexSt.clear();
        dotBlkIndexEd.clear();
        return;
    }

    // extract x, y, and paint value
    vector<double> dx, dy, dp, dd;
    vector<int> index0, index1;
    for (int i = 1; i < numOfPoints; i++)
    {
        dx.push_back(lineData[i].lon - lineData[i - 1].lon);
        dy.push_back(lineData[i].lat - lineData[i - 1].lat);

        dd.push_back(abs(dx[i - 1] + dy[i - 1]));
        if (DASH_DIST_DIFF < dd[i - 1])
        {
            index1.push_back(i - 1);
        }

        dp.push_back(lineData[i].paintFlag - lineData[i - 1].paintFlag);
        if (0 != dp[i - 1])
        {
            index0.push_back(i - 1);
        }
    }

    // dash line painting start and stop
    if (index0.empty())
    {
        dotBlkIndexSt.push_back(0);
        dotBlkIndexEd.push_back(numOfPoints - 1);
    }
    else
    {
        for (uint32 ii = 0; ii < index0.size(); ii++)
        {
            if (1.0 == dp[index0[ii]])
            {
                // start of dash painting
                dotBlkIndexSt.push_back(index0[ii] + 1);
                if ((index0.size() - 1) == ii)
                {
                    dotBlkIndexEd.push_back(numOfPoints - 1);
                }
            }
            else
            {
                // end of dash painting
                dotBlkIndexEd.push_back(index0[ii]);
                if (dotBlkIndexSt.empty())
                {
                    dotBlkIndexSt.push_back(0);
                }
            }
        }
    }

    // merge block
    int ind = 0;
    for (uint32 jj = 0; jj < index1.size(); jj++)
    {
        ind = index1[jj] + 1;

        for (uint32 ii = 0; ii < dotBlkIndexSt.size(); ii++)
        {
            if ((dotBlkIndexSt[ii] < ind) && (dotBlkIndexEd[ii] >= ind))
            {
                dotBlkIndexSt.insert(dotBlkIndexSt.begin() + ii + 1, ind);
                dotBlkIndexEd.insert(dotBlkIndexEd.begin() + ii, ind - 1);
            }
        }
    }
}


void extractFeature(IN list<vector<point3D_t>> &laneData,
    OUT cv::Mat &fMat)
{
    if (laneData.empty())
    {
        return;
    }

    vector<point3D_t> left, right;
    int numOfPnts = laneData.front().size();
    for (int i = 0; i < numOfPnts; i++)
    {
        point3D_t pnt = laneData.front().at(i);
        pnt.paintFlag = (pnt.paintFlag >= 0) ? pnt.paintFlag : 0;
        left.push_back(pnt);

        pnt = laneData.back().at(i);
        pnt.paintFlag = (pnt.paintFlag >= 0) ? pnt.paintFlag : 0;
        right.push_back(pnt);

    }

    vector<int> stInd, edInd;
    int numOfBlks = 0;
    int index = 0;
    double maxlLen = -1 * DBL_MAX, maxrLen = -1 * DBL_MAX,
        maxlSLen = -1 * DBL_MAX, maxrSLen = -1 * DBL_MAX,
        tmp = 0.0, normLen = 0.0;
    vector<double> lenL, lenSL, lenR, lenSR;

    // left one
    dotLineBlockIndex(left, stInd, edInd);
    numOfBlks = stInd.size();
    for (int i = 0; i < numOfBlks; i++)
    {
        tmp = abs(edInd[i] - stInd[i]);
        lenL.push_back(tmp);
        if (tmp > maxlLen)
        {
            maxlLen = tmp;
        }

        if (1 <= i)
        {
            tmp = abs(stInd[i] - edInd[i-1]);
            lenSL.push_back(tmp);
            if (tmp > maxlSLen)
            {
                maxlSLen = tmp;
            }
        }
    }

    for (int i = 0; i < numOfBlks; i++)
    {
        normLen = (lenL[i] - 0.005) / maxlLen * NORM_FACTOR;
        index = (int)(normLen / INTERVAL_LEN);

        fMat.at<float>(0, index) += 1.0;
    }

    for (int i = 0; i < numOfBlks - 1; i++)
    {
        normLen = (lenSL[i] - 0.005) / maxlSLen * NORM_FACTOR;
        index = (int)(normLen / INTERVAL_LEN + INTERVAL_NUM);

        fMat.at<float>(0, index) += 1.0;
    }

    // right one
    stInd.clear();
    edInd.clear();
    dotLineBlockIndex(right, stInd, edInd);
    numOfBlks = stInd.size();
    for (int i = 0; i < numOfBlks; i++)
    {
        tmp = abs(edInd[i] - stInd[i]);
        lenR.push_back(tmp);
        if (tmp > maxrLen)
        {
            maxrLen = tmp;
        }

        if (1 <= i)
        {
            tmp = abs(stInd[i] - edInd[i-1]);
            lenSR.push_back(tmp);
            if (tmp > maxrSLen)
            {
                maxrSLen = tmp;
            }
        }
    }

    for (int i = 0; i < numOfBlks; i++)
    {
        normLen = (lenR[i] - 0.005) / maxrLen * NORM_FACTOR;
        index = (int)(normLen / INTERVAL_LEN);

        fMat.at<float>(1, index) += 1;
    }

    for (int i = 0; i < numOfBlks - 1; i++)
    {
        normLen = (lenSR[i] - 0.005) / maxrSLen * NORM_FACTOR;
        index = (int)(normLen / INTERVAL_LEN + INTERVAL_NUM);

        fMat.at<float>(1, index) += 1.0;
    }
}



