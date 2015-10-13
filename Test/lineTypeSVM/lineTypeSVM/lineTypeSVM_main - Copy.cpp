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

#include "FeatureExtract.h"
#include "VisualizationApis.h"


#define ARGC_NUM           5
#define INTERVAL_NUM       100

// global parameters definition
list<list<vector<point3D_t>>>    rptData;


// function headers declaration
void usage(int argc, char* argv[]);

#if 0

int main(int argc, char* argv[])
{
    int TRAIN_CNT = 530;
    int TEST_CNT  = 104;

    cv::Mat tf = cv::Mat::zeros(TRAIN_CNT, INTERVAL_NUM, CV_32F);
    cv::Mat tl = cv::Mat::zeros(TRAIN_CNT, 1, CV_32F);

    cv::Mat cf = cv::Mat::zeros(TEST_CNT, INTERVAL_NUM, CV_32F);
    cv::Mat cl = cv::Mat::zeros(TEST_CNT, 1, CV_32F);
    cv::Mat pl = cv::Mat::zeros(TEST_CNT, 1, CV_32F);

    float tmpData = 0.0;
    float tmp[INTERVAL_NUM];
    memset(tmp, 0, sizeof(float) * INTERVAL_NUM);

    FILE *fp = NULL;
    errno_t err = 1; //fopen_s(&fp, "feature_training.txt", "r");
    if (0 == err)
    {
        int index = 0;
        while (!feof(fp))
        {
            fscanf_s(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
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
                         &tmp[ 36], &tmp[ 37], &tmp[ 38], &tmp[ 39],
                         &tmp[ 40], &tmp[ 41], &tmp[ 42], &tmp[ 43],
                         &tmp[ 44], &tmp[ 45], &tmp[ 46], &tmp[ 47],
                         &tmp[ 48], &tmp[ 49], &tmp[ 50], &tmp[ 51],
                         &tmp[ 52], &tmp[ 53], &tmp[ 54], &tmp[ 55],
                         &tmp[ 56], &tmp[ 57], &tmp[ 58], &tmp[ 59],
                         &tmp[ 60], &tmp[ 61], &tmp[ 62], &tmp[ 63],
                         &tmp[ 64], &tmp[ 65], &tmp[ 66], &tmp[ 67],
                         &tmp[ 68], &tmp[ 69], &tmp[ 70], &tmp[ 71],
                         &tmp[ 72], &tmp[ 73], &tmp[ 74], &tmp[ 75],
                         &tmp[ 76], &tmp[ 77], &tmp[ 78], &tmp[ 79],
                         &tmp[ 80], &tmp[ 81], &tmp[ 82], &tmp[ 83],
                         &tmp[ 84], &tmp[ 85], &tmp[ 86], &tmp[ 87],
                         &tmp[ 88], &tmp[ 89], &tmp[ 80], &tmp[ 81],
                         &tmp[ 92], &tmp[ 93], &tmp[ 94], &tmp[ 95],
                         &tmp[ 96], &tmp[ 97], &tmp[ 98], &tmp[ 99]);
            for (int i = 0; i < INTERVAL_NUM; i++)
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

        memset(tmp, 0, sizeof(float) * INTERVAL_NUM);
        while (!feof(fp))
        {
            fscanf_s(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
                         %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, \
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
                         &tmp[ 36], &tmp[ 37], &tmp[ 38], &tmp[ 39],
                         &tmp[ 40], &tmp[ 41], &tmp[ 42], &tmp[ 43],
                         &tmp[ 44], &tmp[ 45], &tmp[ 46], &tmp[ 47],
                         &tmp[ 48], &tmp[ 49], &tmp[ 50], &tmp[ 51],
                         &tmp[ 52], &tmp[ 53], &tmp[ 54], &tmp[ 55],
                         &tmp[ 56], &tmp[ 57], &tmp[ 58], &tmp[ 59],
                         &tmp[ 60], &tmp[ 61], &tmp[ 62], &tmp[ 63],
                         &tmp[ 64], &tmp[ 65], &tmp[ 66], &tmp[ 67],
                         &tmp[ 68], &tmp[ 69], &tmp[ 70], &tmp[ 71],
                         &tmp[ 72], &tmp[ 73], &tmp[ 74], &tmp[ 75],
                         &tmp[ 76], &tmp[ 77], &tmp[ 78], &tmp[ 79],
                         &tmp[ 80], &tmp[ 81], &tmp[ 82], &tmp[ 83],
                         &tmp[ 84], &tmp[ 85], &tmp[ 86], &tmp[ 87],
                         &tmp[ 88], &tmp[ 89], &tmp[ 80], &tmp[ 81],
                         &tmp[ 92], &tmp[ 93], &tmp[ 94], &tmp[ 95],
                         &tmp[ 96], &tmp[ 97], &tmp[ 98], &tmp[ 99]);

            for (int i = 0; i < INTERVAL_NUM; i++)
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


    // SVM definition
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

    // print output
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

    printf("matched rate: %d / %d = %lf\n", matchedCnt, TEST_CNT,
        (double)matchedCnt / TEST_CNT);

    return 0;
}


#else

int main(int argc, char *argv[])
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
    CFeatureExtract fExtObj(cfgFilename);

    // read GPS list file
    readReportData(gpsFilename, rptData);

    list<vector<float>> extData;
    fExtObj.extractFeature(rptData, extData);

    if (extData.empty())
    {
        printf("Feature vector list is empty. \n");
        return -1;
    }

    // save feature vecter list
    FILE *fp = NULL;
    errno_t err = fopen_s(&fp, "feature.txt", "w+");
    if (err == 0)
    {
        int numOfItems = 0;
        list<vector<float>>::iterator it = extData.begin();
        while (it != extData.end())
        {
            numOfItems = it->size();
            if (0 < numOfItems)
            {
                for (int i = 0; i < numOfItems - 1; ++i)
                {
                    fprintf(fp, "%.10f, ", it->at(i));
                }

                // separate last element to add new line
                fprintf(fp, "%.10f\n", it->back());
            }

            ++it;
        }

        fclose(fp);
    }

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
