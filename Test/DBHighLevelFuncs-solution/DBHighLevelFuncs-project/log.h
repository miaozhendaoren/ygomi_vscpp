#pragma once

#include <stdio.h>


#define DEBUG 1


#define WARN_IF(EXP)  do{ if (EXP) fprintf(stdout, "Warning: %s, %d: " #EXP "\n", __FUNCTION__, __LINE__); }   while(0)

#define ERR_IF(EXP)  do{ if (EXP) fprintf(stderr, "Error: %s, %d: " #EXP "\n", __FUNCTION__, __LINE__); }   while(0)

#if DEBUG
#define LOGI(format, ...) fprintf(stdout,"I: %s, %d: "format"", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define LOGI(format, ...)
#endif

#if DEBUG
#define LOGD(format, ...) fprintf(stdout,"D: %s, %d: "format"", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define LOGD(format, ...)
#endif

#define LOGW(format, ...) fprintf(stdout,"W: %s, %d: "format"", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define LOGE(format, ...) fprintf(stderr,"E: %s, %d: "format"", __FUNCTION__, __LINE__, ##__VA_ARGS__)