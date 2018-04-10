#ifndef _COMMON_H
#define _COMMON_H
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>

typedef enum Camera_pf_level_e
{
    IRIS_PF_DEBUG = 1,   /* debug-level         */
    IRIS_PF_WARNING,     /* warning information */
    IRIS_PF_ERROR,       /* error conditions    */
    IRIS_PF_INFO ,       /* always print        */
} TP_PF_LEVEL_E;
#define CAMERA_TRACE_LEVEL IRIS_PF_INFO

#define CAMERA_DEBUG_LEVEL
#ifdef CAMERA_DEBUG_LEVEL
#define IRIS_PRINTF( level, fmt, args... ) \
    do { \
        if (level <= CAMERA_TRACE_LEVEL)\
        {\
            if(level == IRIS_PF_INFO)\
            {\
                printf("[CAMERA]%s().",__FUNCTION__); \
                printf( fmt, ## args ); \
            }\
            else if(level == IRIS_PF_ERROR)\
            {\
                printf("[CAMERA_ERR]%s().",__FUNCTION__); \
                printf( fmt, ## args ); \
            }\
            else if(level == IRIS_PF_WARNING)\
            {\
                printf("[CAMERA_WARN]%s().",__FUNCTION__); \
                printf( fmt, ## args ); \
            }\
            else if(level == IRIS_PF_DEBUG)\
            {\
                printf("[CAMERA_DBG]%s().",__FUNCTION__); \
                printf( fmt, ## args ); \
            }\
        } \
    } while (0)
#else
    #define IRIS_PRINTF( level, fmt, args... ) do{}while(0)
#endif

#define SUCCSSE             0
#define FAIL                -1
#define WRONG               -2
#define TIMEOUT             -3

typedef unsigned long       TMO;
typedef unsigned int        OSAL_ID;
typedef int                 ER;
typedef unsigned int        U32;
typedef unsigned short      U16;
typedef unsigned char       U8;

#endif
