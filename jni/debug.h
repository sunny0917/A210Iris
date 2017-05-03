#include <android/log.h>
#define DEBUG
#ifdef DEBUG
#define LOG_TAG "A210Iris"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define  LOGDebug(str)   __android_log_write(ANDROID_LOG_DEBUG,LOG_TAG,str)
#else
#define  LOGI(...)
#define  LOGE(...)
#define  LOGDebug(str)
#endif
