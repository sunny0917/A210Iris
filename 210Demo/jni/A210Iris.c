#include <jni.h>
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<pthread.h>
#include<string.h>
#include<assert.h>

#include "common.h"
#include "com_task.h"
#include "serial.h"

JavaVM *g_jvm = NULL;
jobject g_obj = NULL;
char* funcname = NULL;
pthread_mutex_t send_mutex;

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Init(JNIEnv *env, jclass jc,jstring path,jint baudrate)
{
	jboolean iscopy;
	jint ret = -1;
	const char* path_sv = (*env)->GetStringUTFChars(env,path, &iscopy);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:start---portname:%s",__func__,path_sv);
	pthread_mutex_init(&send_mutex, NULL);
	ret = init_com_task(path_sv, baudrate);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:end---ret = %d",__func__,ret);
	(*env)->ReleaseStringUTFChars(env,path,path_sv);
	return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Destory(JNIEnv *env, jclass jc)
{
    int ret = -1;
    IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
    pthread_mutex_destroy(&send_mutex);
    //ret = destroy_com_task();
    (*env)->DeleteGlobalRef(env,g_obj);
    IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Start(JNIEnv *env, jclass jc)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);

	ret = SkySendData(START,NULL,(unsigned short)0x00);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Recog(JNIEnv *env, jclass jc,jbyteArray sendBuf,jint nDatalen)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);
	jbyte * sendData = (jbyte*)(*env)->GetByteArrayElements(env,sendBuf, 0);
    jsize  size = (*env)->GetArrayLength(env,sendBuf);

	ret = SkySendData(RECOG,(U8*)sendData,(U16)nDatalen);
    (*env)->ReleaseByteArrayElements(env, sendBuf,sendData, 0);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Register(JNIEnv *env, jclass jc,jbyteArray sendBuf,jint nDatalen)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);
	jbyte * sendData = (jbyte*)(*env)->GetByteArrayElements(env,sendBuf, 0);
    jsize  size = (*env)->GetArrayLength(env,sendBuf);

	ret = SkySendData(REGISTER,(U8*)sendData,(U16)nDatalen);
    (*env)->ReleaseByteArrayElements(env, sendBuf,sendData, 0);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Delete(JNIEnv *env, jclass jc,jbyteArray sendBuf,jint nDatalen)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);
	jbyte * sendData = (jbyte*)(*env)->GetByteArrayElements(env,sendBuf, 0);
    jsize  size = (*env)->GetArrayLength(env,sendBuf);

	ret = SkySendData(DELET,(U8*)sendData,(U16)nDatalen);
    (*env)->ReleaseByteArrayElements(env, sendBuf,sendData, 0);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Read(JNIEnv *env, jclass jc,jbyteArray sendBuf,jint nDatalen)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);
	jbyte * sendData = (jbyte*)(*env)->GetByteArrayElements(env,sendBuf, 0);
    jsize  size = (*env)->GetArrayLength(env,sendBuf);

	ret = SkySendData(READ,(U8*)sendData,(U16)nDatalen);
    (*env)->ReleaseByteArrayElements(env, sendBuf,sendData, 0);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Cancle(JNIEnv *env, jclass jc)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);

	ret = SkySendData(CANCLE,NULL,(U16)0x00);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Sync(JNIEnv *env, jclass jc)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);

	ret = SkySendData(SYNC,NULL,(U16)0x00);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Get(JNIEnv *env, jclass jc,jbyteArray sendBuf,jint nDatalen)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);
	jbyte * sendData = (jbyte*)(*env)->GetByteArrayElements(env,sendBuf, 0);
    jsize  size = (*env)->GetArrayLength(env,sendBuf);

	ret = SkySendData(GET,(U8*)sendData,(U16)nDatalen);
    (*env)->ReleaseByteArrayElements(env, sendBuf,sendData, 0);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Down(JNIEnv *env, jclass jc,jbyteArray sendBuf,jint nDatalen)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);
	jbyte * sendData = (jbyte*)(*env)->GetByteArrayElements(env,sendBuf, 0);
    jsize  size = (*env)->GetArrayLength(env,sendBuf);

	ret = SkySendData(DOWN,(U8*)sendData,(U16)nDatalen);
    (*env)->ReleaseByteArrayElements(env, sendBuf,sendData, 0);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_App(JNIEnv *env, jclass jc,jbyteArray sendBuf,jint nDatalen)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);
	jbyte * sendData = (jbyte*)(*env)->GetByteArrayElements(env,sendBuf, 0);
    jsize  size = (*env)->GetArrayLength(env,sendBuf);

	ret = SkySendData(APP,(U8*)sendData,(U16)nDatalen);
    (*env)->ReleaseByteArrayElements(env, sendBuf,sendData, 0);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Mode(JNIEnv *env, jclass jc,jbyte Data,jint nDatalen)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);

	ret = SkySendData(MODE,&Data,(U16)nDatalen);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_ReadCard(JNIEnv *env, jclass jc)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);

	ret = SkySendData(CARDACC,NULL,(U16)0x00);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Image(JNIEnv *env, jclass jc)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);

	ret = SkySendData(IMAGE,NULL,(U16)0x00);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Config(JNIEnv *env, jclass jc,jbyteArray sendBuf,jint nDatalen)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);
	jbyte * sendData = (jbyte*)(*env)->GetByteArrayElements(env,sendBuf, 0);
    jsize  size = (*env)->GetArrayLength(env,sendBuf);

	ret = SkySendData(CONFIG,(U8*)sendData,(U16)nDatalen);
    (*env)->ReleaseByteArrayElements(env, sendBuf,sendData, 0);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_BuildStart(JNIEnv *env, jclass jc)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);

	ret = SkySendData(BUILDSTART,NULL,(U16)0x00);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_Build(JNIEnv *env, jclass jc)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);

	ret = SkySendData(BUILD,NULL,(U16)0x00);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_BuildEnd(JNIEnv *env, jclass jc)
{
	int ret = -1;
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----start-----\n",__func__);
	pthread_mutex_lock(&send_mutex);

	ret = SkySendData(BUILDEND,NULL,(U16)0x00);
	pthread_mutex_unlock(&send_mutex);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:-----end-----ret = %d",__func__,ret);
    return ret;
}

//由java调用来建立JNI环境
JNIEXPORT void Java_com_sykean_A210Iris_Iris_setJNIEnv( JNIEnv* env, jobject obj,jobject j_obj,jstring func)
{
	//保存全局JVM以便在子线程中使用
	(*env)->GetJavaVM(env,&g_jvm);
	//不能直接赋值(g_obj = obj)
	g_obj = (*env)->NewGlobalRef(env,j_obj);

	jsize len = (*env)->GetStringLength(env,func);
	const char* f = (*env)->GetStringUTFChars(env,func, 0);

	funcname = malloc(len+1);
	memset(funcname,0,len+1);
	memcpy(funcname,f,len);
	IRIS_PRINTF(IRIS_PF_DEBUG, "[%s]:funcname=%s,f=%s,len=%d",__func__,funcname,f,len);

	(*env)->ReleaseStringUTFChars(env, func, f);
}


 //当动态库被加载时这个函数被系统调用
 JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved)
 {
     JNIEnv* env = NULL;
     jint result = -1;

     //获取JNI版本
     if ((*vm)->GetEnv(vm, (void**)&env, JNI_VERSION_1_4) != JNI_OK)
     {
    	 IRIS_PRINTF(IRIS_PF_DEBUG, "GetEnv failed!");
         return result;
     }

     return JNI_VERSION_1_4;
 }
