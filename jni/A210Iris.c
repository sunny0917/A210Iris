#include <jni.h>
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<pthread.h>
#include<string.h>
#include<assert.h>
#include "debug.h"

JavaVM *g_jvm = NULL;
jobject g_obj = NULL;
char* funcname = NULL;


/*JNIEXPORT void JNICALL Java_com_sykean_A210Iris_Iris_nativeInit(JNIEnv *env, jobject obj)
{
    //保存全局JVM以便在子线程中使用
     (*env)->GetJavaVM(env,&g_jvm);
     //不能直接赋值(g_obj = obj)
     g_obj = (*env)->NewGlobalRef(env,obj);

}*/

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_openPort(JNIEnv *env, jclass jc,jstring path,jint baudrate)
{
	jboolean iscopy;

	const char* path_sv = (*env)->GetStringUTFChars(env,path, &iscopy);
	LOGI("Java_android_hardware_utils_Irsi_openPort %s",path_sv);
	jint ret = init_com_task(path_sv, baudrate);
	LOGI("Java_android_hardware_utils_Irsi_openPort ret=%d",ret);
	(*env)->ReleaseStringUTFChars(env,path,path_sv);
	return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_closePort(JNIEnv *env, jclass jc)
{
    int ret = -1;
    LOGE("Java_android_hardware_utils_Irsi_closePort");
    ret = destroy_com_task();
    (*env)->DeleteGlobalRef(env,g_obj);
    LOGE("Java_android_hardware_utils_Irsi_closePort ret = %d",ret);
    return ret;
}

JNIEXPORT jint JNICALL Java_com_sykean_A210Iris_Iris_sendData(JNIEnv *env, jclass jc,jbyte cmd,jbyteArray send_data,jint nDatalen)
{
	int ret = -1;
	LOGI("Java_android_hardware_utils_Irsi_sendData");
	jbyte * send_data_tmp = (jbyte*)(*env)->GetByteArrayElements(env,send_data, 0);
    jsize  size = (*env)->GetArrayLength(env,send_data);

	ret = SkySendData(cmd,(char *)send_data_tmp,(unsigned short)nDatalen);
    (*env)->ReleaseByteArrayElements(env, send_data,send_data_tmp, 0);
	LOGI("Java_android_hardware_utils_Irsi_sendData ret = %d",ret);
    return ret;
}

static jint Java_com_sykean_A210Iris_Iris_recvData(JNIEnv* env, jobject thiz, jbyteArray cmd, jbyteArray recv_data, jintArray recv_len)
{
    int ret = -1;
    unsigned char* cmd_tmp = NULL;
    unsigned char* recv_data_tmp = NULL;
    unsigned int* recv_len_tmp = NULL;

    cmd_tmp = (*env)->GetByteArrayElements(env, cmd, 0);
    recv_data_tmp = (*env)->GetByteArrayElements(env, recv_data, 0);
    recv_len_tmp = (*env)->GetIntArrayElements(env, recv_len, 0);

    //ret = SkyReadData(cmd_tmp,  recv_data_tmp ,recv_len_tmp);
    //LOGE("return(%d) get (%d) data\n",ret,*recv_len_tmp);
	(*env)->ReleaseByteArrayElements(env, cmd, cmd_tmp, 0);
	(*env)->ReleaseByteArrayElements(env, recv_data, recv_data_tmp, 0);
	(*env)->ReleaseIntArrayElements(env, recv_len, recv_len_tmp, 0);

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
	LOGE("funcname=%s,f=%s,len=%d",funcname,f,len);

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
         LOGE("GetEnv failed!");
             return result;
     }

     return JNI_VERSION_1_4;
 }
