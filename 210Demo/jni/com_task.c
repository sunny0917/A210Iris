#include <jni.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>
#include<pthread.h>
#include <sys/sysinfo.h>
#include <linux/input.h>
#include <unistd.h>

#include "serial.h"
#include "common.h"


int comserial = INVALL_DEV;
volatile int Opend = 0;
pthread_mutex_t queue_cs;
pthread_cond_t queue_cv;
pthread_mutex_t get_lock_mux;

extern JavaVM* g_jvm;
extern jobject g_obj;
extern char* funcname;

int SkySendData(unsigned char cmd,U8* sendBuf,  unsigned short nDatalen);

int init_comerial(const char *comName,int baudrate)
{
    //if(Uart_DevInit(&comserial,COM_DEV_NAME,8,'N',COM_DEV_BUAN,1) != SUCCSSE)
    if(Uart_DevInit(&comserial,comName,8,'N',baudrate,1) != SUCCSSE)
    {
        printf("fail open %s  \n",comName);
        return FAIL;
    }
    printf("success open %s  \n",comName);
    return SUCCSSE;
}

int uninit_comerial()
{
    if (Uart_DevClose(comserial) != SUCCSSE)
    {
        printf("fail close   \n");
        return FAIL;
    }
    printf("success close   \n");
    return SUCCSSE;
}

U8 combuild_crc(U8* TXD,U32 len)
{
    U8 BBC = 0;
    U32 i;
    for(i=0;i<len;i++)
    {
        BBC ^= TXD[i];
    }
    return ~BBC;
}

/*****get a complete pack******/
int comcard_read(U8 *RXD, int *len)
{
    U32 Rlen = 0;
    U32 dlen = 0;
    int ret = 0;
    int i = 0;

    memset(RXD, 0, SERIAL_MAXLEN);
    if (Uart_DevRead(comserial, RXD, 1)!=SUCCSSE)
    {
        //printf("read com tag fail\n");
        return FAIL;
    }
    //printf("read com tag RXD[%d]\n",RXD[TAG]);
    if (RXD[TAG] != TAGHEX)
    {
        return FAIL;
    }
    if (Uart_DevRead(comserial, RXD + 1, 2) != SUCCSSE)
    {
        return FAIL;
    }
    Rlen = RXD[FRAMELEN] << 8 | RXD[FRAMELEN + 1];
    //printf("RXD[COMMAND] is 0x%x, len is 0x%x\n", RXD[COMMAND], Rlen);
    ret = Uart_DevRead(comserial, RXD + TAGLEN, Rlen);
#if 0
    if (Rlen < 200) {
        printf("\n");
        for(i = 0; i < Rlen + TAGLEN; i++)
        {
            printf("%02x", RXD[i]);
        }
        printf("\n");
    }
#endif
    if (ret != SUCCSSE)
    {
        return FAIL;
    }

    *len = Rlen + TAGLEN;

    if (combuild_crc(RXD, *len - 2) != RXD[*len - 2])
    {
        #if 1
        printf("crc:0x%x, 0x%x, %d\n", RXD[*len - 2], combuild_crc(RXD, *len - 2), *len);
        #endif
        printf("crc is diferent\n");
        return WRONG;
    }
    if (RXD[*len - 1] != ETX)
    {
        printf("seq is diferent\n");
        return WRONG;
    }

    return SUCCSSE;
}

static void ap_com_task(void)
{
	U8 TXD[SERIAL_MAXLEN] = {0};
    U8 RXD[SERIAL_MAXLEN] = {0};
    struct timeval timeout;
    fd_set readfd;
	int ret = -1;
    int result = 0;
	int len = 0;
	unsigned int packindex = 0;
	unsigned int packmax = 0;
	unsigned int infolen = 0;
	unsigned int recvLen = 0;
	unsigned char* recvData = NULL;

	JNIEnv *env;
	jclass cls;
	jmethodID recvCallback;

	//Attach主线程
	if((*g_jvm)->AttachCurrentThread(g_jvm, &env, NULL) != JNI_OK)
	{
		IRIS_PRINTF(IRIS_PF_DEBUG, "%s: AttachCurrentThread() failed", __FUNCTION__);
		return;// NULL;
	}
	//找到对应的类
	cls = (*env)->GetObjectClass(env,g_obj);
	if(cls == NULL)
	{
		IRIS_PRINTF(IRIS_PF_DEBUG, "FindClass() Error.....");
		goto error1;
	}
	//再获得类中的方法
	recvCallback = (*env)->GetStaticMethodID(env, cls, funcname, "(B[BI)V");
	if (recvCallback == NULL)
	{
		IRIS_PRINTF(IRIS_PF_DEBUG, "GetMethodID() Error.....");
		goto error;
	}

    while(Opend)
    {
		timeout.tv_sec = 2;
		timeout.tv_usec = 0;

		FD_ZERO(&readfd);
		FD_SET(comserial,&readfd);
		IRIS_PRINTF(IRIS_PF_DEBUG, "select start");
		ret = select(comserial+1,&readfd,NULL,NULL,NULL);
		IRIS_PRINTF(IRIS_PF_DEBUG, "select end ret=%d",ret);
		switch(ret){
		case -1:
			result = -1;
			IRIS_PRINTF(IRIS_PF_DEBUG, "fd read failure");
		break;
		case 0:
		break;
		default:
			if(FD_ISSET(comserial,&readfd))
			{
				pthread_mutex_lock(&get_lock_mux);
				ret = comcard_read(RXD,&len);
				if(ret == FAIL)
				{
					//printf("read is FAIL\n");
					usleep(30000);
					continue;
				}else if(ret == WRONG)
				{
					IRIS_PRINTF(IRIS_PF_DEBUG, "read is WRONG\n");
					//com_send_msg(CMD_ERROR,1);
					usleep(30000);
					continue;
				}
				if(RXD[COMMAND] == ACK)
				{
					//printf("get ack\n");
					pthread_mutex_lock(&queue_cs);
					//printf("set signal\n");
					pthread_cond_signal(&queue_cv);
					pthread_mutex_unlock(&queue_cs);
					continue;
				}

				packindex = RXD[PACK]<<8|RXD[PACK+1];//当前包号，从0开始
				packmax = RXD[PACKMAX]<<8|RXD[PACKMAX+1];//最大包号
				infolen = RXD[LENGTH]<<8|RXD[LENGTH+1];//有效数据长度

				if(recvData == NULL || packindex == 0)
				{
					if(recvData != NULL)
					{
						free(recvData);
						recvData = NULL;
					}
					recvLen = 0;
					IRIS_PRINTF(IRIS_PF_DEBUG, "malloc recvData");
					recvData = malloc((packmax + 1) * MAXTXTOTAL);
					memset(recvData,0x00,(packmax + 1) * MAXTXTOTAL);
				}

				memcpy(recvData + recvLen,&RXD[DATA],infolen);
				IRIS_PRINTF(IRIS_PF_DEBUG, "recvLen=%d",recvLen);
				recvLen += infolen;
				IRIS_PRINTF(IRIS_PF_DEBUG, "ap_task packindex=%d,packmax=%d,infolen=%d,recvLen=%d",packindex,packmax,infolen,recvLen);
				if(packindex < packmax)//发送应答包
				{
					unsigned char cmd = ACK;
					char info = 0x00;
					int dataLen = 1;
					SkySendData(cmd,&info,dataLen);
					memset(RXD,0,SERIAL_MAXLEN);
					pthread_mutex_unlock(&get_lock_mux);
					continue;
				}

				/**发送数据**/
				jbyteArray array = (*env)->NewByteArray(env,recvLen);
				jbyte *jby =(*env)->GetByteArrayElements(env,array,0);
				memcpy(jby, recvData, recvLen);
				(*env)->SetByteArrayRegion(env,array,0,recvLen,jby);
				(*env)->CallStaticVoidMethod(env, cls, recvCallback,RXD[COMMAND],array,recvLen);
				(*env)->ReleaseByteArrayElements(env,array, jby, 0);
				(*env)->DeleteLocalRef(env,array);

			fail:
				IRIS_PRINTF(IRIS_PF_DEBUG, "free recvData");
				recvLen = 0;
				len = 0;
				memset(RXD,0,SERIAL_MAXLEN);
				if(recvData != NULL)
				{
					free(recvData);
					recvData = NULL;
				}

			   	pthread_mutex_unlock(&get_lock_mux);
			}
		break;
		}
    }

error:
IRIS_PRINTF(IRIS_PF_DEBUG, "DeleteLocalRef");
   (*env)->DeleteLocalRef(env,cls);
error1:
	//Detach主线程
	if((*g_jvm)->DetachCurrentThread(g_jvm) != JNI_OK)
	{
		IRIS_PRINTF(IRIS_PF_DEBUG, "%s: DetachCurrentThread() failed", __FUNCTION__);
	}
	IRIS_PRINTF(IRIS_PF_DEBUG, "stop run!");
	return;// NULL;
}

int send_ack(U8* TXD,int datalen)
{
    int framelen = 0;
    framelen = datalen+FRAMLEN;
    TXD[FRAMELEN] = framelen>>8&0xFF;
    TXD[FRAMELEN+1] = framelen&0xFF;

    TXD[DATA+datalen] = combuild_crc(TXD,DATA+datalen);
    TXD[TAGLEN+framelen-1] = ETX;
    //printf("send_ack\n");
    if(Uart_DevWrite(comserial,TXD,TAGLEN+framelen)!=SUCCSSE)
    {
        printf("send dvcinfo fail\n");
        return FAIL;
    }
    fsync(comserial);
}

int send_delive(U8* TXD,int datalen)
{
    int framelen = 0;
    int count = 0;
    struct timeval now;
    struct timespec outtime;
    int rc;

    framelen = datalen+FRAMLEN;
    TXD[FRAMELEN] = framelen>>8&0xFF;
    TXD[FRAMELEN+1] = framelen&0xFF;

    TXD[DATA+datalen] = combuild_crc(TXD,DATA+datalen);
    TXD[TAGLEN+framelen-1] = ETX;
    for(count = 0;count < MAXTIMERS;count++)
    {
        //printf("send_delive %d time\n",count);
        if(Uart_DevWrite(comserial,TXD,TAGLEN+framelen)!=SUCCSSE)
        {
            printf("send dvcinfo fail,errno(%d):%s\n",errno,strerror(errno));
        }else{
            return 0;
        }
        /*eTime = getTimeInMs();
        pthread_mutex_lock(&queue_cs);
        gettimeofday(&now, NULL);
        outtime.tv_sec = now.tv_sec + 2;
        outtime.tv_nsec = now.tv_usec * 1000;
        //printf("wait timedwait\n");
        rc = pthread_cond_timedwait(&queue_cv, &queue_cs, &outtime);
        //printf("finish timedwait\n");
        pthread_mutex_unlock(&queue_cs);
        printf("send wait time = %ldms,rc = %d\n",getTimeInMs()-eTime,rc);
        if(rc == 0)
        {
            //printf("send 0x%x cmd is success in %d time\n", TXD[COMMAND], count);
            return 0;
        }*/
    }

    printf("send 0x%x cmd is fail\n", TXD[COMMAND]);
    return FAIL;
}

int SkySendData(U8 cmd,U8* sendBuf,  U16 nDatalen)
{
	int ret = -1;
    int alllen = 0;
    int datalen = 0;
    int packindex = 0;
    int packmax = 0;
    int surplus = 0;
	U8 TXD[SERIAL_MAXLEN] = {0};

	TXD[TAG]            = TAGHEX;
	TXD[CMDTYPE]        = TYPE;
	TXD[COMMAND]        = cmd;

	alllen              = nDatalen;
	packindex           = 0;
	packmax             = alllen / MAXTXTOTAL;
	surplus             = alllen % MAXTXTOTAL;
	if ((packmax > 0) && (surplus == 0))
	{
		packmax--;
	}

	for (packindex = 0; packindex <= packmax; packindex++)
	{
		TXD[PACK]           = (packindex >> 8) & 0xFF;
		TXD[PACK + 1]       = packindex & 0xFF;
		TXD[PACKMAX]        = (packmax >> 8) & 0xFF;
		TXD[PACKMAX + 1]    = packmax & 0xFF;
		if ((packindex == packmax) && (surplus < MAXTXTOTAL))
		{
			datalen         = surplus;
		}
		else
		{
			datalen         = MAXTXTOTAL;
		}
		TXD[LENGTH]         = (datalen >> 8) & 0xFF;
		TXD[LENGTH + 1]     = datalen & 0xFF;

		memcpy(TXD + DATA, sendBuf + (packindex * MAXTXTOTAL), datalen);
		if (packindex != packmax)
		{
			if (-1 == send_delive(TXD, datalen))
			{
				//printf("send preview image pack(%d)fail\n", packindex);
				//break;
				return FAIL;
			}
		}
		else
		{
			return send_ack(TXD, datalen);
		}
	}

	return 0;
}

int init_com_task(const char *comName,int baudrate)
{
    int ret = 0;
    pthread_t com_task_id;

    if(init_comerial(comName,baudrate) == FAIL)
    {
        uninit_comerial();
        return -1;
    }
    pthread_mutex_init(&queue_cs, NULL);
    pthread_cond_init(&queue_cv, NULL);

    pthread_mutex_init(&get_lock_mux, NULL);
    Opend = 1;
    ret=pthread_create((pthread_t*)&com_task_id,NULL,(void*)ap_com_task,NULL);
    if(ret!=0)
    {
        printf ("Create pthread error!\n");
        return -2;
    }

    return 0;
}

int destroy_com_task()
{
	int ret = -1;
	Opend = 0;

    pthread_mutex_destroy(&queue_cs);
    pthread_cond_destroy(&queue_cv);
    pthread_mutex_destroy(&get_lock_mux);

    ret = uninit_comerial();

    return ret;
}
