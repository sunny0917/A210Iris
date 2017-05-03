#include <jni.h>
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<pthread.h>
#include<string.h>
#include<assert.h>

#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <termios.h>
#include <sys/types.h>

#include<assert.h>

#include "debug.h"
#include "serial.h"


#define BUFSIZE		1024
static int mOpen = 0;

char sendbuf[BUFSIZE];
char recvbuf[BUFSIZE];

int init_comerial(char* tty,int buan)
{
	if(openPort(tty,8,'N',buan,1) != 0)
	{
		LOGE("fail open %s  \n",tty);
		return -1;
	}
	LOGE("success open %s  \n",tty);
	return 0;
}

int destroy_comerial()
{
	if(closePort() != 0)
	{
		printf("fail close \n");
		return -1;
	}
	printf("success close \n");
	return 0;
}

unsigned char combuild_crc(unsigned char* TXD,unsigned int len)
{
	unsigned char BBC = 0;
	unsigned int i;
	for(i=0;i<len;i++)
	{
		BBC ^= TXD[i];
	}
	return ~BBC;
}

/*****get a complete pack******/
int comcard_read(unsigned char* RXD,int* len)
{
	unsigned int Rlen = 0;
	unsigned int dlen = 0;
	int ret=0;
	int i;
	memset(RXD,0,SERIAL_MAXLEN);
	if(readPort(RXD,1)!=0)
	{
		LOGE("read com tag fail\n");
		return -1;
	}
	//printf("read com tag RXD[%d]\n",RXD[TAG]);
	if(RXD[TAG] != TAGHEX)
	{
		return -1;
	}
	if(readPort(RXD+1,2)!=0)
	{
		return -1;
	}
	Rlen = RXD[FRAMELEN]<<8|RXD[FRAMELEN+1];
	//printf("RXD[COMMAND] is 0x%x,len is 0x%x\n",RXD[COMMAND],Rlen);
	ret = readPort(RXD+TAGLEN,Rlen);
#if 0
	if(Rlen<200){
	printf("\n");
	for(i=0;i<Rlen+TAGLEN;i++)
	{
		printf("%02x",RXD[i]);
	}
	printf("\n");
	}
#endif
	if(ret!=0)
	{
		return -1;
	}

	*len = Rlen+TAGLEN;

	if(combuild_crc(RXD,*len-2)!= RXD[*len-2])
	{
		#if 1
		LOGE("crc:0x%x,0x%x,%d\n",RXD[*len-2],combuild_crc(RXD,*len-2),*len);
		#endif
		LOGE("crc is diferent\n");
		return -2;
	}
	if(RXD[*len-1] != ETX)
	{
		printf("seq is diferent\n");
		return -2;
	}

	return 0;
}

extern JavaVM* g_jvm;
extern jobject g_obj;
static void* ap_com_task(void* arg)
{
	int ret = -1;
	int i = 0;
	int len = 0;
	unsigned char TXD[SERIAL_MAXLEN] = {0};
	unsigned char RXD[SERIAL_MAXLEN] = {0};
	char buf[BUFSIZE];

    int result = 0;
    fd_set readfd;
    struct timeval timeout;
	LOGE("run read data");
    /*JNIEnv *env;
    jclass class;
    jmethodID recvCallback;
    (*g_jvm)->AttachCurrentThread(g_jvm,&env,NULL);
    class = (*env)->GetObjectClass(env,g_obj);
    if(class == NULL)
    {
    	LOGE("Find class error");
    	return;
    }
    recvCallback = (*env)->GetMethodID(env,class,"recvCallBack","(C[BI)V");
    if(recvCallback == NULL)
    {
    	LOGE("find method error");
    	return;
    }*/
    while(mOpen){
       timeout.tv_sec = 2;
       timeout.tv_usec = 0;

       FD_ZERO(&readfd);
       FD_SET(fd,&readfd);
       LOGE("select");
       ret = select(fd+1,&readfd,NULL,NULL,&timeout);
       LOGE("select end");

       switch(ret){
       case -1:
           result = -1;
           LOGE("mTtyfd read failure");
           break;
       case 0:
           break;
       default:
           if(FD_ISSET(fd,&readfd)){
              //int len = read(fd,buf,sizeof(buf));
        	  ret = comcard_read(RXD,&len);
              LOGI("thread read len = %d",len);
              for(i = 0;i<len;i++)
              {
            	  printf("0x%02x ",RXD[i]);
            	  if(i%10 == 0)
            		  printf("\n");
              }
              /**·¢ËÍÊý¾Ý**/
             // if(!(arg))break;
              //JNIMyObserver *l = static_cast<JNIMyObserver *>(arg);
              //l->OnEvent(buf,len,RECEIVE_DATA_INDEX);
              //postEvent(buf,len,RECEIVE_DATA_INDEX);
              //(*env)->CallVoidMethod(env,g_obj,recvCallback,RXD[COMMAND],RXD[DATA],len);
              memset(RXD,0,sizeof(RXD));
           }
           break;
       }
       if(result == -1){
           break;
       }
    }
    LOGE("stop run!");
    return NULL;
}

int init_com_task(char* tty,int buan)
{
	int ret = 0;
	pthread_t id;

	if(init_comerial(tty,buan) == -1)
	{
		return -1;
	}

	mOpen = 1;
	LOGE("init_com_task");
	ret = pthread_create(&id,NULL,(void *)ap_com_task,NULL);
	if(ret!=0)
	{
		LOGE("Create pthread error!\n");
		return -1;
	}
	return 0;
}

int destroy_com_task()
{
    mOpen = 0;
    destroy_comerial();
}

int makeFrame(short pack,short maxPack,char cmdType,char cmd,short infoLen,char *info)
{
	int len = 0;
	int i = 0;
	char bbc = 0x00;
	memset(sendbuf,0,BUFSIZE);
	sendbuf[len++] = TAGHEX;
	sendbuf[len++] = ((infoLen + 10) & 0xff00) >> 8;
	sendbuf[len++] = (infoLen + 10) & 0x00ff;
	sendbuf[len++] = (pack & 0xff00) >> 8;
	sendbuf[len++] = pack & 0x00ff;
	sendbuf[len++] = (maxPack & 0xff00) >> 8;
	sendbuf[len++] = maxPack & 0x00ff;
	sendbuf[len++] = cmdType;
	sendbuf[len++] = cmd;
	sendbuf[len++] = (infoLen & 0xff00) >> 8;
	sendbuf[len++] = infoLen & 0x00ff;

	for(i = 0;i < infoLen;i++)
		sendbuf[len++] =info[i];

	for(i=0;i< len;i++)
	{
		bbc ^= sendbuf [i];
	}
	sendbuf[len++] = ~bbc;
	sendbuf[len] = ETX;


	return len;
}

int SkySendData(unsigned char cmd,char *send_data_tmp,  unsigned short nDatalen)
{
	int ret = -1;
	int len = 0;
	int i = 0;
	int pack = 0;
	int maxPack = 0;
	unsigned char cmdType = 0x01;

	if(nDatalen/512 >= 1)
	{
		if(nDatalen%512 == 0)
			maxPack = nDatalen/512;
		else
			maxPack = nDatalen/512 + 1;
	}

	for(i = 0;i < maxPack;i++)
	{
		len = makeFrame(i,maxPack,cmdType,cmd,nDatalen,send_data_tmp);
		ret = writePort(sendbuf,len);
		if(ret < 0)
		{
			LOGE("SkySendData cmd=%02x,pack=%d,error",cmd,i);
		}
	}
	return 0;
}
