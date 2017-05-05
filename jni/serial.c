#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include<pthread.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <termios.h>
#include <sys/types.h>
#include <jni.h>

#include<assert.h>

#include "debug.h"
#include "serial.h"

pthread_mutex_t queue_cs;
pthread_mutex_t queue_start;
pthread_cond_t queue_cv;
pthread_mutex_t get_lock_mux;

int getBaudrate(int baudrate) {
	switch (baudrate) {
	case 0:
		return B0;
	case 50:
		return B50;
	case 75:
		return B75;
	case 110:
		return B110;
	case 134:
		return B134;
	case 150:
		return B150;
	case 200:
		return B200;
	case 300:
		return B300;
	case 600:
		return B600;
	case 1200:
		return B1200;
	case 1800:
		return B1800;
	case 2400:
		return B2400;
	case 4800:
		return B4800;
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	case 230400:
		return B230400;
	case 460800:
		return B460800;
	case 500000:
		return B500000;
	case 576000:
		return B576000;
	case 921600:
		return B921600;
	case 1000000:
		return B1000000;
	case 1152000:
		return B1152000;
	case 1500000:
		return B1500000;
	case 2000000:
		return B2000000;
	case 2500000:
		return B2500000;
	case 3000000:
		return B3000000;
	case 3500000:
		return B3500000;
	case 4000000:
		return B4000000;
	default:
		return -1;
	}
}

int setPort(int NBits, unsigned char NEvent, int NSpeed, int NStop)
 {
	struct termios newtio;
	struct termios oldtio;

	if(tcgetattr(fd,&oldtio) != 0)
	{
		LOGE("SetupSerial 1\n");
		return -1;
	}

	bzero(&newtio,sizeof(newtio));
	newtio.c_cflag |= CLOCAL |CREAD;
	newtio.c_cflag &= ~CSIZE;

	switch(NBits)
	{
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
	}

	switch(NEvent)
	{
		case 'O':
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK | ISTRIP);
			break;
		case 'E':
			newtio.c_iflag |= (INPCK |ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
			break;
		case 'N':
			newtio.c_cflag &= ~PARENB;
			break;
	}

	switch(NSpeed)
	{
		case 2400:
			cfsetispeed(&newtio,B2400);
			cfsetospeed(&newtio,B2400);
			break;
		case 4800:
			cfsetispeed(&newtio,B4800);
			cfsetospeed(&newtio,B4800);
			break;
		case 9600:
			cfsetispeed(&newtio,B9600);
			cfsetospeed(&newtio,B9600);
			break;
		case 19200:
			cfsetispeed(&newtio,B19200);
			cfsetospeed(&newtio,B19200);
			break;
		case 115200:
			cfsetispeed(&newtio,B115200);
			cfsetospeed(&newtio,B115200);
			break;
		case 460800:
			cfsetispeed(&newtio,B460800);
			cfsetospeed(&newtio,B460800);
			break;
		default:
			cfsetispeed(&newtio,B9600);
			cfsetospeed(&newtio,B9600);
			break;
	}

	if(NStop == 1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}
	else if(NStop ==2)
	{
		newtio.c_cflag |= CSTOPB;
	}
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;

	tcflush(fd,TCIOFLUSH);
	if((tcsetattr(fd,TCSANOW,&newtio)) != 0)
	{
		LOGE("com set error\n");
		return -1;
	}
	//SERIAL_DEBUG(IRIS_PF_INFO,"set done!\n");
	return 0;
}

int openPort(const char* name,int NBits, char NEvent,int NSpeed,int NStop) {
	int i;
	fd = open(name,O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd < 0)
	{
		LOGE("open tty fail\n");
		return -1;
	}
	//fcntl(-1, F_SETFL, 0);//
	if(setPort(NBits,NEvent,NSpeed,NStop) < 0)
	{
		LOGE("set_opt error\n");
		return -1;
	}
	LOGE("open serial successfull fd = %d",fd);
	return 0;
}

int closePort() {
	if(fd < 0)
	{
		LOGE("DevClose a invall fd\n");
		return -1;
	}
	close(fd);
	fd = -1;
	return 0;
}

int readPort(unsigned char* readb,int maxlen)
{
	int nread = 0;
	int time = 0;
	int len = 0;
	int i=0;

	if(fd < 0)
	{
		LOGE("tty fail\n");
		return -1;
	}
	while(1)
	{
		nread = read(fd,readb+len,maxlen-len);
		if(nread < 0)
		{
			LOGE("device is error %d:%s",errno,strerror(errno));
			return -1;
		}
		else
		{
			//for(i = 0;i < nread;i++)
				//LOGE("readPort buf[%d]=0x%02x",i+len,readb[i+len]);
			if(nread == (maxlen-len))
			{
				LOGE("get all number:%d:%d:%d\n",maxlen,nread,len);
				return 0;
			}
			else if(nread == 0)
			{
				usleep(20000);
				continue;
			}
			else
			{
				len += nread;
				LOGE("get %d left:%d:%d\n",len,maxlen-len,nread);
				continue;
			}
		}
	}
	return -1;
}

int writePort(char *buf,int maxlen)
{
	int sendlen = 0;
	int len = 0;
	int i = 0;

	if(fd < 0)
	{
		LOGE("writePort a invall fd\n");
		return fd;
	}
	for(i = 0;i < maxlen;i++)
		LOGE("writePort buf[%d]=0x%02x",i,buf[i]);
	//tcflush(sfd,TCOFLUSH);
	while(1)
	{
		sendlen = write(fd,buf+len,maxlen-len);
		if(sendlen < 0)
			return sendlen;
		len +=sendlen;
		if(len == maxlen){
			break;
		}
	}
	LOGE("writePort success len=%d",len);
	return len;
}

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

	if(readPort(RXD,1)!=0)
	{
		LOGE("read com tag fail");
		return -1;
	}
	LOGE("comcard_read RXD[%d]=0x%02x\n",TAG,RXD[TAG]);
	if(RXD[TAG] != TAGHEX)
	{
		LOGE("read com tag is not 0x55");
		return -1;
	}
	if(readPort(RXD+1,2)!=0)
	{
		LOGE("read com framelen fail");
		return -1;
	}
	Rlen = RXD[FRAMELEN]<<8|RXD[FRAMELEN+1];
	LOGE("len is 0x%x\n",Rlen);
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
		LOGE("crc:0x%x,0x%x,%d\n",RXD[*len-2],combuild_crc(RXD,*len-2),*len);
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

#if 1
extern JavaVM* g_jvm;
extern jobject g_obj;
extern char* funcname;
int SkySendData(unsigned char cmd,char *send_data_tmp,  unsigned short nDatalen);
static void* ap_com_task(void* arg)
{
	int ret = -1;
	int i = 0;
	int len = 0;
	unsigned int packindex = 0;
	unsigned int packmax = 0;
	unsigned int infolen = 0;
	unsigned int recvLen = 0;
	unsigned char* recvData = NULL;
	unsigned char TXD[SERIAL_MAXLEN] = {0};
	unsigned char RXD[SERIAL_MAXLEN] = {0};
	char buf[BUFSIZE];

    int result = 0;
    fd_set readfd;
    struct timeval timeout;

    JNIEnv *env;
	 jclass cls;
	 jmethodID recvCallback;

	 //Attach主线程
	 if((*g_jvm)->AttachCurrentThread(g_jvm, &env, NULL) != JNI_OK)
	 {
		 LOGE("%s: AttachCurrentThread() failed", __FUNCTION__);
		 return NULL;
	 }
	 //找到对应的类
	 cls = (*env)->GetObjectClass(env,g_obj);
	 if(cls == NULL)
	 {
		 LOGE("FindClass() Error.....");
		 goto error1;
	 }
	 //再获得类中的方法
	 recvCallback = (*env)->GetStaticMethodID(env, cls, funcname, "(B[BI)V");
	 if (recvCallback == NULL)
	 {
		 LOGE("GetMethodID() Error.....");
		 goto error;
	 }
	 //最后调用java中的静态方法
     /*jbyteArray array = (*env)->NewByteArray(env,3);
     jbyte *jby =(*env)->GetByteArrayElements(env,array,0);
     memcpy(jby, &RXD[DATA], 3);
     (*env)->SetByteArrayRegion(env,array,0,3,jby);
	 (*env)->CallVoidMethod(env, cls, recvCallback,RXD[COMMAND],array,13);*/
	 LOGE("thread mOpen=%d",mOpen);
    while(mOpen){
       timeout.tv_sec = 2;
       timeout.tv_usec = 0;

       FD_ZERO(&readfd);
       FD_SET(fd,&readfd);
       LOGE("select start");
       ret = select(fd+1,&readfd,NULL,NULL,NULL);
       LOGE("select end ret=%d",ret);
       switch(ret){
       case -1:
           result = -1;
           LOGE("fd read failure");
           break;
       case 0:
    	   break;
       default:
           if(FD_ISSET(fd,&readfd))
           {
        	   pthread_mutex_lock(&get_lock_mux);
				ret = comcard_read(RXD,&len);
				if(ret < 0)
				{
					LOGE("thread comcard_read error ret = %d",ret);
		       		goto fail;
				}
				if(RXD[COMMAND] == 0x41)
				{
					pthread_mutex_lock(&queue_start);
					pthread_cond_signal(&queue_cv);
					pthread_mutex_unlock(&queue_start);
				}
				LOGE("thread read len = %d",len);
				for(i = 0;i<len;i++)
				{
				  LOGE("ap_task RXD[%d]=0x%02x ",i,RXD[i]);
				}

				if(RXD[COMMAND] == ACK)
				{
					pthread_mutex_lock(&queue_cs);
					pthread_cond_signal(&queue_cv);
					pthread_mutex_unlock(&queue_cs);
				   	pthread_mutex_unlock(&get_lock_mux);
					memset(RXD,0,SERIAL_MAXLEN);
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
					LOGE("malloc recvData");
					recvData = malloc((packmax + 1) * 512);
					memset(recvData,0x00,(packmax + 1) * 512);
				}

				memcpy(recvData + recvLen,&RXD[DATA],infolen);
				LOGE("recvLen=%d",recvLen);
				recvLen += infolen;
				LOGE("ap_task packindex=%d,packmax=%d,infolen=%d,recvLen=%d",packindex,packmax,infolen,recvLen);
				if(packindex < packmax)//发送应答包
				{
					unsigned char cmd = 0x53;
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
					LOGE("free recvData");
					recvLen = 0;
					len = 0;
					memset(RXD,0,SERIAL_MAXLEN);
					if(recvData != NULL)
					{
						free(recvData);
						recvData = NULL;
					}
				/*jbyteArray array = (*env)->NewByteArray(env,len-13);
				jbyte *jby =(*env)->GetByteArrayElements(env,array,0);
				memcpy(jby, &RXD[DATA], len-13);
				(*env)->SetByteArrayRegion(env,array,0,len-13,jby);
				(*env)->CallVoidMethod(env, cls, recvCallback,RXD[COMMAND],array,len-13);
				memset(RXD,0,sizeof(RXD));*/
			   	pthread_mutex_unlock(&get_lock_mux);
           }
           break;
       }
       if(result == -1){
           break;
       }
    }

error:
   LOGE("DeleteLocalRef");
   (*env)->DeleteLocalRef(env,cls);
error1:
	//Detach主线程
	if((*g_jvm)->DetachCurrentThread(g_jvm) != JNI_OK)
	{
		LOGE("%s: DetachCurrentThread() failed", __FUNCTION__);
	}
    LOGE("stop run!");
    return NULL;
}

int init_com_task(char* tty,int buan)
{
	int ret = -1;
	pthread_t id;

	if(init_comerial(tty,buan) == -1)
	{
		return -1;
	}

	pthread_mutex_init(&queue_cs, NULL);
	pthread_mutex_init(&queue_start, NULL);
	pthread_cond_init(&queue_cv, NULL);
	pthread_mutex_init(&get_lock_mux, NULL);

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
	int ret = -1;
    mOpen = 0;

    pthread_mutex_destroy(&queue_cs);
    pthread_mutex_destroy(&queue_start);
    pthread_cond_destroy(&queue_cv);
    pthread_mutex_destroy(&get_lock_mux);

    ret = destroy_comerial();

    return ret;
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
	sendbuf[len++] = ETX;

	LOGE("makeFrame len=%d",len);
	/*for(i = 0;i < len;i++)
		LOGE("0x%02x",sendbuf[i]);*/

	return len;
}

int SkySendData(unsigned char cmd,char *send_data_tmp,  unsigned short nDatalen)
{
	int ret = -1;
	struct timeval now;
	struct timespec outtime;
	int               rc;
	int start;
	int count = 0;
	int len = 0;
	int i = 0;
	int pack = 0;
	int maxPack = 0;
	unsigned char cmdType = 0x01;
	LOGE("SkySendData,cmd=0x%02x,nDatalen=%d",cmd,nDatalen);
	if(nDatalen/512 >= 1)
	{
		if(nDatalen%512 == 0)
			maxPack = nDatalen/512 - 1;
		else
			maxPack = nDatalen/512;
	}
	LOGE("SkySendData,maxPack=%d",maxPack);
#if 1
	for(i = 0;i < maxPack+1;i++)
	{
	reSend:
		if(maxPack == 0)// 0 <= nDatalen <= 512
			len = makeFrame(i,maxPack,cmdType,cmd,nDatalen,send_data_tmp);
		else if(nDatalen%512 == 0)// nDatalen % 512 == 0
			len = makeFrame(i,maxPack,cmdType,cmd,512,send_data_tmp+i*512);
		else{
			if(i != maxPack)
				len = makeFrame(i,maxPack,cmdType,cmd,512,send_data_tmp+i*512);
			else
				len = makeFrame(i,maxPack,cmdType,cmd,nDatalen%512,send_data_tmp+i*512);
		}
		//len = makeFrame(i,maxPack,cmdType,cmd,nDatalen,send_data_tmp+i*512);
		ret = writePort(sendbuf,len);
		if(ret < 0)
		{
			LOGE("SkySendData cmd=%02x,pack=%d,error",cmd,i);
			return ret;
		}
		if(cmd == 0x41)
		{
			pthread_mutex_lock(&queue_start);
			gettimeofday(&now, NULL);
			outtime.tv_sec = now.tv_sec + 3;
			outtime.tv_nsec = now.tv_usec * 1000;
			start = pthread_cond_timedwait(&queue_cv, &queue_start,&outtime);
			LOGE("finish timedwait start = %d",start);
			pthread_mutex_unlock(&queue_start);
			if(start == 0)
			{
				return 0;
			}
			return -1;
		}

		if(maxPack == 0 || i == maxPack)
			break;
		pthread_mutex_lock(&queue_cs);
		gettimeofday(&now, NULL);
		outtime.tv_sec = now.tv_sec + 2;
		outtime.tv_nsec = now.tv_usec * 1000;
		LOGE("wait timedwait pack = %d maxPack = %d count=%d",i,maxPack,count);
		rc = pthread_cond_timedwait(&queue_cv, &queue_cs,&outtime);
		LOGE("finish timedwait rc = %d",rc);
		pthread_mutex_unlock(&queue_cs);
		if(rc == 0)
		{
			count = 0;
			continue;
		}else{
			count++;
			LOGE("send  0x%x cmd is success in %d time,count = %d",sendbuf[COMMAND],i,count);
			if(count == 3)
				return -1;
			goto reSend;
		}
		count = 0;
	}
#endif
	return 0;
}
#endif
