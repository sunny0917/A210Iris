#ifndef __SERIAL_UTILS_H
#define __SERIAL_UTILS_H

static int fd = -1;
#define SERIAL_MAXLEN	1024

#define TAGHEX	0x55
#define ETX	0x03
#define TYPE 0x01

#define TAGLEN				3
#define TAGHEARD			5

#define MAXTXTOTAL			512

#define TAG			0
#define FRAMELEN 	1 //帧长度
#define PACK			3
#define PACKMAX		5
#define CMDTYPE 		7 //包号
#define COMMAND 	8 //命令
#define STATUS 		8 //响应状态
#define LENGTH 		9 //数据长度
#define DATA 		11 //数据起始位置

#define START				'A'
#define RECOG			'B'
#define REGISTER			'C'
#define DELET				'D'
#define READ				'E'
#define CANCLE				'F'
#define SYNC				'G'
#define GET				'H'
#define DOWN			'I'
#define APP				'J'

#define MODE				'K'
#define CARDACC			'L'
#define BUILDSTART		'M'
#define BUILD		        'N'
#define IMAGE			'O'
#define LOG				'P'
#define BUILDEND		'Q' //R

#define ACK				'S'
#define ERR				'T'

#define CONFIG			'U'
#define FEEDBACK		'V' // w x y z

int setPort(int NBits, unsigned char NEvent, int NSpeed, int NStop);
int openPort(const char* name,int NBits, char NEvent,int NSpeed,int NStop);
int closePort();
int readPort(unsigned char* readb,int maxlen);
int writePort(char *buf,int maxlen);
#endif
