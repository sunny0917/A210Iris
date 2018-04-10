#ifndef _SERIAL_H_
#define _SERIAL_H_
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include "common.h"

#define INVALL_DEV      -1
#define MATCHDIR        "/usr/data"

//#define _REG_SAVE_TMP_DISABLE_
//#define _VERSION_NEW_RULE_
#define _USE_USB_COM_

#ifdef _USE_USB_COM_
#define COM_DEV_NAME    "/dev/ttyGS0"
#define COM_DEV_BUAN    4000000//460800
#else
#define COM_DEV_NAME    "/dev/ttyS1"
#define COM_DEV_BUAN    115200//
#endif
#define _Double_Light_
#define SERIAL_MAXLEN   1024
//#define __BUTTON_ENABLE__
#define SERIAL_MAXLEN_EXTEND    409600
#define _FB_SHOW_
#define WINDOW_W        1920
#define WINDOW_H        960
#define WINDOW_SIZE     (WINDOW_W * WINDOW_H)

#if 0
#define CMD_RUN_START                   0
//#define CMD_START_RECOG               1
#define CMD_RECOG_SUCCESS               2
#define CMD_RECOG_FAIL                  3
//#define CMD_START_REGISTER            4
#define CMD_REGISTER_SUCCESS            5
#define CMD_REGISTER_FAIL               6
//#define CMD_DELET_IRIS                7
#define CMD_DELET_SUCCESS               8
#define CMD_DELET_FAIL                  9
//#define CMD_READ_USER                 10
#define CMD_READ_SUCCESS                11
//#define CMD_CANCLE                    12
#define CMD_CANCLE_SUCCESS              13
#define CMD_SYNC                        14

#define CMD_GET_FAIL                    15
#define CMD_GET_SUCCESS                 16

#define CMD_DOWN_FAIL                   17
#define CMD_DOWN_SUCCESS                18

#define CMD_APP_FAIL                    19
#define CMD_APP_SUCCESS                 20

#endif

#define START               'A'
#define RECOG               'B'
#define REGISTER            'C'
#define DELET               'D'
#define READ                'E'
#define CANCLE              'F'
#define SYNC                'G'
#define GET                 'H'
#define DOWN                'I'
#define APP                 'J'

#define MODE                'K'
#define CARDACC             'L'
#define BUILDSTART          'M'
#define BUILD               'N'
#define IMAGE               'O'
#define LOG                 'P'
#define BUILDEND            'Q' //R
#define ACK                 'S'
#define ERR                 'T'

#define CONFIG              'U'
#define FEEDBACK            'V'

#define PREVIEW             'W'
#define IRISLEFT            'X'
#define IRISRIGHT           'Y'

#define RECOG_ADJUST        0x70


#define TAGHEX              0x55
#define ETX                 0x03
#define TYPE                0x01

#define TAGLEN              3
#define TAGHEARD            5

#define MAXTXTOTAL          512

#define MAXTIMERS           3
#define FRAMLEN             10
#define USERLEN             24
#define VERSIONLEN          4
#define IRIS_LEN            1024
#define PVW_WIDTH           320
#define PVW_HEIGHT          160
#define PVW_LEN             (PVW_WIDTH * PVW_HEIGHT)
#define BMP_WIDTH           640 // 360
#define BMP_HEIGHT          480 // 360
#define BMP_LEN             (BMP_WIDTH * BMP_HEIGHT)
#define BMP_HEAD            (54 + 1024)
#define VERSION             0x10000221
#ifdef _VERSION_NEW_RULE_
#define VERSION_NEW         "SW_ELF01A_YAF_V2.21"
#define VERSION_NEW_LEN     strlen(VERSION_NEW)
#define VERSION_SCAN_FORMAT "SW_%[^_]_%[^_]_V%d.%d"
#define VERSION_SCAN_LEN    32
#endif

#define TAG             0
#define FRAMELEN        1 //帧长度
#define PACK            3
#define PACKMAX         5
#define CMDTYPE         7 //包号
#define COMMAND         8 //命令
#define STATUS          8 //响应状态
#define LENGTH          9 //数据长度
#define DATA            11 //数据起始位置

#define CARD_LEN        4 //+x

typedef enum{
    CMD_RUN_START               =   0,
    CMD_RECOG_SUCCESS           =   2,
    CMD_RECOG_FAIL              =   3,
    CMD_RECOG_ADJUST            =   4,
    CMD_REGISTER_SUCCESS        =   5,
    CMD_REGISTER_FAIL           =   6,
    CMD_REGISTER_ADJUST         =   7,
    CMD_DELET_SUCCESS           =   8,
    CMD_DELET_FAIL              =   9,
    CMD_READ_SUCCESS            =   11,
    CMD_READ_FAIL               =   12,
    CMD_CANCLE_SUCCESS          =   13,
    CMD_SYNC                    =   14,
    CMD_GET_FAIL                =   15,
    CMD_GET_SUCCESS             =   16,
    CMD_DOWN_FAIL               =   17,
    CMD_DOWN_SUCCESS            =   18,
    CMD_APP_FAIL                =   19,
    CMD_APP_SUCCESS             =   20,
    CMD_ACK                     =   21,
    CMD_ERROR                   =   22,
    CMD_CONFIG                  =   23,
    CMD_MODE_SUCCESS            =   24,
    CMD_CARD_FAIL               =   25,
    CMD_CARD_SUCCESS            =   26,
    CMD_CARD_MATCHSUCCESS       =   27,
    CMD_CARD_MATCHFAIL          =   28,
    CMD_CAPTURE_SUCCESS         =   29,
    CMD_FEEDBACK                =   30,
    CMD_IRIS_GET_FAIL           =   31,
    CMD_IRIS_GET_SUCCESS        =   32,
    CMD_IRIS_CANCLE_SUCCESS     =   33,
    CMD_IRIS_START_SUCCESS      =   34,
    CMD_IRIS_LEFT_BMP_FAIL      =   35,
    CMD_IRIS_LEFT_BMP_SUCCESS   =   36,
    CMD_IRIS_RIGHT_BMP_FAIL     =   37,
    CMD_IRIS_RIGHT_BMP_SUCCESS  =   38,
    CMD_PREVIEW_BMP_FAIL        =   39,
    CMD_PREVIEW_BMP_SUCCESS     =   40,
} ComMsgType_t;

typedef struct
{
    ComMsgType_t msgType;
    U32 msgCode;
} ComMsg_t,  *pComMsg_t;

int com_send_msg(ComMsgType_t msg_type, U32 msg_code);
int Uart_Setopt(int sfd,int NBits, unsigned char NEvent, int NSpeed, int NStop);
int Uart_DevInit(int* sfd,const char* name,int NBits, char NEvent,int NSpeed,int NStop);
int Uart_DevRead(int sfd,unsigned char* readb,int maxlen);
int Uart_DevClose(int sfd);
int Uart_DevWrite(int sfd,unsigned char* writeb,int maxlen);
void time_udelay(U32 us);

#endif
