#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
//#include <error.h>
#include "common.h"
#include "serial.h"

//#define SERIAL_PORTING_DEBUG
#ifdef SERIAL_PORTING_DEBUG
#define SERIAL_DEBUG IRIS_PRINTF
#else
#define SERIAL_DEBUG
#endif

int Uart_Setopt(int sfd, int NBits, unsigned char NEvent, int NSpeed, int NStop)
{
    struct termios newtio = {0};
    struct termios oldtio = {0};

    if (tcgetattr(sfd, &oldtio) != 0)
    {
        SERIAL_DEBUG(IRIS_PF_ERROR, "SetupSerial 1\n");
        return FAIL;
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (NBits)
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

    switch (NEvent)
    {
        case 'o':
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'e':
        case 'E':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'n':
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
        case 's':
        case 'S':
            newtio.c_cflag &= ~PARENB;
            newtio.c_cflag &= ~CSTOPB;
            break;
    }

    switch (NSpeed)
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 19200:
            cfsetispeed(&newtio, B19200);
            cfsetospeed(&newtio, B19200);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        case 4000000:
            cfsetispeed(&newtio, B4000000);
            cfsetospeed(&newtio, B4000000);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }

    if (NStop == 1)
    {
        newtio.c_cflag &= ~CSTOPB;
    }
    else if (NStop ==2)
    {
        newtio.c_cflag |= CSTOPB;
    }
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 0;

    tcflush(sfd, TCIOFLUSH);
    if ((tcsetattr(sfd, TCSANOW, &newtio)) != 0)
    {
        SERIAL_DEBUG(IRIS_PF_ERROR, "com set error\n");
        return FAIL;
    }
    //SERIAL_DEBUG(IRIS_PF_INFO, "set done!\n");
    return SUCCSSE;
}

int Uart_DevInit(int *sfd, const char *name, int NBits, char NEvent, int NSpeed, int NStop)
{
    int i;
    *sfd= open(name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (*sfd == INVALL_DEV)
    {
        SERIAL_DEBUG(IRIS_PF_ERROR, "open tty fail\n");
        return FAIL;
    }
    fcntl(*sfd, F_SETFL, 0);//
    if (Uart_Setopt(*sfd, NBits, NEvent, NSpeed, NStop) < 0)
    {
        SERIAL_DEBUG(IRIS_PF_ERROR, "set_opt error\n");
        return FAIL;
    }
    //SERIAL_DEBUG(IRIS_PF_INFO, "open serial successfull\n");
    return SUCCSSE;
}


int Uart_DevRead(int sfd, unsigned char *readb, int maxlen)
{
    int nread = 0;
    int time = 0;
    int len = 0;
    int i = 0;

    struct timeval start, stop, diff;
    int gettimes = 0;
    gettimeofday(&start, 0);
    if (sfd == INVALL_DEV)
    {
        SERIAL_DEBUG(IRIS_PF_ERROR, "tty fail\n");
        return FAIL;
    }
    while (1)
    {
        gettimeofday(&stop, 0);
        tim_subtract(&diff, &start, &stop);
        if (diff.tv_sec * 1000000 + diff.tv_usec > 2000000)
        {
            //printf("read timeout\n");
            return TIMEOUT;
        }
        nread = read(sfd, readb + len, maxlen - len);
        if (nread<0)
        {
            printf("device is error\n");
            return FAIL;
        }
        else
        {
            if (nread == (maxlen - len))
            {
                SERIAL_DEBUG(IRIS_PF_INFO, "get all number:%d:%d:%d\n", maxlen, nread, len);
                return SUCCSSE;
            }
            else if (nread == 0)
            {
                usleep(20000);
                continue;
            }
            else
            {
                len += nread;
                SERIAL_DEBUG(IRIS_PF_INFO, "get %d left:%d:%d\n", len , maxlen - len, nread);
                continue;
            }
        }
    }
    return FAIL;
}

int Uart_DevClose(int sfd)
{
    if (sfd == INVALL_DEV)
    {
        SERIAL_DEBUG(IRIS_PF_ERROR, "DevClose a invall fd\n");
        return FAIL;
    }
    close(sfd);
    sfd = INVALL_DEV;
    return SUCCSSE;
}

int Uart_DevWrite(int sfd, unsigned char *writeb, int maxlen)
{
    int sendlen = 0;
    int len = 0;

    if (sfd == INVALL_DEV)
    {
        SERIAL_DEBUG(IRIS_PF_ERROR, "DevWrite a invall fd\n");
        return FAIL;
    }
    //tcflush(sfd, TCOFLUSH);
    while (1)
    {
        sendlen = write(sfd, writeb + len, maxlen - len);
        if (sendlen == -1)
        {
            return FAIL;
        }
        len += sendlen;
        if (len == maxlen)
        {
            break;
        }
    }
    return SUCCSSE;
}

int tim_subtract(struct timeval *result, struct timeval *x, struct timeval *y)
{
    if (x->tv_sec > y->tv_sec)
    {
        return -1;
    }
    if ((x->tv_sec == y->tv_sec) && (x->tv_usec > y->tv_usec))
    {
        return -1;
    }
    result->tv_sec = (y->tv_sec - x->tv_sec);
    result->tv_usec = (y->tv_usec - x->tv_usec);
    if (result->tv_usec < 0)
    {
        result->tv_sec--;
        result->tv_usec += 1000000;
    }
    return 0;
}

void time_udelay(U32 us)
{
    struct timeval start, stop, diff;
    int gettimes = 0;
    gettimeofday(&start, 0);
    while (1)
    {
        gettimeofday(&stop, 0);
        tim_subtract(&diff, &start, &stop);
        if (diff.tv_usec >= us)
        {
            return;
        }
    }
    return;
}
