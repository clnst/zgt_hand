/*******************************************************************************
* Copyright 2017 CLNST CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef ___SERIAL_DRIVER_H__
#define ___SERIAL_DRIVER_H__

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <sys/times.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define TIMEOUT_SEC(buflen,baud) (buflen*20/baud+2)             //接收超时
#define TIMEOUT_USEC 0

typedef struct{
    int         baudrate;                                       //baudrate
    int         databit;                                        //data bits, 5, 6, 7, 8
    int         fctl;                                           //flow control, 0: none, 1: hardware, 2: software
    int         parity;                                         //parity 0: none, 1: odd, 2: even
    int         stopbit;                                        //stop bits, 1, 2
    const int   reserved;                                       //reserved, must be zero
}PortInfo_t;

typedef PortInfo_t *pPortInfo_t;

/*
 * 打开串口，返回文件描述符
 * dev：设备文件名
*/
int SerialOpen(const char* dev);

/*
 * 设置串口
 * fdcom: 串口文件描述符， pportinfo： 待设置的串口信息
*/
int SerialSet(int fdcom, const pPortInfo_t pportinfo);

/*
 * 关闭串口
 * fdcom：串口文件描述符
*/
void SerialClose(int fdcom);

/*
 * 发送数据
 * fdcom：串口描述符， data：待发送数据， datalen：数据长度
 * 返回实际发送长度
*/
int SerialSend(int fdcom, const unsigned char *data, int datalen);

/*
 * 接收数据
 * fdcom：串口描述符， data：接收缓冲区, datalen.：接收长度， baudrate：波特率
 * 返回实际读入的长度
*/
int SerialRecv(int fdcom, unsigned char *data, int datalen, int baudrate);


#endif // ___SERIAL_DRIVER_H__