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

#ifndef ___ZGT_CTRL_H__
#define ___ZGT_CTRL_H__

#include <ros/ros.h>
#include <ros/time.h>
#include <zgt_bringup/serial_driver.h>

#define CMD_MAX         7
#define TX_MAX          12
#define BAUDRATE        115200
#define C_NULL          0x00

const unsigned char cmd_tbl[CMD_MAX][3] = {
    {0x00, 0xBD, 0x42},   //五指张开
    {0x01, 0xBE, 0x41},   //五指合并
    {0x03, 0xC0, 0x3F},   //抓取卡片
    {0x04, 0xC1, 0x3E},   //顺序闭合
    {0x05, 0xC2, 0x3D},   //顺序张开
    {0x06, 0xC3, 0x3C},   //剪刀手
    {0x07, 0xC4, 0x3B},   //OK手势
};

unsigned char tx_buffer[] = {0xAB, 0x00, 0x11, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, C_NULL, C_NULL, C_NULL};

class HandCtrl{
public:
    HandCtrl(void);
    ~HandCtrl(void);

    bool update(int demo);

private:
    ros::NodeHandle nh;
    int fdcom;
};


#endif //___ZGT_CTRL_H__
