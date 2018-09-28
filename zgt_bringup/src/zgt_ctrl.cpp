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

#include <zgt_bringup/zgt_ctrl.h>

HandCtrl::HandCtrl(void)
{
    printf("HandCtrl init.\r\n");
    fdcom = SerialOpen("/dev/ttyUSB0");
    if(fdcom > 0)
    {
        PortInfo_t portinfo = {BAUDRATE, 8, 2, 0, 1, 0};
        SerialSet(fdcom, &portinfo);
        printf("[COM PARAM]: 115200, 8, 2, 0, 1, 0\n");
    }
    else
    {
        printf("[ERROR]: open serial port error.\n");
    }
}

HandCtrl::~HandCtrl(void)
{
    SerialClose(fdcom);
    printf("HandCtrl deinit.\n");
}

bool HandCtrl::update(int demo)
{
    tx_buffer[9] = cmd_tbl[demo][0];
    tx_buffer[10] = cmd_tbl[demo][1];
    tx_buffer[11] = cmd_tbl[demo][2];
    SerialSend(fdcom, tx_buffer, TX_MAX);
    switch(demo)
    {
        case 0:
            printf("[手势]： 五指张开.\r\n");
            break;
        case 1:
            printf("[手势]： 五指合并.\r\n");
            break;
        case 2:
            printf("[手势]： 抓取卡片.\r\n");
            break;
        case 3:
            printf("[手势]： 顺序闭合.\r\n");
            break;
        case 4:
            printf("[手势]： 顺序张开.\r\n");
            break;
        case 5:
            printf("[手势]： 剪刀手.\r\n");
            break;
        case 6:
            printf("[手势]： OK手势.\r\n");
            break;
        default:
            printf("[手势]： 未知手势.\r\n");
            break;
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "zgt_ctrl");
    ros::Time::init();
    ros::Rate loop_rate(10);
    HandCtrl hc;
    int zgt_demo = 0;
    if(argc == 2)
    {
        zgt_demo = atoi(argv[1]);
        if((zgt_demo < 0)||(zgt_demo >= CMD_MAX))
        {
            zgt_demo = 0;
        }
    }

    while (ros::ok()) {
        hc.update(zgt_demo);
        loop_rate.sleep();
        return 0;
    }
}