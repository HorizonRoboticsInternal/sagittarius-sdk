/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2021, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>
 */
#include "sdk_sagittarius_arm/sdk_sagittarius_arm_common_serial.h"
#include "sdk_sagittarius_arm/sdk_sagittarius_arm_log.h"
#include <algorithm>
#include <boost/asio.hpp>
#include <iterator>

#include <chrono>
#include <iostream>

namespace sdk_sagittarius_arm
{
    CSDarmCommonSerial::CSDarmCommonSerial(const std::string& serialname,
                                           int& baudrate, int& timelimit)
        : CSDarmCommon(), mSerialName(serialname), mBaudrate(baudrate),
          mTimeLimit(timelimit)
    {
    }

    CSDarmCommonSerial::~CSDarmCommonSerial()
    {
        SendArmLockOrFree(0);
        CloseDevice();
    }

    int CSDarmCommonSerial::InitDevice()
    {
        int i;
        int speed_arr[] = {B1500000, B1000000, B460800, B230400,
                           B115200,  B19200,   B9600,   B4800,
                           B2400,    B1200,    B300};
        int name_arr[] = {1500000, 1000000, 460800, 230400, 115200, 19200,
                          9600,    4800,    2400,   1200,   300};
        mFd = -1;
        mFd = open(mSerialName.c_str(), O_RDWR | O_NOCTTY);
        if (mFd == -1)
        {
            log_print(LOG_TYPE_ERROR, "打开串口失败 :%s fail\n",
                      mSerialName.c_str());
            return -1;
        }
        struct termios options;
        bzero(&options, sizeof(options));
        cfmakeraw(&options); //ÉèÖÃÎªRawÄ£Ê½
        //设置串口输入波特率和输出波特率
        for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
        {
            if (mBaudrate == name_arr[i])
            {
                cfsetispeed(&options, speed_arr[i]);
                cfsetospeed(&options, speed_arr[i]);
                log_print(LOG_TYPE_DEBUG, "使用波特率：%d\n", mBaudrate);
                break;
            }
        }
        if (i == sizeof(speed_arr) / sizeof(int))
        {
            log_print(LOG_TYPE_ERROR, "不支持此波特率：%d\n", mBaudrate);
            close(mFd);
            return -1;
        }

        options.c_cflag |= CLOCAL;    // 混乱模式，保证程序不会成为端口的占有者。
        //  options.c_cflag &= ~CLOCAL;// 混乱模式，保证程序不会成为端口的占有者。
        options.c_cflag |= CREAD;     // 控制模式，使端口能够读取输入的数据。
        options.c_cflag &= ~CRTSCTS;  // 无流控制 No Flow Control
        options.c_cflag &= ~CSIZE;    // 控制模式，设置字符大小位。
        options.c_cflag |= CS8;       // 8位
        options.c_cflag &= ~PARENB;   // 无校验
        options.c_cflag &= ~CSTOPB; // 1Í£Ö¹Î»
        options.c_oflag &= ~OPOST; //Êä³öÄ£Ê½£¬Ô­Ê¼Êý¾ÝÊä³ö
        options.c_cc[VMIN] = 0; //¿ØÖÆ×Ö·û, ËùÒª¶ÁÈ¡×Ö·ûµÄ×îÐ¡ÊýÁ¿
        options.c_cc[VTIME] =
            0; //¿ØÖÆ×Ö·û, ¶ÁÈ¡µÚÒ»¸ö×Ö·ûµÄµÈ´ýÊ±¼ä£¬µ¥Î»Îª0.1s

        tcflush(mFd, TCIFLUSH); //Òç³öµÄÊý¾Ý¿ÉÒÔ½ÓÊÕ£¬µ«²»¶Á

        //ÉèÖÃÐÂÊôÐÔ£¬TCSANOW£ºËùÓÐ¸Ä±äÁ¢¼´ÉúÐ§
        if (tcsetattr(mFd, TCSANOW, &options) != 0)
        {
            log_print(LOG_TYPE_ERROR, "set device error\n");
            return -1;
        }
        log_print(LOG_TYPE_INFO, "打开串口 %s 成功!\n", mSerialName.c_str());
        return 0;
    }

    int CSDarmCommonSerial::CloseDevice()
    {
        if (mFd != -1)
        {
            close(mFd);
            mFd = -1;
            log_print(LOG_TYPE_DEBUG, "close serial and Close Device\n");
        }
        return 0;
        // log_print(LOG_TYPE_DEBUG, "close serial and Close Device");
    }

    int CSDarmCommonSerial::SendSerialData2Arm(char* buf, int length)
    {
        int n = -1;
        if (mFd > 0)
        {
            n = write(mFd, buf, length);
        }
        return n;
    }

    unsigned char CSDarmCommonSerial::CheckSum(unsigned char* buf)
    {
        int i;
        unsigned char sum = 0;
        for (i = 0; i < buf[2]; i++)
        {
            sum += buf[3 + i];
        }
        // log_print(LOG_TYPE_DEBUG, "sum is %02x\n",sum);
        return sum;
    }
    void hex_printf(unsigned char* buf)
    {
        int i;
        for (i = 0; i < buf[2] + 5; i++)
        {
            printf("%02x ", buf[i]);
        }
        printf("\n");
    }

    //发送末端命令
    int CSDarmCommonSerial::SendArmEndAction(unsigned char onoff, short value)
    {
        unsigned char buf[30];
        short degree;
        if (mFd == -1)
        {
            log_print(LOG_TYPE_ERROR, "SendArmLockOrFree: Serial NOT open\n");
            return -1;
        }
        buf[0] = 0x55;                   //帧头1 55
        buf[1] = 0xAA;                   //帧头2 AA
        buf[2] = 5;                      //字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;   //请求信息
        buf[4] = CMD_CONTROL_END_ACTION; //发送末端命令
        buf[5] = onoff;                  // 0:关； 1：开
        buf[6] = value & 0xFF;
        buf[7] = value >> 8;
        buf[8] = CheckSum(buf); //校验和
        buf[9] = 0x7D;          //结束位
        // hex_printf(buf);
        if (SendSerialData2Arm((char*)buf, 10) != 10)
        {
            log_print(LOG_TYPE_WARN, "Write error for req command");
            return -1;
        }
        return 0;
    }
    //发送机械臂舵机释放或者锁住命令
    int CSDarmCommonSerial::SendArmLockOrFree(unsigned char onoff)
    {
        unsigned char buf[30];
        short degree;
        if (mFd == -1)
        {
            log_print(LOG_TYPE_ERROR, "SendArmLockOrFree: Serial NOT open\n");
            return -1;
        }
        buf[0] = 0x55;                     //帧头1 55
        buf[1] = 0xAA;                     //帧头2 AA
        buf[2] = 3;                        //字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;     //请求信息
        buf[4] = CMD_CONTROL_LOCK_OR_FREE; //舵机释放or锁住
        buf[5] = onoff;                    // 0:释放； 1：锁住
        buf[6] = CheckSum(buf);            //校验和
        buf[7] = 0x7D;                     //结束位

        if (SendSerialData2Arm((char*)buf, 8) != 8)
        {
            log_print(LOG_TYPE_WARN, "Write error for req command\n");
            return -1;
        }
        return 0;
    }
    //设置舵机的插补速度
    int CSDarmCommonSerial::SetArmVel(unsigned short vel)
    {
        unsigned char buf[30];
        short degree;
        if (mFd == -1)
        {
            log_print(LOG_TYPE_ERROR, "SetArmVel: Serial NOT open\n");
            return -1;
        }
        buf[0] = 0x55;                 //帧头1 55
        buf[1] = 0xAA;                 //帧头2 AA
        buf[2] = 4;                    //字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE; //请求信息
        buf[4] = CMD_SET_SERVO_VEL;    //设置舵机的插补速度
        buf[5] = vel;                  //速度低位
        buf[6] = vel >> 8;             //速度高位
        buf[7] = CheckSum(buf);        //校验和
        buf[8] = 0x7D;                 //结束位

        if (SendSerialData2Arm((char*)buf, 9) != 9)
        {
            log_print(LOG_TYPE_WARN, "SendSerialData2Arm: error\n");
            return -1;
        }
        log_print(LOG_TYPE_INFO, "SetArmVel: %d\n", vel);
        return 0;
    }
    //设置舵机的加速度
    int CSDarmCommonSerial::SetArmAcc(unsigned char acc)
    {
        unsigned char buf[30];
        if (mFd == -1)
        {
            log_print(LOG_TYPE_ERROR, "SetArmAcc: Serial NOT open\n");
            return -1;
        }
        buf[0] = 0x55;                 //帧头1 55
        buf[1] = 0xAA;                 //帧头2 AA
        buf[2] = 3;                    //字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE; //请求信息
        buf[4] = CMD_SET_SERVO_ACC;    //设置舵机的加速度
        buf[5] = acc;                  //加速度
        buf[6] = CheckSum(buf);        //校验和
        buf[7] = 0x7D;                 //结束位

        if (SendSerialData2Arm((char*)buf, 8) != 8)
        {
            log_print(LOG_TYPE_WARN, "SendSerialData2Arm: error\n");
            return -1;
        }
        log_print(LOG_TYPE_INFO, "SetArmAcc: %d\n", acc);
        return 0;
    }
    //设置舵机的扭矩大小
    int CSDarmCommonSerial::SetArmTorque(int torque[])
    {
        unsigned char buf[30];
        int i;
        if (mFd == -1)
        {
            log_print(LOG_TYPE_ERROR, "SetArmTorque: Serial NOT open\n");
            return -1;
        }
        buf[0] = 0x55;                 //帧头1 55
        buf[1] = 0xAA;                 //帧头2 AA
        buf[2] = 2 * 7 + 2;            //字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE; //请求信息
        buf[4] = CMD_SET_SERVO_TORQUE; //设置舵机的扭矩大小
        for (i = 0; i < 7; i++)
        {
            buf[5 + 2 * i] = torque[i];
            buf[6 + 2 * i] = torque[i] >> 8;
            log_print(LOG_TYPE_INFO, "SetArmTorque(%d): %d\n", i, torque[i]);
        }
        buf[5 + 2 * i] = CheckSum(buf); //校验和
        buf[6 + 2 * i] = 0x7D;          //结束位

        if (SendSerialData2Arm((char*)buf, 7 + 2 * i) != 7 + 2 * i)
        {
            log_print(LOG_TYPE_WARN, "SendSerialData2Arm: error\n");
            return -1;
        }
        return 0;
    }
    int CSDarmCommonSerial::SendArmAllServerTime(float v1, float v2, float v3,
                                                 float v4, float v5, float v6)
    {
        unsigned char buf[30];
        static unsigned char lastbuf[30];
        short degree;
        if (mFd == -1)
        {
            log_print(LOG_TYPE_ERROR,
                      "SendArmAllServerTime: Serial NOT open\n");
            return -1;
        }
        buf[0] = 0x55;                                 //帧头1 55
        buf[1] = 0xAA;                                 //帧头2 AA
        buf[2] = 16;                                   //字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;                 //请求信息
        buf[4] = CMD_CONTROL_ALL_DEGREE_AND_DIFF_TIME; //多个舵机控制和时间差

        buf[5] = 0; //角度低位
        buf[6] = 0 >> 8;
        degree = 1800 / PI * v1; // v1转成角度x10
        buf[7] = degree;         //角度低位
        buf[8] = degree >> 8;    //角度高位
        degree = 1800 / PI * v2; // v2转成角度x10
        buf[9] = degree;         //角度低位
        buf[10] = degree >> 8;   //角度高位
        degree = 1800 / PI * v3; // v3转成角度x10  德晟此处为负
        buf[11] = degree;        //角度低位
        buf[12] = degree >> 8;   //角度高位
        degree = 1800 / PI * v4; // v4转成角度x10
        buf[13] = degree;        //角度低位
        buf[14] = degree >> 8;   //角度高位
        degree = 1800 / PI * v5; // v5转成角度x10
        buf[15] = degree;        //角度低位
        buf[16] = degree >> 8;   //角度高位
        degree = 1800 / PI * v6; // v6转成角度x10
        buf[17] = degree;        //角度低位
        buf[18] = degree >> 8;   //角度高位
        buf[19] = CheckSum(buf); //校验和
        buf[20] = 0x7D;          //结束位
        //    ROS_INFO("[%d,%d,%d,%d,%d,%d]",
        //    short(buf[5]|(buf[6]<<8)),short(buf[7]|(buf[8]<<8)),short(buf[9]|(buf[10]<<8)),short(buf[11]|(buf[12]<<8)),short(buf[13]|(buf[14]<<8)),short(buf[15]|(buf[16]<<8)));
        log_print(LOG_TYPE_INFO, "time:%dms [%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]\n",
                  0, (float(short(buf[7] | (buf[8] << 8)))) / 10,
                  (float(short(buf[9] | (buf[10] << 8)))) / 10,
                  (float(short(buf[11] | (buf[12] << 8)))) / 10,
                  (float(short(buf[13] | (buf[14] << 8)))) / 10,
                  (float(short(buf[15] | (buf[16] << 8)))) / 10,
                  (float(short(buf[17] | (buf[18] << 8)))) / 10);
        if (memcmp(buf, lastbuf, 19) != 0)
        {
            memcpy(lastbuf, buf, 21);
            if (SendSerialData2Arm((char*)buf, 21) != 21)
            {
                log_print(LOG_TYPE_WARN, "Write error for req command\n");
                return -1;
            }
        }
        return 0;
    }

    //插补控制命令
    int CSDarmCommonSerial::SendArmAllServerWithIndex(ServoStruct sv[], int num)
    {
        unsigned char buf[30];
        static unsigned char lastbuf[30];
        short degree;
        int i;
        if (mFd == -1)
        {
            log_print(LOG_TYPE_ERROR,
                      "SendArmAllServerWithIndex: Serial NOT open\n");
            return -1;
        }
        buf[0] = 0x55;                  //帧头1 55
        buf[1] = 0xAA;                  //帧头2 AA
        buf[2] = 2 + 3 * num;           //字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;  //请求信息
        buf[4] = CMD_CONTROL_ID_DEGREE; //指定舵机控制
        //    v1 = -v1;
        for (i = 0; i < num; i++)
        {
            degree = 1800 / PI * sv[i].value; // v1转成角度x10

            buf[5 + 3 * i] = sv[i].id;
            buf[6 + 3 * i] = degree;
            buf[7 + 3 * i] = degree >> 8;
        }
        buf[5 + 3 * num] = CheckSum(buf); //校验和
        buf[6 + 3 * num] = 0x7D;          //结束位
        //    ROS_INFO("[%d,%d,%d,%d,%d,%d]",
        //    short(buf[5]|(buf[6]<<8)),short(buf[7]|(buf[8]<<8)),short(buf[9]|(buf[10]<<8)),short(buf[11]|(buf[12]<<8)),short(buf[13]|(buf[14]<<8)),short(buf[15]|(buf[16]<<8)));
        // log_print(LOG_TYPE_INFO, "[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
        // (float(short(buf[5]|(buf[6]<<8))))/10,(float(short(buf[7]|(buf[8]<<8))))/10,(float(short(buf[9]|(buf[10]<<8))))/10,(float(short(buf[11]|(buf[12]<<8))))/10,(float(short(buf[13]|(buf[14]<<8))))/10,(float(short(buf[15]|(buf[16]<<8))))/10);
        if (SendSerialData2Arm((char*)buf, 7 + 3 * num) != (7 + 3 * num))
        {
            log_print(LOG_TYPE_WARN, "Write error for req command\n");
            return -1;
        }
        return 0;
    }

    //插补控制命令
    int CSDarmCommonSerial::SendArmAllServerCB(float v1, float v2, float v3,
                                               float v4, float v5, float v6)
    {
        unsigned char buf[30];
        static unsigned char lastbuf[30];
        short degree;
        if (mFd == -1)
        {
            log_print(LOG_TYPE_ERROR, "SendArmAllServerCB: Serial NOT open\n");
            return -1;
        }
        buf[0] = 0x55;                      //帧头1 55
        buf[1] = 0xAA;                      //帧头2 AA
        buf[2] = 14;                        //字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;      //请求信息
        buf[4] = CMD_CONTROL_ALL_DEGREE_CB; //插补控制命令
        degree = 1800 / PI * v1;            // v1转成角度x10
        buf[5] = degree;                    //角度低位
        buf[6] = degree >> 8;               //角度高位
        degree = 1800 / PI * v2;            // v2转成角度x10
        buf[7] = degree;                    //角度低位
        buf[8] = degree >> 8;               //角度高位
        degree = 1800 / PI * v3; // v3转成角度x10  德晟此处为负
        buf[9] = degree;         //角度低位
        buf[10] = degree >> 8;   //角度高位
        degree = 1800 / PI * v4; // v4转成角度x10
        buf[11] = degree;        //角度低位
        buf[12] = degree >> 8;   //角度高位
        degree = 1800 / PI * v5; // v5转成角度x10
        buf[13] = degree;        //角度低位
        buf[14] = degree >> 8;   //角度高位
        degree = 1800 / PI * v6; // v6转成角度x10
        buf[15] = degree;        //角度低位
        buf[16] = degree >> 8;   //角度高位
        buf[17] = CheckSum(buf); //校验和
        buf[18] = 0x7D;          //结束位
        // log_print(LOG_TYPE_INFO, "[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
        // (float(short(buf[5]|(buf[6]<<8))))/10,(float(short(buf[7]|(buf[8]<<8))))/10,(float(short(buf[9]|(buf[10]<<8))))/10,(float(short(buf[11]|(buf[12]<<8))))/10,(float(short(buf[13]|(buf[14]<<8))))/10,(float(short(buf[15]|(buf[16]<<8))))/10);
        if (memcmp(buf, lastbuf, 19) != 0)
        {
            memcpy(lastbuf, buf, 19);
            if (SendSerialData2Arm((char*)buf, 19) != 19)
            {
                log_print(LOG_TYPE_WARN, "Write error for req command\n");
                return -1;
            }
        }
        return 0;
    }

    int CSDarmCommonSerial::SendArmAllServer(float v1, float v2, float v3,
                                             float v4, float v5, float v6)
    {
        unsigned char buf[30];
        static unsigned char lastbuf[30];
        short degree;
        if (mFd == -1)
        {
            log_print(LOG_TYPE_ERROR, "SendArmAllServer: Serial NOT open\n");
            return -1;
        }
        buf[0] = 0x55;                   //帧头1 55
        buf[1] = 0xAA;                   //帧头2 AA
        buf[2] = 14;                     //字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;   //请求信息
        buf[4] = CMD_CONTROL_ALL_DEGREE; //多个舵机控制
        degree = 1800 / PI * v1;         // v1转成角度x10
        buf[5] = degree;                 //角度低位
        buf[6] = degree >> 8;            //角度高位
        degree = 1800 / PI * v2;         // v2转成角度x10
        buf[7] = degree;                 //角度低位
        buf[8] = degree >> 8;            //角度高位
        degree = -1800 / PI * v3; // v3转成角度x10  德晟此处为负
        buf[9] = degree;          //角度低位
        buf[10] = degree >> 8;    //角度高位
        degree = 1800 / PI * v4;  // v4转成角度x10
        buf[11] = degree;         //角度低位
        buf[12] = degree >> 8;    //角度高位
        degree = 1800 / PI * v5;  // v5转成角度x10
        buf[13] = degree;         //角度低位
        buf[14] = degree >> 8;    //角度高位
        degree = 1800 / PI * v6;  // v6转成角度x10
        buf[15] = degree;         //角度低位
        buf[16] = degree >> 8;    //角度高位
        buf[17] = CheckSum(buf);  //校验和
        buf[18] = 0x7D;           //结束位
        // printf("[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
        // (float(short(buf[5]|(buf[6]<<8))))/10,(float(short(buf[7]|(buf[8]<<8))))/10,(float(short(buf[9]|(buf[10]<<8))))/10,(float(short(buf[11]|(buf[12]<<8))))/10,(float(short(buf[13]|(buf[14]<<8))))/10,(float(short(buf[15]|(buf[16]<<8))))/10);
        if (memcmp(buf, lastbuf, 19) != 0)
        {
            memcpy(lastbuf, buf, 19);
            if (SendSerialData2Arm((char*)buf, 19) != 19)
            {
                log_print(LOG_TYPE_WARN, "Write error for req command\n");
                return -1;
            }
        }
        return 0;
    }

    int CSDarmCommonSerial::GetDataGram(unsigned char* receiveBuffer,
                                        int bufferSize, int* length)
    {
        static int first_bit = 1;
        int len, fs_sel;
        fd_set fs_read;
        int i;
        struct timeval time;
        if (mFd == -1)
        {
            if (first_bit)
            {
                first_bit = 0;
                log_print(
                    LOG_TYPE_ERROR,
                    "GetDataGram: Serial not open\n"); //只打印一次，使用红色打印。
            }
            return -1;
        }

        if (1) //(!stream_stopped_)
        {
            FD_ZERO(&fs_read);
            FD_SET(mFd, &fs_read);
            time.tv_sec = 1; // mTimeLimit;
            time.tv_usec = 0;
            //使用select实现串口的多路通信
            // auto start = std::chrono::high_resolution_clock::now();
            fs_sel = select(mFd + 1, &fs_read, NULL, NULL, &time);
            // auto end = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> diff = end - start;
            // std::cout << "GetDataGram: select() time: " << diff.count() << " seconds" << std::endl;
            if (fs_sel > 0)
            {
                if (FD_ISSET(mFd, &fs_read))
                {
                    *length = read(mFd, receiveBuffer, bufferSize);
                    /*for(i=0;i<*length;i++)
                    {
                        printf("%02x ",receiveBuffer[i]);
                    }
                    printf("\n");*/
                }
            }
            else
            {
                log_print(LOG_TYPE_WARN,
                          "GetDataGram: serial-receive timeout for %ds\n",
                          mTimeLimit); // warnning!
                return -1;
            }
        }

        return 0;
    }
} // namespace sdk_sagittarius_arm
