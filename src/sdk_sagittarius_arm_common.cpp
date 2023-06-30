/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2021, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>
 */
#include "sdk_sagittarius_arm/sdk_sagittarius_arm_common.h"

#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "sdk_sagittarius_arm/sdk_sagittarius_arm_log.h"
namespace sdk_sagittarius_arm {

CSDarmCommon::CSDarmCommon() : mThrcv(NULL) {
  /*Initialize receive buffer*/
  memset(mRecvBuffer, 0, RECV_BUFFER_SIZE);
  mDataLength = 0;
}

/**
 * 	Ïû»ÙÏß³Ì
 */
bool CSDarmCommon::DestroyThread(boost::thread** th) {
  if ((*th) != NULL) {
    (*th)->interrupt();
    (*th)->join();
    delete (*th);
    (*th) = NULL;
    return true;
  }
  return true;
}

int CSDarmCommon::StopArmLock() {
  int result = 0;

  return result;
}

bool CSDarmCommon::RebootDevice() {
  return true;
}

CSDarmCommon::~CSDarmCommon() {
  DestroyThread(&mThrcv);
  log_print(LOG_TYPE_DEBUG, "sdk_sagittarius_arm drvier exiting.\n");
}

int CSDarmCommon::Init() {
  int result = InitDevice();
  if (0 != result) {
    log_print(LOG_TYPE_ERROR, "Failed to init device: %d\n", result);
    return result;
  }

  result = InitArm();
  if (0 != result) {
    log_print(LOG_TYPE_ERROR, "Failed to init scanner: %d\n", result);
  }

  return result;
}

unsigned char CSDarmCommon::CheckSum(unsigned char* buf) {
  int i;
  unsigned char sum = 0;
  for (i = 0; i < buf[2]; i++) {
    sum += buf[3 + i];
  }
  // log_print(LOG_TYPE_DEBUG, "sum is %02x\n",sum);
  return sum;
}

int CSDarmCommon::InitArm() {
  SendArmLockOrFree(1);
  return 0;
}

void CSDarmCommon::print_hex(unsigned char* buf, int len) {
#if 0
        int i;
        for(i=0; i<len; i++)
        {
            printf("%02x ",buf[i]);
        }
        printf("\n");
#endif
}

void CSDarmCommon::StartReceiveSerail() {
  if (mThrcv == NULL)
    mThrcv = new boost::thread(boost::bind(&CSDarmCommon::LoopRcv, this));
}
void CSDarmCommon::LoopRcv() {
  while (1) {
    // If this cycle time is long, e.g. 100ms, then all joint readings will stay
    // the same during this whole time.
    // auto start = std::chrono::high_resolution_clock::now();
    LoopOnce();
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> diff = end - start;
    // std::cout << "LoopOnce: total time: " << diff.count() << " seconds" << std::endl;
  }
}

/// @brief  ROS Publishers JointState
void CSDarmCommon::PublishJointStates(unsigned char* buf) {
  // These assignments are not atomic, and can have race conditions, where
  // the joint_status is read while it is being written to.
  joint_status[0] = (short(buf[0] | buf[1] << 8)) / 1800.0 * PI;
  joint_status[1] = (short(buf[2] | buf[3] << 8)) / 1800.0 * PI;
  joint_status[2] = (short(buf[4] | buf[5] << 8)) / 1800.0 * PI;
  joint_status[3] = (short(buf[6] | buf[7] << 8)) / 1800.0 * PI;
  joint_status[4] = (short(buf[8] | buf[9] << 8)) / 1800.0 * PI;
  joint_status[5] = (short(buf[10] | buf[11] << 8)) / 1800.0 * PI;
  joint_status[6] = (short(buf[12] | buf[13] << 8)) / 1800.0 * PI;
}

int CSDarmCommon::LoopOnce() {
  int dataLength = 0;
  int result     = GetDataGram(mRecvBuffer, RECV_BUFFER_SIZE, &dataLength);
  if (result != 0) {
    // This error is never caught.
    log_print(LOG_TYPE_ERROR, "sdk_sagittarius_arm - Read Error when getting datagram: %d", result);
    return -1;
  } else {
    if (dataLength > 0) {
      if ((dataLength < 255) && (mDataLength < 255)) {
        print_hex(mRecvBuffer, dataLength);
        memcpy(mFrameBuffer + mDataLength, mRecvBuffer, dataLength);
        mDataLength = mDataLength + dataLength;
        if (mFrameBuffer[0] != 0x55) mDataLength = 0;
      } else
        mDataLength = 0;
      if ((mDataLength > 3) && (mDataLength >= (mFrameBuffer[2] + 5)))  //检测包的大小
      {
        if ((mFrameBuffer[0] == 0x55) && (mFrameBuffer[1] == 0xAA) &&
            (mFrameBuffer[mFrameBuffer[2] + 4] == 0x7D) &&
            (CheckSum(mFrameBuffer) ==
             (unsigned char)mFrameBuffer[mFrameBuffer[2] + 3]))  //校验和与头尾比较
        {
          // log_print(LOG_TYPE_DEBUG, "receive data:");
          print_hex(mFrameBuffer, mDataLength);
          if (mFrameBuffer[4] == 0x0A)  //升级相关的命令
          {
            log_print(LOG_TYPE_INFO, "升级的命令\n");
          } else if (mFrameBuffer[4] == 0x09)  // version的命令
          {
            if (mFrameBuffer[3] == 0x02)
              log_print(LOG_TYPE_INFO, "version is %s\n", mFrameBuffer + 5);
          } else if (mFrameBuffer[4] == 0x06)  // version的命令
          {
            if (mFrameBuffer[3] == 0x01) {
              PublishJointStates(mFrameBuffer + 5);
              // std::cout << "Published joint_status: " << (float(short(mFrameBuffer[7]|(mFrameBuffer[8]<<8))))/10 << std::endl;
              // ROS_INFO("ARM->ROS:length=%d
              // [%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
              // mDataLength,
              // (float(short(mFrameBuffer[5]|(mFrameBuffer[6]<<8))))/10,(float(short(mFrameBuffer[7]|(mFrameBuffer[8]<<8))))/10,(float(short(mFrameBuffer[9]|(mFrameBuffer[10]<<8))))/10,(float(short(mFrameBuffer[11]|(mFrameBuffer[12]<<8))))/10,(float(short(mFrameBuffer[13]|(mFrameBuffer[14]<<8))))/10,(float(short(mFrameBuffer[15]|(mFrameBuffer[16]<<8))))/10,(float(short(mFrameBuffer[17]|(mFrameBuffer[18]<<8))))/10);

              // log_print(LOG_TYPE_INFO, "ARM->ROS:
              // %f,%f,%f,%f,%f,%f\n",float(mFrameBuffer[5]|mFrameBuffer[6]<<8)/10,float(mFrameBuffer[7]|mFrameBuffer[8]<<8)/10,float(mFrameBuffer[9]|mFrameBuffer[10]<<8)/10,float(mFrameBuffer[11]|mFrameBuffer[12]<<8)/10,float(mFrameBuffer[13]|mFrameBuffer[14]<<8)/10,float(mFrameBuffer[15]|mFrameBuffer[16]<<8)/10);
            }
          } else {
            log_print(LOG_TYPE_INFO, "其它的命令\n");
          }
        } else {
          log_print(LOG_TYPE_WARN, "帧数据出错!\n");
          print_hex(mFrameBuffer, mDataLength);
        }
        mDataLength = 0;
      }
    }
  }
  return 0;  // return success to continue
}
}  // namespace sdk_sagittarius_arm
