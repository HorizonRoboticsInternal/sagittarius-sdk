# Sagittarius SDK (K1 Robotic Arm)

# Updated Development Guide

Work in progress ...


# Original Development Guide

非ROS下的SDK

### 依赖库安装

**Eigen 库安装**

```bash
sudo apt-get install libeigen3-dev libboost-all-dev
```

如果在编译时找不到`Eigen/Dense`而报错，原因是Eigen3相比前一个版本多套了一层文件夹.

解决方式是软连接到上一层目录。先找到Eigen3的安装目录

```bash
whereis eigen3
```

假设Eigen3的安装目录是`/usr/include/eigen3`

```bash
cd /usr/include/
sudo ln -s eigen3/Eigen Eigen
```


### 设备说明

**串口**

使用的是USB串口，在linux下设置号为/dev/ttyACMx  当没有其它的ttyACM串口设备时，默认为/dev/ttyACM0
支持多种波特率通讯，建议使用1000000, 460800, 230400, 115200中的一种。

**舵机**


舵机0～6的弧度下限位 ： 
```bash
{-2, -1.57, -1.48, -2.9, -1.8, -3.1};
```
舵机0～6的弧度上限位 ： 
```bash
{2, 1.4, 1.8, 2.9, 1.6, 3.1};
```


### 例程编译

此程序为非ROS下的例程编译

```bash
cd sagittarius_sdk
g++ -std=c++11 -I ./ -o test  sagittarius_example.cpp sdk_sagittarius_arm_log.cpp sdk_sagittarius_arm_common.cpp sdk_sagittarius_arm_common_serial.cpp sdk_sagittarius_arm_real.cpp modern_robotics.cpp -lpthread -lboost_system -lboost_thread
```

编译成动态库

```bash
cd sagittarius_sdk
g++ -shared -fPIC -I ./ modern_robotics.cpp sdk_sagittarius_arm_log.cpp sdk_sagittarius_arm_common.cpp sdk_sagittarius_arm_common_serial.cpp sdk_sagittarius_arm_real.cpp -o libsagittarius_sdk.so
```
