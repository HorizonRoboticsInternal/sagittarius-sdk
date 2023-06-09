# Sagittarius SDK (K1 Robotic Arm)

This is a curated fork of the original repo [sagittarius_sdk](https://github.com/NXROBO/sagittarius_sdk). The original repo is licensed under BSD and the notice is kept at the top of each of the original source files.

The modification including the `CMakeLists.txt` updates, a websocket service and a Python binding are all licensed under BSD following the original repo.

## Development Guide

### Build

```bash
$ nix develop
$ mkdir build
$ cd build
$ cmake ..
$ make -j10
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

