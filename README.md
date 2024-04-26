# Autonomous Driving Visualization Tool
- [English](#english)
- [中文](#中文)

## English
### Discription
This tool, specifically designed and implemented in autonomous driving domain, enables you to replay logs in the (.csv) format  
If equipped with speed planning source code, it could be utilized not only for data loopback but also for local simulation  
The following display information is currently supported:
1. Obstacles (position, speed, heading, prediction trajactory)
2. Lane lines (left, right, next left, next right)
3. Speed plan results (S-T, V-T, A-T graphs)
4. Ego motion status (set/actual speed, ACC mode)
5. TSR info (speed limit sign, special sign, ego display/alert status)
### Installation 
* Platform: Windows (Linux not supported due to incompatiblity of EasyX)
* Language: C++
* MinGW: To run executable file (.exe), an environment installed MinGW is strictly required. [MinGW official download site](https://github.com/niXman/mingw-builds-binaries/releases/tag/13.2.0-rt_v11-rev1)
* EasyX: For compiling the program with source code, the installlation of EasyX is essential. [EasyX official download site](https://codebus.cn/bestans/easyx-for-mingw)
### Usage
- Log format recognizable: (**.csv**)  
- To download complied executable file, see 
` /build/spdplan.exe  `
- For your convenience, two example logs are provided in the same directory for self-testing purposes
### License 
GNU General Public License v3.0  
All rights reserved by HGZ

## 中文
### 概要
该工具适用于自动驾驶行业，功能是可视化(.csv)格式的日志  
若配合速度规划源码，可额外激活数据回灌和本地仿真功能  
目前已支持以下信息显示：
1. 障碍物 （位置、速度、航向角、预测轨迹）
2. 车道线 （左、右、左左、右右）
3. 速度规划结果 （S-T, V-T, A-T 图）
4. 自车运动状态 （设置/当前 速度, 自适应巡航模式）
5. 交通标志识别信息 （限速标识、特殊标识、自车显示、报警状态）
### 安装
* 平台: Windows (Linux不兼容EasyX)
* 语言: C++
* MinGW: 运行可执行文件 (.exe)，必须先安装MinGW编译器. [MinGW官方下载](https://github.com/niXman/mingw-builds-binaries/releases/tag/13.2.0-rt_v11-rev1)
* EasyX: 编译源码，还需额外安装EasyX库. [EasyX官方下载](https://codebus.cn/bestans/easyx-for-mingw)
### 使用方式
- 日志格式: （**.csv**）  
- 可执行文件路径 
` /build/spdplan.exe  `
- 同路径下还提供了两个样例，供自测
