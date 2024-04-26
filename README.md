# Autonomous Driving Visualization Tool
## Discription
This tool is developed and applied in autonomous driving field, which allows you to play a log with format(.csv)   
If equipped with speed planning source code, it could also be used for data loopback and local simulation  
The following display information is currently supported:
- Obstacles (position, speed, heading, prediction trajactory)
- Lane lines (left, right, next left, next right)
- Speed plan results (S-T, V-T, A-T graphs)
- Ego motion status (set/current speed, ACC mode)
- TSR info (speed limit sign, special sign, ego display/alert status)
## Installation 
* Platform: Windows (Linux not supported due to EasyX)
* Language: C++
* MinGW: To run executable file(.exe), a computer installed MinGW is strictly required. [MinGW official download site](https://github.com/niXman/mingw-builds-binaries/releases/tag/13.2.0-rt_v11-rev1)
* EasyX: To complie this program with source code, EasyX installlation is also essential. [EasyX official download site](https://codebus.cn/bestans/easyx-for-mingw)
## Usage
- Log format recognizable: (.csv)  
- To download complied executable file, see 
` /build/spdplan.exe  `
- Two log examples are concurrently offered under the same path, for users to self-testing
## License 
GNU General Public License v3.0  
All rights reserved by HGZ
