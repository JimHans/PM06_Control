<p align="center">
<h1 align="center"> PAIMON 06 Upper Software </h1>
<h3 align="center">派蒙06（PM-06）迷宫寻宝智能车的上位机控制软件仓库</h3>
<br/>
<p align="center">
<img src="https://img.shields.io/badge/build-passing-green.svg?style=flat-square">
<img src="https://img.shields.io/badge/Award-CNUSOC 1st,2023-yellow.svg?style=flat-square">
<img src="https://img.shields.io/badge/Contributors-3-blue.svg?style=flat-square">
<img src="https://img.shields.io/badge/License-GPL v3.0-purple.svg?style=flat-square">
</p>

语言/Language:
[中文](https://github.com/JimHans/PM06_Control/blob/master/README.md) | [English](https://github.com/JimHans/PM06_Control/blob/master/README_EN.md)

---

### 本仓库为2023年全国大学生光电设计竞赛全国一等奖队伍`Electron`设计的`PM-06`迷宫寻宝智能车的上位机控制软件仓库。全套软件支持拍照藏宝图识别、路径规划、一键启动与动态路径更新与避障功能。同时，通过Cython静态化预编译，本软件大大提升了动态路径规划的效率，使得`PM-06`能够在更短的时间内完成迷宫寻宝任务。

## 🕹部署项目
---
运行环境：Python 3.9及以上

运行方法
- 使用 `git clone https://github.com/JimHans/PM06_Control.git` 克隆本仓库到开发板桌面，请注意开发板需要至少支持aarch64指令集
- 使用`pip3 install -r requirements.txt`安装本项目依赖
- 使用Cython对`Astar.py` `MapScan.py` `Identify.py`进行静态化预编译
- 使用`python3 mainV2.py`运行本项目V2版本，或者通过`./start_car.sh`脚本一键启动

- ---

## 💻软件截图：

To be filled

## 🤔存在问题？
如果在复现或使用过程中遇到问题，请在Issues中提出，我们会在收到反馈后尽量解决。

## 🧡感谢：

本项目离不开以下项目支持：

[mpv / ©mpv-player / GPL-2.0, LGPL-2.1 licenses][1]  


This Program is open sourced under the GPL v3.0 license.

本程序基于 GPL v3.0 License 开源，不得用于商业用途

[1]: https://github.com/mpv-player/mpv