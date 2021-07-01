# 中文说明
必须注意：
**Webots版本为R2021a Ubuntu版本为20.04**

中文教程地址[ROS联合webots实战案例目录](https://blog.csdn.net/xiaokai1999/article/details/112601720)
## 各文件功能介绍

|launch文件|功能|
| ---- | ---- |
|amcl_demo.launch|使用amcl定位算法，base算法作为导航算法进行已知地图导航|
|slam_base_cartographer.launch|cartographer算法作为实时建图算法，base算法作为导航算法进行导航建图|
|slam_base_gmapping.launch|gmapping算法作为实时建图算法，base算法作为导航算法进行导航建图|
|slam_teb_cartographer.launch|cartographer算法作为实时建图算法，teb算法作为导航算法进行导航建图|
|slam_teb_gmapping.launch|gmapping算法作为实时建图算法，teb算法作为导航算法进行导航建图|
|slam_with_gmapping.launch|gmapping算法作为实时建图算法，通过键盘控制机器人行走建立地图|
|webots.launch|启动webots仿真软件|

|src文件|功能|
| ---- | ---- |
|demo_2dnav_move.cpp|用于导航的底层控制代码|
|robot_broadcaster_cartographer.cpp|机器人在cartographer建图算法下专用启动程序|
|robot_broadcaster_gmapping.cpp|机器人在gmapping建图算法下专用启动程序|
|robot_set_goals.cpp|机器人通过程序设定目标点|
|velocity_joy.cpp|通过手柄控制机器人移动|
|velocity_keyboard_v2.cpp|通过webots控制机器人移动|
## 操作方法
1. 安装必备功能包
- 安装gmapping功能包
``` shell
$ sudo apt install ros-noetic-gmapping
```
- 安装amcl定位功能包
``` shell
$ sudo apt-get install ros-noetic-amcl
```

- 安装cartographer功能包

参照这篇文章中的cartographer安装方法：[Ubuntu 18.04 安装cartographer](https://blog.csdn.net/xiaokai1999/article/details/112791787)

- 安装map-server功能包
``` shell
$ sudo apt install ros-noetic-map-server
```

- 安装move_base功能包
``` shell
$ sudo apt install ros-noetic-move-base
```

- 安装TEB导航功能包
``` shell
$ sudo apt install ros-noetic-teb-local-planner
```

2. 命令行进入`catkin_ws/src`
``` shell
$ cd catkin_ws/src
$ git clone https://github.com/JackyMao1999/webots_demo.git
```
3. 编译
``` shell
$ catkin_make
```
4. 运行
``` shell
$ roslaunch webots_demo amcl_demo.launch
$ rosrun  webots_demo demo_2dnav_move
```
## 效果
![1](2-2.gif)

结语
本文也是基于笔者的学习和使用经验总结的，主观性较强，如果有哪些不对的地方或者不明白的地方，欢迎评论区留言交流~
这个教程能手把手带领大家解决webots和ROS联合仿真的问题，我也是摸爬滚打完成的，希望能帮助到大家。

✌Bye
