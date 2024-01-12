> **注意：** 下文件由人工智能生成，可能存在错误。在使用本文件中的信息时，请进行适当的验证。
# gao-s_mot_ros

这是一个使用ROS (Robot Operating System) 进行物体追踪和地图构建的项目。项目主要包括物体检测、追踪和地图构建等功能。

## 功能
物体检测：通过接收 /centerpoint/dets 话题的数据进行物体检测。
物体追踪：对检测到的物体进行追踪，并发布追踪结果到 /mot_tracking/box 话题。
地图构建：暂未实现。

## Topic输入/输出
### 输入：
/centerpoint/dets：物体检测结果，类型为 visualization_msgs::MarkerArray。
### 输出：
/mot_tracking/box：物体追踪结果，类型为 visualization_msgs::MarkerArray。

## 安装运行方法
1. 克隆项目到你的 catkin 工作空间的 src 目录下：
```bash
cd ~/catkin_ws/src
git clone https://github.com/rand0md00r/gao-s_mot_ros.git
```

2. 在 catkin 工作空间目录下编译项目：
```bash
cd ~/catkin_ws
catkin_make
```

3. 运行项目：
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch mot_tracking track.launch
```

## 致谢
本项目的部分代码参考了[Multi-Object-Tracking](https://github.com/wangx1996/Multi-Object-Tracking.git)项目。我们对原作者的贡献表示感谢。

## 许可证
此项目遵循 MIT 许可证。
