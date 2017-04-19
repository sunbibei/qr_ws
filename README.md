# DRAGON ROBOT

## OVERVIEW
`dragon_robot` package, 是电子科技大学MII研究所恐龙机器人软件包. 依赖于ROS环境, 以catkin方式管理, 编译. 文件系统如下所示:

<center>
![](dragon_robot/img/tree.png)
</center>

将该目录放到自己的catkin工作空间中的src文件夹下(例如`~/catkin_ws/src/'), 使用`catkin_make`编译即可.

总共包含4个功能包, 分别是`dragon_description`, `dragon_driver`, `dragon_gazebo`, 和 `dragon_moveit_config`.

## `dragon_description`

该功能包, 包含恐龙机器人描述文件, 启动launch文件等. 另外, 还包含单腿平台模型文件.

## `dragon_gazebo`

该功能包, 包含恐龙机器人Gazebo仿真launch文件, 以及某些控制器配置文件以及实验参数等.

## `dragon_moveit_config`

该功能包, 由`moveit_assistant`包自动生成, 可以使用MoveIt!配置恐龙机器人在仿真中进行轨迹规划.

## `dragon_driver`

该功能包, 依据实际机器人平台, 完成与实际机器人通讯, 启动机器人, 获取机器人数据, 并提供ROS Control 机制支持.