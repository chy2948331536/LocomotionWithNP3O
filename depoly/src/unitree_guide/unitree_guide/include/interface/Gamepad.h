/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <termios.h>
#include <sys/time.h>
#include "interface/CmdPanel.h"
#include "common/mathTools.h"
class Gamepad : public CmdPanel{
public:
    Gamepad();  // 构造函数
    ~Gamepad(); // 析构函数

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);  // 手柄回调函数
    static void* runGamepad(void *arg);  // 运行手柄线程
    void* run(void *arg);  // 运行手柄线程
    pthread_t _tid;
private:
    ros::NodeHandle nh;  // ROS节点句柄
    ros::Subscriber joy_sub;  // 订阅 /joy 话题
    sensor_msgs::Joy currentJoyData;
    std::array<UserCommand, 9> commandValues = {
        UserCommand::L1_Y,
        UserCommand::L1_A,
        UserCommand::L1_X,
        UserCommand::NONE,
        UserCommand::L2_X,
#ifdef COMPILE_WITH_MOVE_BASE
        UserCommand::L2_Y,
#endif
        UserCommand::L2_A,
        UserCommand::NONE,
        UserCommand::NONE,
    };
};

#endif  // GAMEPAD_H
