/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "interface/Gamepad.h"
#include <iostream>
Gamepad::Gamepad() {
    // 初始化手柄输入值
    // userValue.setZero();
    
    // // 订阅 /joy 话题，手柄输入
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Gamepad::joyCallback, this);
    
    std::cout << "Gamepad initialized and subscribed to /joy topic" << std::endl;
    pthread_create(&_tid, NULL, runGamepad, (void*)this);
}

Gamepad::~Gamepad() {
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    std::cout << "Gamepad object is being deleted" << std::endl;
}

void Gamepad::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    currentJoyData = *joy;
}

void* Gamepad::runGamepad(void *arg){
    ((Gamepad*)arg)->run(NULL);
    return NULL;
}

void* Gamepad::run(void *arg){
    // 无限循环，处理订阅的手柄数据
    while (ros::ok()) {
        ros::spinOnce();
        // std::cout << "Axes values: ";
        for (size_t i = 0; i < currentJoyData.axes.size(); i++) {
            // std::cout << currentJoyData.axes[i] << " ";
            userValue.ly = max<float>(min<float>(currentJoyData.axes[1], 1.0),-1.0);
            userValue.lx = max<float>(min<float>(currentJoyData.axes[0], 1.0),-1.0);
            userValue.rx = max<float>(min<float>(currentJoyData.axes[3], 1.0),-1.0);
        }
        // std::cout << std::endl;
        // std::cout << "Buttons values: ";
        for (size_t i = 0; i < currentJoyData.buttons.size(); i++) {
            // std::cout << currentJoyData.buttons[i] << " ";
            if (currentJoyData.buttons[i] == 1){
                userCmd = commandValues[i];
                // std::cout << "Command value at index " << i << " is " << static_cast<int>(commandValues[i]) <<
                    // std::endl;
            }
        }
        // std::cout << std::endl;
        // 等待1毫秒
        usleep(1000);
    }
    return NULL;
}