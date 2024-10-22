/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include "free_dog_sdk_cpp/common.hpp"
#include "free_dog_sdk_cpp/complex.hpp"
#include "free_dog_sdk_cpp/lowCmd.hpp"
#include "free_dog_sdk_cpp/lowState.hpp"
#include "free_dog_sdk_cpp/unitreeConnectBoost.hpp"
#include <string>

class IOInterface{
public:
IOInterface(){}
~IOInterface(){delete cmdPanel;}

#if defined(COMPILE_WITH_SIMULATION) || defined(COMPILE_WITH_REAL_ROBOT)
 virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
#elif defined(COMPILE_WITH_REAL_ROBOT_FREE_DOG)
 virtual void sendRecv(const FDSC::lowCmd *cmd, FDSC::lowState *state) = 0;
#endif

void zeroCmdPanel(){cmdPanel->setZero();}
void setPassive(){cmdPanel->setPassive();}

protected:
CmdPanel *cmdPanel;
};

#endif  //IOINTERFACE_H