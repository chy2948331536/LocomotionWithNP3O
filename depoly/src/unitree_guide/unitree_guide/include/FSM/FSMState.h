/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "interface/CmdPanel.h"
#include "free_dog_sdk_cpp/lowCmd.hpp"
#include "free_dog_sdk_cpp/lowState.hpp"

class FSMState{
public:
    FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString);

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() {return FSMStateName::INVALID;}

    FSMStateName _stateName;
    std::string _stateNameString;
protected:
    CtrlComponents *_ctrlComp;
    FSMStateName _nextStateName;

#if defined(COMPILE_WITH_SIMULATION) || defined(COMPILE_WITH_REAL_ROBOT)
    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
#elif defined(COMPILE_WITH_REAL_ROBOT_FREE_DOG)
    FDSC::lowCmd *_lowCmd;
    FDSC::lowState *_lowState;
#endif
    UserValue _userValue;
};

#endif  // FSMSTATE_H