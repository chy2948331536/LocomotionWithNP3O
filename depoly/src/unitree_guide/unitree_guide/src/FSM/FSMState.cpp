/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSMState.h"

#if defined(COMPILE_WITH_REAL_ROBOT) || defined(COMPILE_WITH_SIMULATION)
FSMState::FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString)
            :_ctrlComp(ctrlComp), _stateName(stateName), _stateNameString(stateNameString){
    _lowCmd = _ctrlComp->lowCmd;
    _lowState = _ctrlComp->lowState;
}
#endif

#if defined(COMPILE_WITH_REAL_ROBOT_FREE_DOG)
FSMState::FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString)
            :_ctrlComp(ctrlComp), _stateName(stateName), _stateNameString(stateNameString){
    _lowCmd = _ctrlComp->lowCmd_;
    _lowState = _ctrlComp->lowState_;
}
#endif

