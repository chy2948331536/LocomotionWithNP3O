/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_REAL_ROBOT_FREE_DOG

#include "interface/IOFREEDOGSDK.h"

IOFREEDOGSDK::IOFREEDOGSDK():IOInterface()
{
 udp_ = std::make_shared<FDSC::UnitreeConnection>("LOW_WIRED_DEFAULTS");
 udp_->startRecv();
 std::vector<uint8_t> cmdBytes = lowCmd_.buildCmd(false);
 udp_->send(cmdBytes);
 std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Some time to collect packets
 cmdPanel = new Gamepad();
}

IOFREEDOGSDK::~IOFREEDOGSDK(){
 delete cmdPanel;
 ros::shutdown();
}

void IOFREEDOGSDK::recvState(FDSC::lowState *state) {
 std::vector<std::vector<uint8_t>> dataAll;
 udp_->getData(dataAll);
 if (!dataAll.empty()) {
  std::vector<uint8_t> data = dataAll.at(dataAll.size() - 1);
  lowState_.parseData(data);
  }
 // for (int i = 0; i < 12; ++i) {
 //  jointData_[i].pos_ = lowState_.motorState_free_dog[i].q;
 //  jointData_[i].vel_ = lowState_.motorState_free_dog[i].dq;
 //  jointData_[i].tau_ = lowState_.motorState_free_dog[i].tauEst;
 //  std::cout << "Joint " << i << " - Pos: " << jointData_[i].pos_
 //              << ", Vel: " << jointData_[i].vel_
 //              << ", Tau: " << jointData_[i].tau_ << std::endl;
 // }

 // imuData_.ori_[0] = lowState_.imu_quaternion[1];
 // imuData_.ori_[1] = lowState_.imu_quaternion[2];
 // imuData_.ori_[2] = lowState_.imu_quaternion[3];
 // imuData_.ori_[3] = lowState_.imu_quaternion[0];
 // imuData_.angularVel_[0] = lowState_.imu_gyroscope[0];
 // imuData_.angularVel_[1] = lowState_.imu_gyroscope[1];
 // imuData_.angularVel_[2] = lowState_.imu_gyroscope[2];
 // imuData_.linearAcc_[0] = lowState_.imu_accelerometer[0];
 // imuData_.linearAcc_[1] = lowState_.imu_accelerometer[1];
 // imuData_.linearAcc_[2] = lowState_.imu_accelerometer[2];
 // std::cout << "IMU Orientation: [" << imuData_.ori_[0] << ", " << imuData_.ori_[1] << ", " << imuData_.ori_[2] << ", " << imuData_.ori_[3] << "]" << std::endl;
 // std::cout << "IMU Angular Velocity: [" << imuData_.angularVel_[0] << ", " << imuData_.angularVel_[1] << ", " << imuData_.angularVel_[2] << "]" << std::endl;
 // std::cout << "IMU Linear Acceleration: [" << imuData_.linearAcc_[0] << ", " << imuData_.linearAcc_[1] << ", " << imuData_.linearAcc_[2] << "]" << std::endl;
  for(int i(0); i < 12; ++i){
   state->motorState[i].q = lowState_.motorState_free_dog[i].q;
   state->motorState[i].dq = lowState_.motorState_free_dog[i].dq;
   state->motorState[i].ddq = lowState_.motorState_free_dog[i].ddq;
   state->motorState[i].tauEst = lowState_.motorState_free_dog[i].tauEst;
   // std::cout << "Motor " << i << " - Pos: " << state->motorState[i].q
   //           << ", Vel: " << state->motorState[i].dq
   //           << ", Tau: " << state->motorState[i].tauEst << std::endl;
  }
  for(int i(0); i < 3; ++i){
   state->imu.quaternion[i] = lowState_.imu_quaternion[i];
   state->imu.accelerometer[i] = lowState_.imu_accelerometer[i];
   state->imu.gyroscope[i] = lowState_.imu_gyroscope[i];

  }
  state->imu.quaternion[3] = lowState_.imu_quaternion[3];
  // std::cout << "IMU Orientation: [" << state->imu.quaternion[0] << ", " << state->imu.quaternion[1] << ", " << state->imu.quaternion[2] << ", " << state->imu.quaternion[3] << "]" << std::endl;
  // std::cout << "IMU Angular Velocity: [" << state->imu.gyroscope[0] << ", " << state->imu.gyroscope[1] << ", " << state->imu.gyroscope[2] << "]" << std::endl;
  // std::cout << "IMU Linear Acceleration: [" << state->imu.accelerometer[0] << ", " << state->imu.accelerometer[1] << ", " << state->imu.accelerometer[2] << "]" << std::endl;
}

void IOFREEDOGSDK::sendCmd(const FDSC::lowCmd *lowCmd) {
 FDSC::MotorCmdArray cmdArr;
 for (int i = 0; i < 12; ++i) {
  lowCmd_.motorCmd[i].mode = lowCmd->motorCmd[i].mode;
  lowCmd_.motorCmd[i].q = lowCmd->motorCmd[i].q;
  lowCmd_.motorCmd[i].dq = lowCmd->motorCmd[i].dq;
  lowCmd_.motorCmd[i].Kp = lowCmd->motorCmd[i].Kp;
  lowCmd_.motorCmd[i].Kd = lowCmd->motorCmd[i].Kd;
  lowCmd_.motorCmd[i].tau = lowCmd->motorCmd[i].tau;
  jointData_[i].posDes_ = lowCmd_.motorCmd[i].q;
  jointData_[i].kp_ = lowCmd_.motorCmd[i].Kp;
  jointData_[i].kd_ = lowCmd_.motorCmd[i].Kd;
  std::vector<double> joint{jointData_[i].posDes_, jointData_[i].velDes_, jointData_[i].ff_, jointData_[i].kp_, jointData_[i].kd_};
  // std::cout << jointData_[i].posDes_ << " " << jointData_[i].velDes_ << " " << jointData_[i].ff_ << " " << jointData_[i].kp_ << " " << jointData_[i].kd_ << std::endl;
  cmdArr.setMotorCmd(i, FDSC::MotorModeLow::Servo, joint);
 }
 lowCmd_.motorCmd_free_dog = cmdArr;
 std::vector<uint8_t> cmdBytes = lowCmd_.buildCmd(false);
 udp_->send(cmdBytes);
}


void IOFREEDOGSDK::sendRecv(const FDSC::lowCmd *lowCmd_, FDSC::lowState *lowState_){
 // std::cout << "Sending command to robot..." << std::endl;
 recvState(lowState_);
 // for (int i = 0; i < 12; ++i) {
 //  std::cout << "Motor " << i << " - Mode: " << lowCmd_->motorCmd[i].mode
 //            << ", Q: " << lowCmd_->motorCmd[i].q
 //            << ", DQ: " << lowCmd_->motorCmd[i].dq
 //            << ", Kp: " << lowCmd_->motorCmd[i].Kp
 //            << ", Kd: " << lowCmd_->motorCmd[i].Kd
 //            << ", Tau: " << lowCmd_->motorCmd[i].tau << std::endl;
 // }
 sendCmd(lowCmd_);
 lowState_->userCmd = cmdPanel->getUserCmd();
 lowState_->userValue = cmdPanel->getUserValue();

}
#endif

