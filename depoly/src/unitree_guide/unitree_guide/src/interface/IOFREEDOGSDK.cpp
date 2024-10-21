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
}

IOFREEDOGSDK::~IOFREEDOGSDK(){
 delete cmdPanel;
 ros::shutdown();
}

void IOFREEDOGSDK::read() {
 std::vector<std::vector<uint8_t>> dataAll;
 udp_->getData(dataAll);
 if (!dataAll.empty()) {
  std::vector<uint8_t> data = dataAll.at(dataAll.size() - 1);
  lowState_.parseData(data);
  }
 // for (int i = 0; i < 12; ++i) {
 //  jointData_[i].pos_ = lowState_.motorState[i].q;
 //  jointData_[i].vel_ = lowState_.motorState[i].dq;
 //  jointData_[i].tau_ = lowState_.motorState[i].tauEst;
 //  // std::cout << "Joint " << i << " - Pos: " << jointData_[i].pos_
 //  //             << ", Vel: " << jointData_[i].vel_
 //  //             << ", Tau: " << jointData_[i].tau_ << std::endl;
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
 // for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
 //  contactState_[i] = lowState_.footForce[i] > contactThreshold_;
 // }
}

void IOFREEDOGSDK::write() {
 FDSC::MotorCmdArray cmdArr;
 for (int i = 0; i < 12; ++i) {
  jointData_[i].posDes_ = _targetPos_3[i];
  jointData_[i].kp_ = 20;
  jointData_[i].kd_ = 0.5;
  std::vector<double> joint{jointData_[i].posDes_, jointData_[i].velDes_, jointData_[i].ff_, jointData_[i].kp_, jointData_[i].kd_};
  cmdArr.setMotorCmd(i, FDSC::MotorModeLow::Servo, joint);
 }
 lowCmd_.motorCmd_free_dog = cmdArr;
 std::vector<uint8_t> cmdBytes = lowCmd_.buildCmd(false);
 udp_->send(cmdBytes);
}


void IOFREEDOGSDK::sendRecv(const FDSC::lowCmd *lowCmd_, FDSC::lowState *lowState_){
 // std::cout << "Sending command to robot..." << std::endl;
 read();
 write();
}
#endif

