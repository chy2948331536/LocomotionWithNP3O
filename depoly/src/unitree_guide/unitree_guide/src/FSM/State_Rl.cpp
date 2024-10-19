#include <iostream>
#include "FSM/State_Rl.h"
#include <cmath>
#define _USE_MATH_DEFINES
using namespace std;


State_Rl::State_Rl(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::RL, "rl"),
    _est(ctrlComp->estimator)
{

}

void State_Rl::enter()
{
    // load policy
    model_path = "/home/chy/Downloads/LocomotionWithNP3O/model_9900_deploy.pt";
    load_policy();
    indices = torch::tensor({3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8}, 
                            torch::TensorOptions().dtype(torch::kLong).device(device));

    // initialize record
    action_buf = torch::zeros({history_length,12},device);
//    obs_buf = torch::zeros({history_length,30},device);
    obs_buf = torch::zeros({history_length,45},device);
    last_action = torch::zeros({1,12},device);

    action_buf.to(torch::kHalf);
    obs_buf.to(torch::kHalf);
    last_action.to(torch::kHalf);

    // initialize default values
    gravity(0,0) = 0.0;
    gravity(1,0) = 0.0;
    gravity(2,0) = -1.0;

    _gxFilter = new LPFilter(infer_dt,3.0);
    _gyFilter = new LPFilter(infer_dt,3.0);
    _gzFilter = new LPFilter(infer_dt,3.0);

    forward(0,0) = 1.0;
    forward(1,0) = 0.0;
    forward(2,0) = 0.0;

    for (int j = 0; j < 12; j++)
    {
        action_temp.push_back(0.0);
        //action.push_back(_lowState->motorState[j].q);
        //prev_action.push_back(_lowState->motorState[j].q);
	action.push_back(init_pos[j]);
        prev_action.push_back(init_pos[j]);

    }
//    for (int j = 0; j < 12; j++)
//    {
//        action.push_back(0.0);
//    }

    for (int i = 0; i < history_length; i++)
    {
        torch::Tensor obs_tensor = get_obs();
        // append obs to obs buffer
        obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);
    }
    std::cout << "init finised predict" << std::endl;

    for (int i = 0; i < 200; i++)
    {
        model_infer();
    }

    // initialize thread
    threadRunning = true;
    infer_thread = new std::thread(&State_Rl::infer,this);


    // smooth transition of kp and kd
    for (int j = 0; j < 12; j++)
    {
        stand_kd[j] = _lowCmd->motorCmd[j].Kd;
        stand_kp[j] = _lowCmd->motorCmd[j].Kp;
    }
    // for(int i = 0; i < _duration_1; i++) {
    //     _percent_1 += (float) 1 / _duration_1;
    //     _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
    //     if (_percent_1 < 1) {
    //         for (int j = 0; j < 12; j++) {
    //             _lowCmd->motorCmd[j].Kp = (1 - _percent_1) * stand_kp[j] + _percent_1 * Kp;
    //             _lowCmd->motorCmd[j].Kd = (1 - _percent_1) * stand_kd[j] + _percent_1 * Kd;
    //         }
    //     }
    // }
    for (int j = 0; j < 12; j++)
    {
      action_filters.push_back(new LPFilter(0.002,20.0));
    }
}

void State_Rl::run()
{   
    if (_percent_1 < 1)
    {
        _percent_1 += (float) 1 / _duration_1;
        _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
        write_cmd_lock.lock();
        if (_percent_1 < 1) {
            for (int j = 0; j < 12; j++) {
                _lowCmd->motorCmd[j].mode = 10;
		_lowCmd->motorCmd[j].q = (1 - _percent_1) * _startPos[j] + _percent_1 * action[j];
                _lowCmd->motorCmd[j].dq = 0;
                _lowCmd->motorCmd[j].Kp = (1 - _percent_1) * stand_kp[j] + _percent_1 * Kp;
                _lowCmd->motorCmd[j].Kd = (1 - _percent_1) * stand_kd[j] + _percent_1 * Kd;
                _lowCmd->motorCmd[j].tau = 0;
            }
            // std::cout << (1 - _percent_1) << std::endl;
        } 
        write_cmd_lock.unlock();
    }
    else
    {
        if (hang_on_change_state == 1){
            _percent_2 = static_cast<float>(getSystemTime() - hang_on_change_begin_time)/2000000.0f;
            _percent_2 = std::min(_percent_2, 1.0f);
            for (int i = 0; i < 9; i++)
            {
                write_cmd_lock.lock();
                _lowCmd->motorCmd[i].q = (1 - _percent_2) * _targetPos_3[i] + _percent_2 * _targetPos_2[i];
                // std::cout << _lowCmd->motorCmd[i].q << std::endl;
                write_cmd_lock.unlock();
            }
            // std::cout << "\n" <<"\n" <<std::endl;
            // std::cout << _percent_2 << std::endl;
        }
        else if (hang_on_change_state == 2){
            _percent_2 = static_cast<float>(getSystemTime() - hang_on_change_begin_time)/2000000.0f;
            _percent_2 = std::min(_percent_2, 1.0f);
            for (int i = 0; i < 12; i++)
            {
                write_cmd_lock.lock();
                _lowCmd->motorCmd[i].q = (1 - _percent_2) * _targetPos_2[i] + _percent_2 * _targetPos_3[i];
                // std::cout << _lowCmd->motorCmd[i].q << std::endl;
                write_cmd_lock.unlock();
            }
            // std::cout << "\n" <<"\n" <<std::endl;

            // std::cout << _percent_2 << std::endl;
        }
        else{
    //        _percent_2 += (float) 1 / _duration_2;
    //        _percent_2 = _percent_2 > 1 ? 1 : _percent_2;
            write_cmd_lock.lock();
            for (int j = 0; j < 12; j++)
            {
                //float target = (1-_percent_2)*prev_action[j]+_percent_2*action[j];
                //action_filters[j]->addValue(action[j]);
                _lowCmd->motorCmd[j].mode = 10;
                _lowCmd->motorCmd[j].q = action[j];//action_filters[j]->getValue();
                _lowCmd->motorCmd[j].dq = 0;
                _lowCmd->motorCmd[j].Kp = Kp;
                _lowCmd->motorCmd[j].Kd = Kd;
                _lowCmd->motorCmd[j].tau = 0;

    //            if(_percent_2 == 1)
    //            {
    //                prev_action[j] = action[j];
    //            }
            }
            write_cmd_lock.unlock();
        }
    }
   
}

void State_Rl::exit()
{
    _percent_1 = 0;
    _percent_2 = 0;
    threadRunning = false;
    infer_thread->join();
}

torch::Tensor State_Rl::get_obs()
{
    std::vector<float> obs;
    // compute gravity
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    Vec3 angvel = _lowState->getGyro();
    Vec3 projected_gravity = _G2B_RotMat*gravity;
    Vec3 projected_forward = _G2B_RotMat*forward;
    // gravity
    _gxFilter->addValue(angvel(0,0));
    _gyFilter->addValue(angvel(1,0));
    _gzFilter->addValue(angvel(2,0));
//
    obs.push_back(_gxFilter->getValue()*0.25);
    obs.push_back(_gyFilter->getValue()*0.25);
    obs.push_back(_gzFilter->getValue()*0.25);

    for (int i = 0; i < 3; ++i)
    {
        obs.push_back(projected_gravity(i,0));
    }

    // cmd
    rx = rx * (1 - smooth) + (std::fabs(_lowState->userValue.rx) < dead_zone ? 0.0 : _lowState->userValue.rx) * smooth;
    ly = ly * (1 - smooth) + (std::fabs(_lowState->userValue.ly) < dead_zone ? 0.0 : _lowState->userValue.ly) * smooth;
    lx = lx * (1 - smooth) + (std::fabs(_lowState->userValue.lx) < dead_zone ? 0.0 : _lowState->userValue.lx) * smooth;

    float max = 1.0;
    float min = -1.0;
//    float vel = std::max(std::min(_lowState->userValue.ly, max), min);
//    if(vel > 0)
//    {
//        vel = 1.0;
//    }
//    else if(vel < 0)
//    {
//        vel = -1.0;
//    }
//    float  vel = 0.6;
    float rot = rx*3.14;
    float vel = ly*2.0;
    float lat_vel = lx*2.0;

    double heading = atan2((double)forward(1,0), (double)forward(0,0));
    double angle = (double)rot - heading;
    angle = fmod(angle,2.0*M_PI);
    if(angle > M_PI)
    {
        angle = angle - 2.0*M_PI;
    }
    angle = angle*0.5;
    angle = std::max(std::min((float)angle, max), min);
    angle = angle * 0.25;

    obs.push_back(vel);
    obs.push_back(lat_vel);
    obs.push_back(angle);

    // std::vector<float> posArray;
    // std::vector<float> velArray;
    // pos
    for (int i = 0; i < 12; ++i)
    {
        float pos = (_lowState->motorState[i].q  - init_pos[i])* pos_scale;
        // posArray.push_back(_lowState->motorState[i].q);
        obs.push_back(pos);
    }
    
    // vel
    for (int i = 0; i < 12; ++i)
    {
        float vel = _lowState->motorState[i].dq * vel_scale;
        // velArray.push_back(_lowState->motorState[i].dq);
        obs.push_back(vel);
    }

    // std::cout << "Position Array: ";
    // for (const auto& pos : posArray) {
    //     std::cout << pos << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "Velocity Array: ";
    // for (const auto& vel : velArray) {
    //     std::cout << vel << " ";
    // }
    // std::cout << std::endl;
    
    // last action
    //float index[12] = {3,4,5,0,1,2,9,10,11,6,7,8};
    for (int i = 0; i < 12; ++i)
    {
        obs.push_back(action_temp[i]);
    }

    // gravity,cmd,dof_pos,dof_vel to tensor
    auto options = torch::TensorOptions().dtype(torch::kFloat32);
//    torch::Tensor obs_tensor = torch::from_blob(obs.data(),{1,30},options).to(device);
    torch::Tensor obs_tensor = torch::from_blob(obs.data(),{1,45},options).to(device);

    return obs_tensor;
}

torch::Tensor State_Rl::model_infer()
{   
    torch::NoGradGuard no_grad;

    torch::Tensor obs_tensor = get_obs();
    // std::cout << "obs_tensor: " << obs_tensor << std::endl;
    //obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);
//    auto obs_buf_batch = obs_buf.unsqueeze(0);
//    auto action_buf_batch = action_buf.unsqueeze(0);
//
//    std::vector<torch::jit::IValue> inputs;
//    inputs.push_back(obs_buf_batch);
//    inputs.push_back(action_buf_batch);

    //auto obs_batch = obs_tensor.unsqueeze(0);
    auto obs_buf_batch = obs_buf.unsqueeze(0);
    // std::cout << "obs_buf_batch shape: " << obs_buf_batch.sizes() << std::endl;
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(obs_tensor.to(torch::kHalf));
    inputs.push_back(obs_buf_batch.to(torch::kHalf));
    // std::cout << obs_tensor << std::endl;
    // for (size_t i = 0; i < inputs.size(); i++) {
    //     std::cout << "inputs[" << i << "] shape: " << inputs[i].toTensor().sizes() << std::endl;
    // }
    // Execute the model and turn its output into a tensor.
    torch::Tensor action_tensor = model.forward(inputs).toTensor();
    action_buf = torch::cat({action_buf.index({Slice(1,None),Slice()}),action_tensor},0);
    // action_getter = action_tensor.accessor<float,1>();
    // std::cout << "action_buf" << action_buf << std::endl;
    // std::cout << "action_buf" << action_tensor << std::endl;
    // torch::Tensor action_blend_tensor = 0.8*action_tensor + 0.2*last_action;
    last_action = action_tensor.clone();

    obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);

    return action_tensor;
}
// long long last_time = getSystemTime();  // 记录初始时间
void State_Rl::infer()
{
    while(threadRunning)
    {
        // long long current_time = getSystemTime();  // 记录初始时间
        // long long time_difference = current_time - last_time;
        // std::cout << "Time interval since last loop: " << time_difference << " microseconds" << std::endl;
        // last_time = current_time;
        long long _start_time = getSystemTime();
        // std::cout << _start_time << std::endl;
        //torch::Tensor obs_tensor = get_obs();
        // append obs to obs buffer
        //obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);

        torch::Tensor action_raw = model_infer();
        // std::cout << "action_raw: " << action_raw << std::endl;
//        obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);
        // action filter
//        action_raw = 0.8*action_raw + 0.2*last_action;
        //last_action = action_raw.clone();
        // append to action buffer
        // action_buf = torch::cat({action_buf.index({Slice(1,None),Slice()}),action_raw},0);
        // assign to control
        torch::Tensor action_blend_tensor = 0.8*action_raw + 0.2*last_action;
        action_raw = action_raw.squeeze(0).to(torch::kFloat32).to(torch::kCPU);
        action_blend_tensor = action_blend_tensor.squeeze(0).to(torch::kFloat32).to(torch::kCPU);
        // assess the result
        auto action_getter = action_raw.accessor<float,1>();
        auto action_blend_getter = action_blend_tensor.accessor<float,1>();

        write_cmd_lock.lock();
        for (int j = 0; j < 12; j++)
        {
//            float  action_value = std::max(std::min(action_getter[j]* action_scale[j], action_delta_max), action_delta_min);
//            action.at(j) = action_value + init_pos[j];
//            action_temp.at(j) = action_value/action_scale[j];
            action[j] = action_blend_getter[j] * action_scale[j] * hip_scale[j] + init_pos[j];
            action_temp[j] = action_getter[j];
        }
        write_cmd_lock.unlock();

        absoluteWait(_start_time, (long long)(infer_dt * 1000000));
    }
    threadRunning = false;

}

void State_Rl::load_policy()
{   
    std::cout << model_path << std::endl;
    // load model from check point
    std::cout << "cuda::is_available():" << torch::cuda::is_available() << std::endl;
    device= torch::kCPU;
    if (torch::cuda::is_available()){
        device = torch::kCUDA;
    }
    model = torch::jit::load(model_path);
    std::cout << "load model is successed!" << std::endl;
    model.to(device);
    model.to(torch::kHalf);
    std::cout << "load model to device!" << std::endl;
    model.eval();
}

FSMStateName State_Rl::checkChange()
{
    if(_lowState->userCmd == UserCommand::L1_Y){
        hang_on_change_state = 0;
        // std::cout << "hang_on_change_state" << hang_on_change_state << std::endl;
        return FSMStateName::RL;
    }
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    if(_lowState->userCmd == UserCommand::L2_A){
        std::cout << "FSMStateName::FIXEDSTAND" << std::endl;
        return FSMStateName::FIXEDSTAND;
    }
    if(_lowState->userCmd == UserCommand::L1_X){
        if (hang_on_change_state == 0 || hang_on_change_state == 2){
            hang_on_change_begin_time = getSystemTime();
            hang_on_change_state = 1;
            std::cout << "hang_on_change_state" << hang_on_change_state << std::endl;
            std::cout << "hang_on_change_begin_time" << hang_on_change_begin_time << std::endl;
        }
        return FSMStateName::RL;
    }
    if(_lowState->userCmd == UserCommand::L1_A){
        if (hang_on_change_state == 0 || hang_on_change_state == 1){
            hang_on_change_begin_time = getSystemTime();
            hang_on_change_state = 2;
            std::cout << "hang_on_change_state" << hang_on_change_state << std::endl;
            std::cout << "hang_on_change_begin_time" << hang_on_change_begin_time << std::endl;
            return FSMStateName::RL;
        }
    }
    return FSMStateName::RL;
}
