#pragma once

#include <array>
#include <vector>
#include <filesystem>
#include <fstream>
#include <string>
#include <torch/torch.h>
#include <torch/script.h> 

#include "robot_interface.hpp"
#include "gamepad.hpp"
#include "cfg.hpp"

using namesapce at::indexing;

namespace fs = std::filesystem;

namespace unitree::common
{
    class BasicUserController
    {
    public:
        BasicUserController() {}

        virtual void LoadParam(fs::path &param_folder) = 0;

        virtual void Reset(RobotInterface &robot_interface, Gamepad &gamepad) = 0;

        virtual void GetInput(RobotInterface &robot_interface, Gamepad &gamepad) = 0;

        virtual void Calculate() = 0;

        virtual std::vector<float> GetLog() = 0;

        float dt, kp, kd, action_scale;
        std::array<float, 12> init_pos;
        std::array<float, 12> jpos_des;
    };

    class ExampleUserController : public BasicUserController
    {
    public:
        ExampleUserController() {}

        void LoadParam(fs::path &param_folder)
        {
            // load param file
            std::ifstream cfg_file(param_folder / "params.json");
            std::cout << "Read params from: " << param_folder / "params.json" << std::endl;
            std::stringstream ss;
            ss << cfg_file.rdbuf();
            FromJsonString(ss.str(), cfg);

            // get data from json
            dt = cfg.dt;
            kp = cfg.kp;
            kd = cfg.kd;
            action_scale = cfg.action_scale;
            command_scale = cfg.command_scale;
            pos_scale = cfg.pos_scale;
            vel_scale = cfg.vel_scale;
            history_length = cfg.history_length;
            model_path = cfg.model_path;

            for (int i = 0; i < 12; ++i)
            {
                init_pos.at(i) = cfg.init_pos.at(i);
            }

            // load model from check point
            std::cout << "cuda::is_avaliable():" << torch::cuda::is_avaliable() << std::endl;
            torch::DeviceType device_type = at::kCPU;
            if (torch::cuda::is_avaliable()){
                device_type = at::kCUDA;
            }

            model = torch::jit::load(model_path)
            assert(module != nullptr);
            std::cout << "load model is successed!" << std::endl
            model.to(device_type)
            std::cout << "load model to device!" << std::endl
            model.eval();

            // initialize record 
            auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA);

            action_buf = torch::zeros({history_length,12}).to(device_type);
            obs_buf = torch::zeros({history_length,34}).to(device_type);

            init_pos_tensor = torch::from_blob(init_pos,{1,12},options).to(device_type);
            last_action_tensor = torch::zeros({1,12}).to(device_type);
            last_contact_tensor = torch::ones({1,4}).to(device_type);
        }

        void GetInput(RobotInterface &robot_interface, Gamepad &gamepad)
        {
            // record command
            cmd.at(0) = gamepad.ly;
            cmd.at(1) = 0;//-gamepad.lx;
            cmd.at(2) = 0;//-gamepad.rx;

            // record robot state
            for (int i = 0; i < 12; ++i)
            {
                jpos_processed.at(i) = robot_interface.jpos.at(i)
                jvel_processed.at(i) = robot_interface.jvel.at(i)
            }
            for (int i = 0; i < 3; ++i)
            {
                gravity.at(i) = robot_interface.projected_gravity.at(i)
            }
            for (int i = 0; i < 4; ++i)
            {
                contact.at(i) = robot_interface.contact.at(i);
            }

            // gravity,cmd,dof_pos,dof_vel to tensor
            auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA);
            torch::Tensor gravity_tensor = torch::from_blob(gravity,{1,3},options).to(device_type);
            torch::Tensor cmd_tensor = torch::from_blob(cmd,{1,3},options).to(device_type);
            torch::Tensor dof_pos_tensor = torch::from_blob(jpos_processed,{1,12},options).to(device_type);
            torch::Tensor dof_vel_tensor = torch::from_blob(jvel_processed,{1,12},options).to(device_type);
            torch::Tensor contact_tensor = torch::from_blob(contact,{1,4},options).to(device_type);
            // scale and offset
            dof_pos_tensor = (dof_pos_tensor - init_pos_tensor)*pos_scale;
            dof_vel_tensor = dof_vel_tensor*vel_scale;
            cmd_tensor = cmd_tensor*command_scale;
            contact_tensor = contact_tensor - 0.5;
            // concat obs
            torch::Tensor obs = torch::cat({gravity_tensor,cmd_tensor,dof_pos_tensor,dof_vel_tensor,contact_tensor},1)
            // append obs to obs buffer
            obs_buf = obs_buf.index({Slice(1,None),Slice()});
            obs_buf = torch::cat({obs_buf,obs},0);
        }

        void Reset(RobotInterface &robot_interface, Gamepad &gamepad)
        {
            GetInput(robot_interface, gamepad);
            Calculate();
        }

        void Calculate()
        {
            std::vector<torch::jit::IValue> inputs;
            inputs.push_back(obs_buf);
            inputs.push_back(action_buf);
            // Execute the model and turn its output into a tensor.
            torch::Tensor action_raw = module->forward(inputs).toTensor();
            // interpolate action
            action = last_action_tensor*0.2 + action_raw*0.8;
            // record action raw
            last_action_tensor = action_raw.clone()
            // append to action buffer
            action_buf = action_buf.index({Slice(1,None),Slice()});
            action_buf = torch::cat({action_buf,action_raw},0)
            // scale action
            action = action * action_scale
            // action modified by default pos
            action = action + init_pos_tensor;
            // assign to control
            auto action_getter = action.accessor<float,1>();
            for (int i = 0; i < 12; ++i)
            {
                jpos_des.at(i) = action_getter[i];
            }
        }

        std::vector<float> GetLog()
        {
            // record input, output and other info into a vector
            std::vector<float> log;
            for (int i = 0; i < 3; ++i)
            {
                log.push_back(cmd.at(i));
            }
            for (int i = 0; i < 12; ++i)
            {
                log.push_back(jpos_processed.at(i));
            }
            for (int i = 0; i < 12; ++i)
            {
                log.push_back(jvel_processed.at(i));
            }
            for (int i = 0; i < 12; ++i)
            {
                log.push_back(jpos_des.at(i));
            }
            
            return log;
        }

        // cfg
        ExampleCfg cfg;

        // state
        std::array<float, 3> cmd;
        std::array<float, 3> gravity;
        std::array<float, 12> jpos_processed;
        std::array<float, 12> jvel_processed;
        std::array<float, 12> action;
        std::array<float, 4> contact;

        //record
        torch::Tensor last_contact_tensor;
        torch::Tensor last_action_tensor;
        torch::Tensor init_pos_tensor;

        //buffer
        torch::Tensor action_buf;
        torch::Tensor obs_buf;
        int history_length;

        //policy model
        string model_path;
        std::shared_ptr<torch::jit::script::Module> model;
    };
} // namespace unitree::common