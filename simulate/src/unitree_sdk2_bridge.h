#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <array>
#include <iostream>
#include <mutex>

#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    int find_sensor_adr(std::initializer_list<const char*> names) const
    {
        for (const char* name : names) {
            int sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, name);
            if (sensor_id >= 0) {
                return mj_model_->sensor_adr[sensor_id];
            }
        }
        return -1;
    }

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

        // Primary IMU / state estimator signals. For R1 we use pelvis signals.
        imu_quat_adr_ = find_sensor_adr({"imu_quat", "orientation_pelvis"});
        imu_gyro_adr_ = find_sensor_adr({"imu_gyro", "gyro_pelvis"});
        imu_acc_adr_ = find_sensor_adr({"imu_acc", "accelerometer_pelvis"});

        // Optional base position / velocity outputs.
        frame_pos_adr_ = find_sensor_adr({"frame_pos"});
        frame_vel_adr_ = find_sensor_adr({"frame_vel", "global_linvel_pelvis"});

        // Secondary IMU. For R1 we use torso signals.
        secondary_imu_quat_adr_ = find_sensor_adr({"secondary_imu_quat", "orientation_torso"});
        secondary_imu_gyro_adr_ = find_sensor_adr({"secondary_imu_gyro", "gyro_torso"});
        secondary_imu_acc_adr_ = find_sensor_adr({"secondary_imu_acc", "accelerometer_torso"});
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }
            
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};

class R1Bridge : public UnitreeSDK2BridgeBase
{
public:
    R1Bridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd_subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>>("rt/lowcmd");
        lowstate_publisher_ = std::make_unique<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");
        secondary_imustate_publisher_ = std::make_unique<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::IMUState_>>("rt/secondary_imu");
        highstate_publisher_ = std::make_unique<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
    }

    void start() override
    {
        lowcmd_subscriber_->InitChannel(std::bind(&R1Bridge::LowCmdHandler, this, std::placeholders::_1), 1);
        lowstate_publisher_->InitChannel();
        secondary_imustate_publisher_->InitChannel();
        highstate_publisher_->InitChannel();

        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "r1_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

private:
    static constexpr int kR1MotorCount = 26;
    static constexpr std::array<int, kR1MotorCount> kJointIdxInIdl = {
        0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11,
        12, 13,
        15, 16, 17, 18, 19,
        22, 23, 24, 25, 26,
        29, 30
    };

    void LowCmdHandler(const void* message)
    {
        std::lock_guard<std::mutex> lock(lowcmd_mutex_);
        latest_lowcmd_ = *static_cast<const unitree_hg::msg::dds_::LowCmd_*>(message);
    }

    void fill_imu_state(unitree_hg::msg::dds_::IMUState_& imu_state, int quat_adr, int gyro_adr, int acc_adr)
    {
        if (quat_adr >= 0) {
            imu_state.quaternion()[0] = mj_data_->sensordata[quat_adr + 0];
            imu_state.quaternion()[1] = mj_data_->sensordata[quat_adr + 1];
            imu_state.quaternion()[2] = mj_data_->sensordata[quat_adr + 2];
            imu_state.quaternion()[3] = mj_data_->sensordata[quat_adr + 3];

            double w = imu_state.quaternion()[0];
            double x = imu_state.quaternion()[1];
            double y = imu_state.quaternion()[2];
            double z = imu_state.quaternion()[3];

            imu_state.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
            imu_state.rpy()[1] = asin(2 * (w * y - z * x));
            imu_state.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        }

        if (gyro_adr >= 0) {
            imu_state.gyroscope()[0] = mj_data_->sensordata[gyro_adr + 0];
            imu_state.gyroscope()[1] = mj_data_->sensordata[gyro_adr + 1];
            imu_state.gyroscope()[2] = mj_data_->sensordata[gyro_adr + 2];
        }

        if (acc_adr >= 0) {
            imu_state.accelerometer()[0] = mj_data_->sensordata[acc_adr + 0];
            imu_state.accelerometer()[1] = mj_data_->sensordata[acc_adr + 1];
            imu_state.accelerometer()[2] = mj_data_->sensordata[acc_adr + 2];
        }
    }

    void run()
    {
        if (!mj_data_) return;

        unitree_hg::msg::dds_::LowCmd_ lowcmd_msg;
        {
            std::lock_guard<std::mutex> lock(lowcmd_mutex_);
            lowcmd_msg = latest_lowcmd_;
        }

        for (int i = 0; i < num_motor_ && i < kR1MotorCount; ++i) {
            const auto& motor_cmd = lowcmd_msg.motor_cmd()[kJointIdxInIdl[i]];
            mj_data_->ctrl[i] = motor_cmd.tau() +
                                motor_cmd.kp() * (motor_cmd.q() - mj_data_->sensordata[i]) +
                                motor_cmd.kd() * (motor_cmd.dq() - mj_data_->sensordata[i + num_motor_]);
        }

        unitree_hg::msg::dds_::LowState_ lowstate_msg;
        for (int i = 0; i < num_motor_ && i < kR1MotorCount; ++i) {
            auto& motor_state = lowstate_msg.motor_state()[kJointIdxInIdl[i]];
            motor_state.q() = mj_data_->sensordata[i];
            motor_state.dq() = mj_data_->sensordata[i + num_motor_];
            motor_state.tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
        }

        fill_imu_state(lowstate_msg.imu_state(), imu_quat_adr_, imu_gyro_adr_, imu_acc_adr_);
        lowstate_msg.tick() = std::round(mj_data_->time / 1e-3);

        if (joystick) {
            joystick->update();
            auto data = joystick->combine();
            memcpy(&lowstate_msg.wireless_remote()[0], &data, sizeof(unitree::common::REMOTE_DATA_RX));
        }

        lowstate_msg.crc() = crc32_core(reinterpret_cast<uint32_t*>(&lowstate_msg), (sizeof(lowstate_msg) >> 2) - 1);
        lowstate_publisher_->Write(lowstate_msg);

        if (secondary_imu_quat_adr_ >= 0 || secondary_imu_gyro_adr_ >= 0 || secondary_imu_acc_adr_ >= 0) {
            unitree_hg::msg::dds_::IMUState_ torso_imu_msg;
            fill_imu_state(torso_imu_msg, secondary_imu_quat_adr_, secondary_imu_gyro_adr_, secondary_imu_acc_adr_);
            secondary_imustate_publisher_->Write(torso_imu_msg);
        }

        unitree_go::msg::dds_::SportModeState_ highstate_msg;
        highstate_msg.position()[0] = static_cast<float>(mj_data_->qpos[0]);
        highstate_msg.position()[1] = static_cast<float>(mj_data_->qpos[1]);
        highstate_msg.position()[2] = static_cast<float>(mj_data_->qpos[2]);
        highstate_msg.velocity()[0] = static_cast<float>(mj_data_->qvel[0]);
        highstate_msg.velocity()[1] = static_cast<float>(mj_data_->qvel[1]);
        highstate_msg.velocity()[2] = static_cast<float>(mj_data_->qvel[2]);
        highstate_msg.yaw_speed() = static_cast<float>(mj_data_->qvel[5]);
        highstate_publisher_->Write(highstate_msg);
    }

    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>> lowcmd_subscriber_;
    std::unique_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowState_>> lowstate_publisher_;
    std::unique_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::IMUState_>> secondary_imustate_publisher_;
    std::unique_ptr<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::SportModeState_>> highstate_publisher_;
    unitree_hg::msg::dds_::LowCmd_ latest_lowcmd_;
    std::mutex lowcmd_mutex_;
    unitree::common::RecurrentThreadPtr thread_;
};
