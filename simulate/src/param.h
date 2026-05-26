#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <vector>

namespace param
{

inline struct SimulationConfig
{
    struct JointDynamicsOverride
    {
        std::vector<std::string> name_filters;
        double damping_scale = 1.0;
        double frictionloss_scale = 1.0;
        double armature_scale = 1.0;
    };

    struct MotorSensorOffset
    {
        std::vector<std::string> name_filters;
        double offset = 0.0;
    };

    struct BodyInertialScale
    {
        std::vector<std::string> name_filters;
        double mass_scale = 1.0;
        double inertia_scale = 1.0;
    };

    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;
    double sim_dt = 0.0;
    int enable_joint_dynamics_scaling = 0;
    double joint_damping_scale = 1.0;
    double joint_frictionloss_scale = 1.0;
    double joint_armature_scale = 1.0;
    std::vector<std::string> joint_dynamics_name_filters;
    std::vector<JointDynamicsOverride> joint_dynamics_overrides;
    std::vector<MotorSensorOffset> motor_sensor_offsets;
    std::vector<BodyInertialScale> body_inertial_scales;
    int validation_log_enabled = 0;
    std::filesystem::path validation_log_file = "/tmp/et1_mujoco_validation.csv";
    int validation_log_decimation = 10;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
            if (cfg["sim_dt"]) {
                sim_dt = cfg["sim_dt"].as<double>();
            }
            if (cfg["enable_joint_dynamics_scaling"]) {
                enable_joint_dynamics_scaling = cfg["enable_joint_dynamics_scaling"].as<int>();
            }
            if (cfg["joint_damping_scale"]) {
                joint_damping_scale = cfg["joint_damping_scale"].as<double>();
            }
            if (cfg["joint_frictionloss_scale"]) {
                joint_frictionloss_scale = cfg["joint_frictionloss_scale"].as<double>();
            }
            if (cfg["joint_armature_scale"]) {
                joint_armature_scale = cfg["joint_armature_scale"].as<double>();
            }
            if (cfg["joint_dynamics_name_filters"]) {
                joint_dynamics_name_filters = cfg["joint_dynamics_name_filters"].as<std::vector<std::string>>();
            }
            if (cfg["joint_dynamics_overrides"]) {
                joint_dynamics_overrides.clear();
                for (const auto &override_cfg : cfg["joint_dynamics_overrides"]) {
                    JointDynamicsOverride item;
                    if (override_cfg["name_filters"]) {
                        item.name_filters = override_cfg["name_filters"].as<std::vector<std::string>>();
                    }
                    if (override_cfg["damping_scale"]) {
                        item.damping_scale = override_cfg["damping_scale"].as<double>();
                    }
                    if (override_cfg["frictionloss_scale"]) {
                        item.frictionloss_scale = override_cfg["frictionloss_scale"].as<double>();
                    }
                    if (override_cfg["armature_scale"]) {
                        item.armature_scale = override_cfg["armature_scale"].as<double>();
                    }
                    joint_dynamics_overrides.push_back(item);
                }
            }
            if (cfg["motor_sensor_offsets"]) {
                motor_sensor_offsets.clear();
                for (const auto &offset_cfg : cfg["motor_sensor_offsets"]) {
                    MotorSensorOffset item;
                    if (offset_cfg["name_filters"]) {
                        item.name_filters = offset_cfg["name_filters"].as<std::vector<std::string>>();
                    }
                    if (offset_cfg["offset"]) {
                        item.offset = offset_cfg["offset"].as<double>();
                    }
                    motor_sensor_offsets.push_back(item);
                }
            }
            if (cfg["body_inertial_scales"]) {
                body_inertial_scales.clear();
                for (const auto &scale_cfg : cfg["body_inertial_scales"]) {
                    BodyInertialScale item;
                    if (scale_cfg["name_filters"]) {
                        item.name_filters = scale_cfg["name_filters"].as<std::vector<std::string>>();
                    }
                    if (scale_cfg["mass_scale"]) {
                        item.mass_scale = scale_cfg["mass_scale"].as<double>();
                    }
                    if (scale_cfg["inertia_scale"]) {
                        item.inertia_scale = scale_cfg["inertia_scale"].as<double>();
                    } else {
                        item.inertia_scale = item.mass_scale;
                    }
                    body_inertial_scales.push_back(item);
                }
            }
            if (cfg["validation_log_enabled"]) {
                validation_log_enabled = cfg["validation_log_enabled"].as<int>();
            }
            if (cfg["validation_log_file"]) {
                validation_log_file = cfg["validation_log_file"].as<std::string>();
            }
            if (cfg["validation_log_decimation"]) {
                validation_log_decimation = cfg["validation_log_decimation"].as<int>();
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
        ("sim_dt", po::value<double>(&config.sim_dt), "Override MuJoCo timestep after loading XML; --sim_dt 0.001")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}
