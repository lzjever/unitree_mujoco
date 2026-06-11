#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
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
    int sim_control_port = 8090;
    double sim_control_band_step = 0.1;
    int band_attached_link = -1;
    double sim_dt = 0.0;

#ifdef UNITREE_MUJOCO_GHOST_VIEWER
    int ghost_ref_enable = 0;
    std::string ghost_ref_url = "http://127.0.0.1:8083/_sim/reference_frame";
    double ghost_ref_poll_hz = 25.0;
    int ghost_ref_timeout_ms = 15;
    int ghost_ref_stale_ms = 250;
#endif

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
            if (cfg["sim_control_port"]) {
                sim_control_port = cfg["sim_control_port"].as<int>();
            }
            if (cfg["sim_control_band_step"]) {
                sim_control_band_step = cfg["sim_control_band_step"].as<double>();
            }
            if (cfg["sim_dt"]) {
                sim_dt = cfg["sim_dt"].as<double>();
            }
#ifdef UNITREE_MUJOCO_GHOST_VIEWER
            if (cfg["ghost_ref_enable"]) {
                ghost_ref_enable = cfg["ghost_ref_enable"].as<int>();
            }
            if (cfg["ghost_ref_url"]) {
                ghost_ref_url = cfg["ghost_ref_url"].as<std::string>();
            }
            if (cfg["ghost_ref_poll_hz"]) {
                ghost_ref_poll_hz = cfg["ghost_ref_poll_hz"].as<double>();
            }
            if (cfg["ghost_ref_timeout_ms"]) {
                ghost_ref_timeout_ms = cfg["ghost_ref_timeout_ms"].as<int>();
            }
            if (cfg["ghost_ref_stale_ms"]) {
                ghost_ref_stale_ms = cfg["ghost_ref_stale_ms"].as<int>();
            }
#endif
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
        ("sim_control_port", po::value<int>(&config.sim_control_port), "Local UDP sim-control port, 0 disables it; --sim_control_port 8090")
#ifdef UNITREE_MUJOCO_GHOST_VIEWER
        ("ghost_ref_enable", po::value<int>(&config.ghost_ref_enable), "Enable reference ghost viewer; --ghost_ref_enable 1")
        ("ghost_ref_url", po::value<std::string>(&config.ghost_ref_url), "Reference frame endpoint URL")
        ("ghost_ref_poll_hz", po::value<double>(&config.ghost_ref_poll_hz), "Reference frame polling rate")
        ("ghost_ref_timeout_ms", po::value<int>(&config.ghost_ref_timeout_ms), "Reference frame HTTP timeout in milliseconds")
        ("ghost_ref_stale_ms", po::value<int>(&config.ghost_ref_stale_ms), "Hide reference ghost after this stale age in milliseconds")
#endif
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
