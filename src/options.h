#pragma once

#include <string>

struct AppOptions {
    bool servo2040_enabled = false;
    bool servo2040_autodiscover = false;
    bool servo2040_pwm_sim_enabled = false;
    bool direct_pwm_control_enabled = false;
    int wifi_controller_port = 8080;
    bool show_help = false;
    bool parse_error = false;
    bool config_path_explicit = false;
    bool validate_config_only = false;
    std::string servo2040_port;
    std::string config_path = "proton.conf";
};

void print_usage(const char* exe);
AppOptions parse_options(int argc, char** argv);
