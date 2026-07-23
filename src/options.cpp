#include "options.h"

#include <cstdio>

namespace {

bool has_value(int argc, char** argv, int index)
{
    return index + 1 < argc && argv[index + 1][0] != '-';
}

bool is_auto_value(const std::string& value)
{
    return value == "auto" || value == "detect" || value == "discover";
}

void enable_servo2040_auto(AppOptions& options)
{
    options.servo2040_enabled = false;
    options.servo2040_autodiscover = true;
    options.servo2040_port.clear();
}

void disable_servo2040(AppOptions& options)
{
    options.servo2040_enabled = false;
    options.servo2040_autodiscover = false;
    options.servo2040_port.clear();
}

} // namespace

void print_usage(const char* exe)
{
    std::printf("Usage: %s [--config FILE] [--servo2040 PORT] [--pwm-control] [--port PORT]\n", exe);
    std::printf("\n");
    std::printf("Options:\n");
    std::printf("  --config FILE          Load robot dimensions, servos, and pin map from a .conf file.\n");
    std::printf("  --validate-config      Check the config file and exit without opening the simulator.\n");
    std::printf("  --servo2040 PORT        Stream HUD PWM values to a Servo2040 serial port.\n");
    std::printf("  --servo2040-port PORT   Same as --servo2040.\n");
    std::printf("  --pwm-control-servo2040 PORT\n");
    std::printf("                          Directly edit PWM values and stream them to Servo2040.\n");
    std::printf("  --servo2040-pwm-sim     Render the simulated robot from the Servo2040 PWM packet.\n");
    std::printf("  --pwm-sim               Same as --servo2040-pwm-sim.\n");
    std::printf("  --pwm-control           Skip gait/IK and directly edit simulated PWM values.\n");
    std::printf("  --port PORT             Serve the Wi-Fi controller page on this port (default 8080).\n");
    std::printf("  --help                  Show this help.\n");
#ifdef PROTON_SERVER_HEADLESS
    std::printf("\nHeadless default: Servo2040 autodiscovery and Proton client mDNS discovery are enabled.\n");
#endif
}

AppOptions parse_options(int argc, char** argv)
{
    AppOptions options;
#ifdef PROTON_SERVER_HEADLESS
    options.servo2040_autodiscover = true;
#endif
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            options.show_help = true;
        } else if (arg == "--config") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s requires a config file path.\n", arg.c_str());
                options.show_help = true;
                options.parse_error = true;
                break;
            }
            options.config_path = argv[++i];
            options.config_path_explicit = true;
        } else if (arg == "--validate-config") {
            options.validate_config_only = true;
        } else if (arg == "--auto-servo2040") {
            enable_servo2040_auto(options);
        } else if (arg == "--no-servo2040" || arg == "--no-servo2040-autodiscover") {
            disable_servo2040(options);
        } else if (arg == "--servo2040" || arg == "--servo2040-port"
                   || arg == "--pwm-control-servo2040") {
            if (arg == "--pwm-control-servo2040") {
                options.direct_pwm_control_enabled = true;
            }
            if (!has_value(argc, argv, i)) {
                enable_servo2040_auto(options);
                continue;
            }
            std::string value = argv[++i];
            if (is_auto_value(value)) {
                enable_servo2040_auto(options);
            } else {
                options.servo2040_enabled = true;
                options.servo2040_autodiscover = false;
                options.servo2040_port = value;
            }
        } else if (arg == "--servo2040-pwm-sim" || arg == "--pwm-sim") {
            options.servo2040_pwm_sim_enabled = true;
        } else if (arg == "--pwm-control") {
            options.direct_pwm_control_enabled = true;
        } else if (arg == "--port") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s requires a port number.\n", arg.c_str());
                options.show_help = true;
                options.parse_error = true;
                break;
            }
            options.wifi_controller_port = std::stoi(argv[++i]);
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", arg.c_str());
            options.show_help = true;
            options.parse_error = true;
            break;
        }
    }
    return options;
}
