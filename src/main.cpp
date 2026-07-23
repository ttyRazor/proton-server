// ============================================================
//  main.cpp — Hexapod Kinematic Simulator
//
//  The simulator loop lives here; Wi-Fi control, Servo2040,
//  options, and user-tunable configuration are split into modules.
// ============================================================
#include "config.h"
#include "control.h"
#include "input.h"
#include "kinematics.h"
#include "options.h"
#include "robot_params.h"
#include "servo.h"
#include "wifi_controller.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <thread>
#include <string>

namespace {

constexpr double WifiRelayInactivityTimeout = 15.0;

volatile std::sig_atomic_t headless_shutdown_signal = 0;

void handle_headless_shutdown_signal(int)
{
    headless_shutdown_signal = 1;
}

int wifi_position_from_control(const RobotControlState& control)
{
    if (control.shutdown_complete) return 0;
    if (control.startup_phase != StartupPhase::DONE) return 0;
    return 1;
}

int wifi_gait_id(GaitType gait)
{
    switch (gait) {
        case GaitType::TRIPOD: return 1;
        case GaitType::RIPPLE_EXT: return 2;
        case GaitType::RIPPLE: return 3;
        case GaitType::AMBLE: return 4;
    }
    return 1;
}

double wifi_speed_from_control(const RobotControlState& control)
{
    return std::clamp(control.current_walk_speed,
                      config::Motion.linear_speed_min,
                      config::Motion.linear_speed_max);
}

struct ServoTelemetry {
    double voltage_poll_time = config::Servo2040VoltagePollInterval, relay_active_time = 0.0;
    double voltage = 0.0, current = 0.0;
    int voltage_critical_samples = 0;
    bool voltage_valid = false, current_valid = false;
    bool voltage_warning = false, voltage_critical = false;

    void reset_voltage_guard()
    {
        voltage_critical_samples = 0;
        voltage_warning = false;
    }

    bool poll(double dt, Servo2040Client& servo2040, WifiControllerServer& wifi_controller)
    {
        if (!servo2040.is_connected() || voltage_critical) return false;

        if (servo2040.relay_enabled()) {
            relay_active_time += dt;
        } else {
            relay_active_time = 0.0;
            reset_voltage_guard();
        }

        voltage_poll_time += dt;
        if (voltage_poll_time < config::Servo2040VoltagePollInterval) return false;
        voltage_poll_time = 0.0;

        double reading = 0.0;
        if (servo2040.read_voltage(reading)) {
            voltage = reading;
            voltage_valid = true;
            wifi_controller.update_voltage(voltage);

            const bool guard_armed =
                servo2040.relay_enabled()
                && relay_active_time >= config::Servo2040VoltageStartupDelay;
            voltage_warning = guard_armed && voltage < config::Servo2040VoltageWarn;
            if (guard_armed && voltage < config::Servo2040VoltageCritical) {
                voltage_critical_samples++;
                if (voltage_critical_samples >= config::Servo2040VoltageCriticalSamples) {
                    voltage_critical = true;
                    voltage_warning = false;
                }
            } else {
                voltage_critical_samples = 0;
            }
        }

        if (!servo2040.read_current(reading)) return voltage_critical;
        current = reading;
        current_valid = true;
        wifi_controller.update_current(current);
        return voltage_critical;
    }
};

bool local_robot_activity(const KeyStatus& keys, const RobotControlState& control)
{
    return keys.kW || keys.kS || keys.kA || keys.kD
        || keys.kQ || keys.kE || keys.kR || keys.kF
        || control.active_dance != DanceMode::NONE
        || control.startup_phase != StartupPhase::DONE
        || control.shutdown_requested;
}

} // namespace

int main(int argc, char** argv)
{
    AppOptions options = parse_options(argc, argv);
    if (options.show_help) {
        print_usage(argv[0]);
        return options.parse_error ? 1 : 0;
    }

    std::signal(SIGINT, handle_headless_shutdown_signal);
    std::signal(SIGTERM, handle_headless_shutdown_signal);
    std::printf("Headless mode: local window and keyboard/gamepad input disabled.\n");

    config::LoadResult config_result =
        config::load_user_config(options.config_path, options.config_path_explicit);
    for (const std::string& warning : config_result.warnings) {
        std::fprintf(stderr, "Config warning: %s\n", warning.c_str());
    }
    if (!config_result.ok) {
        for (const std::string& error : config_result.errors) {
            std::fprintf(stderr, "Config error: %s\n", error.c_str());
        }
        return 1;
    }
    if (config_result.loaded) {
        std::printf("Robot config: %s\n", options.config_path.c_str());
    }
    if (options.validate_config_only) {
        std::printf("Config validation passed.\n");
        return 0;
    }

    WifiControllerServer wifi_controller;
    if (wifi_controller.start(options.wifi_controller_port)) {
        std::printf("Wi-Fi controller: http://0.0.0.0:%d/\n", options.wifi_controller_port);
    } else {
        std::fprintf(stderr, "Wi-Fi controller: %s\n", wifi_controller.status().c_str());
    }

    // ---- Robot initialisation ----------------------------------
    RobotParams params = get_robot_params();

    Servo2040Client servo2040;
    if (options.servo2040_autodiscover && options.servo2040_port.empty()) {
        options.servo2040_port = discover_servo2040_port();
        if (!options.servo2040_port.empty()) {
            options.servo2040_enabled = true;
            std::printf("Servo2040: discovered %s\n", options.servo2040_port.c_str());
        } else {
            options.servo2040_enabled = false;
            std::fprintf(stderr, "Servo2040: autodiscovery found no Pico/RP2040 serial port; continuing without hardware output.\n");
        }
    }
    if (options.servo2040_enabled) {
        bool ok = servo2040.open(options.servo2040_port);
        if (!ok) {
            std::fprintf(stderr, "Servo2040: failed to open %s (%s)\n",
                         options.servo2040_port.c_str(), servo2040.status().c_str());
            wifi_controller.update_relay_status(false);
        } else {
            std::array<int, 18> neutral_pwm_by_pin = {};
            neutral_pwm_by_pin.fill(config::PwmNeutral);
            servo2040.send_pwm_values(neutral_pwm_by_pin);
            wifi_controller.update_relay_status(servo2040.relay_enabled());
        }
    }

    // ---- Gait state --------------------------------------------
    RobotControlState control = make_robot_control_state();
    GaitState gait_state;          // zero-init; gaits self-initialize
    BasePose base_pose, final_pose;
    RobotState robot_state;
    Vec3 feet_world[6];

    // Pre-seed feet at default positions.
    for (int i = 0; i < 6; i++)
        gait_state.feet_world[i] = params.default_foot_positions[i];

    // ============================================================
    //  Main loop
    // ============================================================
    bool shutdown_requested = false;
    bool shutdown_complete = false;
    ServoTelemetry telemetry;
    double relay_inactivity_time = 0.0;
    ControlInputState control_input;

    auto set_wifi_requested_relay = [&]() {
        WifiControllerSnapshot relay_snapshot = wifi_controller.snapshot();
        if (!relay_snapshot.relay_control_active) return;
        bool relay_hardware_target = relay_snapshot.target_relay_status;
        bool complete_relay_request = true;
        if (!relay_snapshot.target_relay_status && !relay_snapshot.relay_switch_ready) {
            if (relay_snapshot.relay_status) return;
            relay_hardware_target = true;
            complete_relay_request = false;
        }

        bool relay_set = true;
        if (options.servo2040_enabled) {
            relay_set = servo2040.is_connected()
                     && servo2040.set_relay(relay_hardware_target);
        }
        if (relay_set) {
            wifi_controller.update_relay_status(relay_hardware_target, complete_relay_request);
            telemetry.relay_active_time = 0.0;
            telemetry.reset_voltage_guard();
        }
    };

    using HeadlessClock = std::chrono::steady_clock;
    constexpr double TargetFrameSeconds = 1.0 / 60.0;
    const auto target_frame_duration =
        std::chrono::duration_cast<HeadlessClock::duration>(
            std::chrono::duration<double>(TargetFrameSeconds));
    auto last_frame_time = HeadlessClock::now() - target_frame_duration;

    while (!shutdown_complete) {
        if (headless_shutdown_signal != 0) {
            shutdown_requested = true;
        }

        const auto frame_start_time = HeadlessClock::now();
        double dt = std::min(
            std::chrono::duration<double>(frame_start_time - last_frame_time).count(),
            0.05);
        last_frame_time = frame_start_time;
        if (options.servo2040_enabled && servo2040.is_connected() && telemetry.poll(dt, servo2040, wifi_controller)) {
            shutdown_requested = true;
        }

        set_wifi_requested_relay();

        InputState input = read_input_state();
        apply_wifi_controller_snapshot(input, wifi_controller.snapshot());
        control_input = update_control_input_source(input, control_input);
        bool wifi_enabled = control_input.active_source == ControlInputSource::WIFI;

        KeyStatus keys;
        RobotFrameState frame;
        shutdown_complete = update_robot_control(options, dt, input, {wifi_enabled},
                                                 params, control, gait_state, base_pose, final_pose,
                                                 robot_state, feet_world, keys, frame,
                                                 shutdown_requested);
        wifi_controller.update_robot_status(control.cmd.pose.z,
                                            wifi_position_from_control(control),
                                            wifi_gait_id(control.gait_type));
        wifi_controller.update_motion_status(control.cmd.body_radius,
                                             control.cmd.gait.step_height,
                                             wifi_speed_from_control(control));
        wifi_controller.update_shutdown_complete(control.shutdown_complete);
        WifiControllerSnapshot activity_snapshot = wifi_controller.snapshot();
        const bool relay_can_timeout =
            activity_snapshot.relay_status
            && activity_snapshot.target_relay_status
            && !activity_snapshot.relay_control_active;
        if (relay_can_timeout && !activity_snapshot.active && !local_robot_activity(keys, control)) {
            relay_inactivity_time += dt;
            if (relay_inactivity_time >= WifiRelayInactivityTimeout) {
                wifi_controller.request_relay_status(false, false);
                relay_inactivity_time = 0.0;
            }
        } else {
            relay_inactivity_time = 0.0;
        }
        set_wifi_requested_relay();

        std::array<int, 18> servo2040_pin_pwm = pwm_by_servo2040_pin_unflipped(frame.current_pwm);

        if (!options.direct_pwm_control_enabled && options.servo2040_pwm_sim_enabled) {
            frame.render_pwm = pwm_by_leg_from_servo2040_pin(servo2040_pin_pwm);
            frame.render_state = robot_state_from_pwm(frame.render_pwm);
        }

        if (options.servo2040_enabled) {
            if (shutdown_complete) {
                servo2040.close();
                wifi_controller.update_relay_status(servo2040.relay_enabled());
            } else if (servo2040.is_connected()) {
                servo2040.send_pwm_values(pwm_by_servo2040_pin_for_hardware(frame.current_pwm));
            }
        }

        // ---- FK + compute max IK error -------------------------
        LegPoints fk_pts[6];
        double max_err = 0.0;
        double max_drag = 0.0;
        for (int i = 0; i < 6; i++) {
            fk_pts[i] = compute_leg_fk(params, final_pose, frame.render_state.legs[i], i);
            if (options.direct_pwm_control_enabled) {
                feet_world[i] = fk_pts[i].pts[4];
            }
            double err = options.direct_pwm_control_enabled ? 0.0 : (fk_pts[i].pts[4] - feet_world[i]).norm();
            if (err > max_err) max_err = err;
            if (gait_state.drag_distance[i] > max_drag) max_drag = gait_state.drag_distance[i];
        }
        wifi_controller.update_visualizer_frame(final_pose, fk_pts, feet_world,
                                                gait_state.is_swing, frame.render_state,
                                                frame.render_pwm);
        std::this_thread::sleep_until(frame_start_time + target_frame_duration);
    }

    wifi_controller.stop();
    return 0;
}
