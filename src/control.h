#pragma once

#include "input.h"
#include "options.h"
#include "robot_params.h"
#include "servo.h"
#include "types.h"

#include <array>

struct KeyStatus {
    bool kW = false;
    bool kS = false;
    bool kA = false;
    bool kD = false;
    bool kQ = false;
    bool kE = false;
    bool kR = false;
    bool kF = false;
};

struct ControlFlags {
    bool wifi_enabled = false;
};

enum class StartupPhase {
    PLACE_LOW_READY = 0,
    SWING_FRONT_LEFT_BACK_RIGHT_FROM_LOW_HOME = 1,
    SWING_BACK_LEFT_FRONT_RIGHT_FROM_LOW_HOME = 2,
    SWING_MIDDLE_TO_IDLE = 3,
    LIFT_BODY = 4,
    DONE = 5
};

enum class ShutdownPhase {
    SETTLE_TO_IDLE = 0,
    LOWER_BODY = 1,
    SWING_MIDDLE_TO_LOW_HOME = 2,
    SWING_BACK_LEFT_FRONT_RIGHT_TO_LOW_HOME = 3,
    SWING_FRONT_LEFT_BACK_RIGHT_TO_LOW_HOME = 4,
    DONE = 5
};

struct RobotControlState {
    Command cmd;
    Command target_cmd;
    double desired_height = 0.0;
    StartupPhase startup_phase = StartupPhase::PLACE_LOW_READY;
    double startup_phase_time = 0.0;
    bool shutdown_requested = false;
    bool shutdown_exits = true;
    bool shutdown_lower_only = false;
    bool shutdown_initialized = false;
    bool shutdown_complete = false;
    ShutdownPhase shutdown_phase = ShutdownPhase::SETTLE_TO_IDLE;
    double shutdown_phase_time = 0.0;
    double shutdown_start_height = 0.0;
    BasePose shutdown_pose;
    Vec3 shutdown_start_feet[6] = {};
    Vec3 shutdown_idle_start_feet[6] = {};
    bool shutdown_idle_swing[6] = {};
    int shutdown_idle_group = 0;
    double current_walk_speed = 0.0;
    double current_strafe_speed = 0.0;
    double current_spin_rate = 0.0;
    double dance_phase = 0.0;
    double dance_front_back_phase = 0.0;
    double dance_side_phase = 0.0;
    double dance_circle_phase = 0.0;
    DanceMode active_dance = DanceMode::NONE;
    GaitType gait_type = GaitType::TRIPOD;
    std::array<PWMValues, 6> direct_pwm = {};
    int selected_pwm_leg = 0;
    int selected_pwm_joint = 0;
};

struct RobotFrameState {
    std::array<PWMValues, 6> current_pwm = {};
    RobotState render_state;
    std::array<PWMValues, 6> render_pwm = {};
};

RobotControlState make_robot_control_state();

bool update_robot_control(const AppOptions& options,
                          double dt,
                          const InputState& input,
                          const ControlFlags& flags,
                          const RobotParams& params,
                          RobotControlState& control,
                          GaitState& gait_state,
                          BasePose& base_pose,
                          BasePose& final_pose,
                          RobotState& robot_state,
                          Vec3 feet_world[6],
                          KeyStatus& keys,
                          RobotFrameState& frame,
                          bool shutdown_requested = false);
