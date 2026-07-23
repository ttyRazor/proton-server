#include "control.h"

#include "config.h"
#include "gaits.h"
#include "kinematics.h"

#include <algorithm>
#include <cmath>

namespace {

double low_body_height()
{
    return config::SitBodyHeight;
}

std::array<PWMValues, 6> neutral_pwm_values()
{
    std::array<PWMValues, 6> values = {};
    for (PWMValues& pwm : values) {
        pwm = {config::PwmNeutral, config::PwmNeutral, config::PwmNeutral};
    }
    return values;
}

double step_height_for_body_height(double body_height)
{
    const auto& motion = config::Motion;
    constexpr double MAX_HEIGHT_STEP_GAIN = 0.75;

    double rise_span = std::max(1e-6, motion.height_max - motion.start_height);
    double rise_ratio = std::clamp((body_height - motion.start_height) / rise_span, 0.0, 1.0);
    return motion.step_height * (1.0 + MAX_HEIGHT_STEP_GAIN * rise_ratio);
}

GaitType gait_type_from_wifi_id(int gait)
{
    switch (gait) {
        case 1: return GaitType::TRIPOD;
        case 2: return GaitType::RIPPLE_EXT;
        case 3: return GaitType::RIPPLE;
        case 4: return GaitType::AMBLE;
        default: return GaitType::TRIPOD;
    }
}

bool dance_speed_active(double speed)
{
    return std::fabs(speed) > 0.05;
}

double smoothstep(double t)
{
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double lerp(double a, double b, double t)
{
    return a + (b - a) * std::clamp(t, 0.0, 1.0);
}

bool is_front_left_back_right_group(int leg)
{
    return leg == 3 || leg == 2;
}

bool is_back_left_front_right_group(int leg)
{
    return leg == 5 || leg == 0;
}

bool is_middle_leg(int leg)
{
    return leg == 1 || leg == 4;
}

int idle_settle_group_count(GaitType gait_type)
{
    switch (gait_type) {
        case GaitType::TRIPOD: return 2;
        case GaitType::RIPPLE: return 6;
        case GaitType::AMBLE: return 6;
        case GaitType::RIPPLE_EXT: return 6;
    }
    return 1;
}

bool leg_in_idle_settle_group(GaitType gait_type, int group, int leg)
{
    static constexpr int TripodGroups[2][3] = {
        {0, 2, 4},
        {1, 3, 5}
    };
    static constexpr int RippleSequence[6] = {0, 4, 2, 3, 1, 5};

    switch (gait_type) {
        case GaitType::TRIPOD:
            if (group < 0 || group >= 2) return false;
            for (int i = 0; i < 3; i++) {
                if (TripodGroups[group][i] == leg) return true;
            }
            return false;
        case GaitType::RIPPLE:
            return group >= 0 && group < 6 && RippleSequence[group] == leg;
        case GaitType::AMBLE:
        case GaitType::RIPPLE_EXT:
            return group >= 0 && group < 6 && RippleSequence[group] == leg;
    }
    return false;
}

double folded_horizontal_reach(const RobotParams& params, double body_height)
{
    constexpr double D2R = M_PI / 180.0;
    constexpr double TIBIA_FOLD_MARGIN_DEG = 4.0;

    double safe_tibia = (config::SafeTibiaFoldedDeg + TIBIA_FOLD_MARGIN_DEG) * D2R;
    double safe_radius = radius_for_tibia_angle(params.dims, safe_tibia);
    double leg_z = -body_height;
    double l_coxa = std::sqrt(std::max(0.0, safe_radius * safe_radius - leg_z * leg_z));
    return params.dims.coxa_len + l_coxa;
}

Vec3 low_home_foot_position(const RobotParams& params, int leg, double body_height)
{
    double mount_angle = params.mount_angles[leg];
    double mount_radius = params.mount_radii[leg];
    double pivot_x = mount_radius * std::cos(mount_angle);
    double pivot_y = mount_radius * std::sin(mount_angle);
    double reach = folded_horizontal_reach(params, body_height);
    double side_angle = (leg < 3) ? -M_PI * 0.5 : M_PI * 0.5;

    return {
        pivot_x + reach * std::cos(side_angle),
        pivot_y + reach * std::sin(side_angle),
        0.0
    };
}

Vec3 lerp(const Vec3& a, const Vec3& b, double t)
{
    return {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

Vec3 body_planar_to_world(const BasePose& pose, const Vec3& body_point)
{
    double cy = std::cos(pose.yaw);
    double sy = std::sin(pose.yaw);
    return {
        pose.x + body_point.x * cy - body_point.y * sy,
        pose.y + body_point.x * sy + body_point.y * cy,
        body_point.z
    };
}

Vec3 idle_foot_position(const RobotParams& params, int leg, const BasePose& pose)
{
    return body_planar_to_world(pose, params.default_foot_positions[leg]);
}

Vec3 low_home_foot_position(const RobotParams& params, int leg, const BasePose& pose)
{
    return body_planar_to_world(pose, low_home_foot_position(params, leg, pose.z));
}

void reset_gait_feet(GaitState& gait_state, const RobotParams& params, const BasePose& pose)
{
    gait_state.initialized = true;
    gait_state.pose_x = pose.x;
    gait_state.pose_y = pose.y;
    gait_state.pose_yaw = pose.yaw;
    gait_state.smooth_vx = gait_state.smooth_vy = gait_state.smooth_wz = 0.0;
    gait_state.sway_roll = gait_state.sway_pitch = 0.0;
    gait_state.prev_vx_w = gait_state.prev_vy_w = 0.0;
    for (int i = 0; i < 6; i++) {
        gait_state.is_swing[i] = false;
        gait_state.swing_start[i] = {};
        gait_state.feet_world[i] = idle_foot_position(params, i, pose);
        gait_state.drag_distance[i] = 0.0;
    }
}

void reset_shutdown_sequence(RobotControlState& control)
{
    control.shutdown_requested = false;
    control.shutdown_exits = true;
    control.shutdown_lower_only = false;
    control.shutdown_initialized = false;
    control.shutdown_complete = false;
    control.shutdown_phase = ShutdownPhase::SETTLE_TO_IDLE;
    control.shutdown_phase_time = 0.0;
    control.shutdown_idle_group = 0;
}

void set_command_height(RobotControlState& control, double height, bool reset_body_radius = true)
{
    const auto& motion = config::Motion;

    control.desired_height = height;
    control.cmd.pose.x = control.cmd.pose.y = 0.0;
    control.cmd.pose.z = height;
    control.cmd.pose.roll = control.cmd.pose.pitch = control.cmd.pose.yaw = 0.0;
    control.cmd.twist.linear_x = control.cmd.twist.linear_y = control.cmd.twist.angular_z = 0.0;
    if (reset_body_radius) {
    control.cmd.body_radius = motion.body_radius;
    }
    control.cmd.gait.cycle_time = motion.cycle_time;
    control.cmd.gait.step_height = step_height_for_body_height(height);
    control.cmd.gait.stride_scale = 1.0;
    control.target_cmd = control.cmd;
}

void solve_static_pose(const RobotParams& params,
                       RobotControlState& control,
                       GaitState& gait_state,
                       BasePose& base_pose,
                       BasePose& final_pose,
                       RobotState& robot_state,
                       Vec3 feet_world[6],
                       RobotFrameState& frame,
                       const BasePose& target_pose)
{
    set_command_height(control, target_pose.z, false);
    base_pose = target_pose;
    final_pose = base_pose;

    hexapod_ik_solver(base_pose, 0.0, params, feet_world, robot_state, final_pose);
    frame.current_pwm = pwm_from_robot_state(robot_state);
    frame.render_state = robot_state;
    frame.render_pwm = frame.current_pwm;

    (void)gait_state;
}

bool update_shutdown_sequence(double dt,
                              const RobotParams& params,
                              RobotControlState& control,
                              GaitState& gait_state,
                              BasePose& base_pose,
                              BasePose& final_pose,
                              RobotState& robot_state,
                              Vec3 feet_world[6],
                              RobotFrameState& frame)
{
    constexpr double IDLE_SETTLE_TIME = 1.25;
    constexpr double LOWER_TIME = 1.20;
    constexpr double SWING_TIME = 1.25;
    constexpr double SHUTDOWN_STEP_HEIGHT = 0.035;
    constexpr double IDLE_POSITION_TOLERANCE = 0.004;

    if (!control.shutdown_requested) return false;
    if (control.shutdown_complete) {
        control.shutdown_phase = ShutdownPhase::DONE;
    }

    if (!control.shutdown_initialized && !control.shutdown_complete) {
        control.shutdown_initialized = true;
        control.shutdown_phase = ShutdownPhase::SETTLE_TO_IDLE;
        control.shutdown_phase_time = 0.0;
        control.shutdown_idle_group = 0;
        control.shutdown_start_height = std::max(0.0, final_pose.z);
        control.shutdown_pose = final_pose;
        control.shutdown_pose.roll = 0.0;
        control.shutdown_pose.pitch = 0.0;
        control.shutdown_pose.z = control.shutdown_start_height;

        bool needs_idle_settle = false;
        for (int i = 0; i < 6; i++) {
            control.shutdown_start_feet[i] = feet_world[i];
            control.shutdown_start_feet[i].z = 0.0;
            control.shutdown_idle_start_feet[i] = control.shutdown_start_feet[i];

            if (control.shutdown_lower_only) continue;

            Vec3 idle = idle_foot_position(params, i, control.shutdown_pose);
            control.shutdown_idle_swing[i] =
                (control.shutdown_start_feet[i] - idle).norm() > IDLE_POSITION_TOLERANCE;
            needs_idle_settle = needs_idle_settle || control.shutdown_idle_swing[i];
        }
        if (control.shutdown_lower_only || !needs_idle_settle) {
            control.shutdown_phase = ShutdownPhase::LOWER_BODY;
        }
        if (!control.shutdown_lower_only) {
        for (int i = 0; i < 6; i++) {
            control.shutdown_start_feet[i] = idle_foot_position(params, i, control.shutdown_pose);
            }
        }
    }

    if (!control.shutdown_complete) {
        control.shutdown_phase_time += dt;
    }
    control.startup_phase = StartupPhase::DONE;
    control.active_dance = DanceMode::NONE;

    for (int i = 0; i < 6; i++) {
        gait_state.drag_distance[i] = 0.0;
    }

    double body_height = low_body_height();
    switch (control.shutdown_phase) {
        case ShutdownPhase::SETTLE_TO_IDLE: {
            int group_count = idle_settle_group_count(control.gait_type);
            while (control.shutdown_idle_group < group_count) {
                bool group_has_work = false;
                for (int i = 0; i < 6; i++) {
                    group_has_work = group_has_work
                                  || (control.shutdown_idle_swing[i]
                                      && leg_in_idle_settle_group(control.gait_type,
                                                                  control.shutdown_idle_group,
                                                                  i));
                }
                if (group_has_work) break;
                control.shutdown_idle_group++;
                control.shutdown_phase_time = 0.0;
            }

            if (control.shutdown_idle_group >= group_count) {
                control.shutdown_phase = ShutdownPhase::LOWER_BODY;
                control.shutdown_phase_time = 0.0;
                for (int i = 0; i < 6; i++) {
                    gait_state.is_swing[i] = false;
                    feet_world[i] = idle_foot_position(params, i, control.shutdown_pose);
                    control.shutdown_start_feet[i] = feet_world[i];
                }
                break;
            }

            double t = smoothstep(control.shutdown_phase_time / IDLE_SETTLE_TIME);
            body_height = control.shutdown_start_height;
            for (int i = 0; i < 6; i++) {
                Vec3 idle = idle_foot_position(params, i, control.shutdown_pose);
                bool active = control.shutdown_idle_swing[i]
                           && leg_in_idle_settle_group(control.gait_type,
                                                       control.shutdown_idle_group,
                                                       i);
                Vec3 foot = idle;
                if (active) {
                    foot = lerp(control.shutdown_idle_start_feet[i], idle, t);
                    foot.z = std::sin(t * M_PI) * SHUTDOWN_STEP_HEIGHT;
                } else if (control.shutdown_idle_swing[i]) {
                    foot = control.shutdown_idle_start_feet[i];
                }
                gait_state.is_swing[i] = active;
                feet_world[i] = foot;
            }
            if (control.shutdown_phase_time >= IDLE_SETTLE_TIME) {
                for (int i = 0; i < 6; i++) {
                    if (control.shutdown_idle_swing[i]
                        && leg_in_idle_settle_group(control.gait_type,
                                                    control.shutdown_idle_group,
                                                    i)) {
                        control.shutdown_idle_swing[i] = false;
                        control.shutdown_idle_start_feet[i] =
                            idle_foot_position(params, i, control.shutdown_pose);
                    }
                }
                control.shutdown_idle_group++;
                control.shutdown_phase_time = 0.0;

                if (control.shutdown_idle_group >= group_count) {
                    control.shutdown_phase = ShutdownPhase::LOWER_BODY;
                    for (int i = 0; i < 6; i++) {
                        gait_state.is_swing[i] = false;
                        feet_world[i] = idle_foot_position(params, i, control.shutdown_pose);
                        control.shutdown_start_feet[i] = feet_world[i];
                    }
                }
            }
            break;
        }
        case ShutdownPhase::LOWER_BODY: {
            double t = smoothstep(control.shutdown_phase_time / LOWER_TIME);
            body_height = control.shutdown_start_height
                        + (low_body_height() - control.shutdown_start_height) * t;
            for (int i = 0; i < 6; i++) {
                feet_world[i] = {control.shutdown_start_feet[i].x,
                                 control.shutdown_start_feet[i].y,
                                 0.0};
                gait_state.is_swing[i] = false;
            }
            if (control.shutdown_phase_time >= LOWER_TIME) {
                control.shutdown_phase_time = 0.0;
                body_height = low_body_height();
                if (control.shutdown_lower_only) {
                    control.shutdown_phase = ShutdownPhase::DONE;
                    control.shutdown_complete = true;
                    break;
                }
                control.shutdown_phase = ShutdownPhase::SWING_MIDDLE_TO_LOW_HOME;
            }
            break;
        }
        case ShutdownPhase::SWING_MIDDLE_TO_LOW_HOME: {
            double t = smoothstep(control.shutdown_phase_time / SWING_TIME);
            body_height = low_body_height();
            BasePose low_pose = control.shutdown_pose;
            low_pose.z = low_body_height();
            for (int i = 0; i < 6; i++) {
                Vec3 low_home = low_home_foot_position(params, i, low_pose);
                Vec3 foot = is_middle_leg(i)
                          ? lerp(control.shutdown_start_feet[i], low_home, t)
                          : control.shutdown_start_feet[i];
                bool swinging = is_middle_leg(i);
                if (swinging) {
                    foot.z = std::sin(t * M_PI) * SHUTDOWN_STEP_HEIGHT;
                }
                gait_state.is_swing[i] = swinging;
                feet_world[i] = foot;
            }
            if (control.shutdown_phase_time >= SWING_TIME) {
                control.shutdown_phase = ShutdownPhase::SWING_BACK_LEFT_FRONT_RIGHT_TO_LOW_HOME;
                control.shutdown_phase_time = 0.0;
            }
            break;
        }
        case ShutdownPhase::SWING_BACK_LEFT_FRONT_RIGHT_TO_LOW_HOME: {
            double t = smoothstep(control.shutdown_phase_time / SWING_TIME);
            body_height = low_body_height();
            BasePose low_pose = control.shutdown_pose;
            low_pose.z = low_body_height();
            for (int i = 0; i < 6; i++) {
                Vec3 low_home = low_home_foot_position(params, i, low_pose);
                Vec3 foot = is_back_left_front_right_group(i)
                          ? lerp(control.shutdown_start_feet[i], low_home, t)
                          : (is_front_left_back_right_group(i) ? control.shutdown_start_feet[i] : low_home);
                bool swinging = is_back_left_front_right_group(i);
                if (swinging) {
                    foot.z = std::sin(t * M_PI) * SHUTDOWN_STEP_HEIGHT;
                }
                gait_state.is_swing[i] = swinging;
                feet_world[i] = foot;
            }
            if (control.shutdown_phase_time >= SWING_TIME) {
                control.shutdown_phase = ShutdownPhase::SWING_FRONT_LEFT_BACK_RIGHT_TO_LOW_HOME;
                control.shutdown_phase_time = 0.0;
            }
            break;
        }
        case ShutdownPhase::SWING_FRONT_LEFT_BACK_RIGHT_TO_LOW_HOME: {
            double t = smoothstep(control.shutdown_phase_time / SWING_TIME);
            body_height = low_body_height();
            BasePose low_pose = control.shutdown_pose;
            low_pose.z = low_body_height();
            for (int i = 0; i < 6; i++) {
                Vec3 low_home = low_home_foot_position(params, i, low_pose);
                Vec3 foot = is_front_left_back_right_group(i)
                          ? lerp(control.shutdown_start_feet[i], low_home, t)
                          : low_home;
                bool swinging = is_front_left_back_right_group(i);
                if (swinging) {
                    foot.z = std::sin(t * M_PI) * SHUTDOWN_STEP_HEIGHT;
                }
                gait_state.is_swing[i] = swinging;
                feet_world[i] = foot;
            }
            if (control.shutdown_phase_time >= SWING_TIME) {
                control.shutdown_phase = ShutdownPhase::DONE;
                control.shutdown_complete = true;
                for (int i = 0; i < 6; i++) {
                    gait_state.is_swing[i] = false;
                    feet_world[i] = low_home_foot_position(params, i, low_pose);
                }
            }
            break;
        }
        case ShutdownPhase::DONE:
            body_height = low_body_height();
            control.shutdown_complete = true;
            {
                BasePose low_pose = control.shutdown_pose;
                low_pose.z = low_body_height();
                for (int i = 0; i < 6; i++) {
                    gait_state.is_swing[i] = false;
                    feet_world[i] = control.shutdown_lower_only
                                  ? control.shutdown_start_feet[i]
                                  : low_home_foot_position(params, i, low_pose);
                }
            }
            break;
    }

    BasePose target_pose = control.shutdown_pose;
    target_pose.z = body_height;
    for (int i = 0; i < 6; i++) {
        feet_world[i] = project_foot_to_safe_workspace(params, i, target_pose, feet_world[i]);
        gait_state.feet_world[i] = feet_world[i];
    }

    solve_static_pose(params, control, gait_state, base_pose, final_pose,
                      robot_state, feet_world, frame, target_pose);
    return true;
}

bool update_startup_sequence(double dt,
                             const RobotParams& params,
                             RobotControlState& control,
                             GaitState& gait_state,
                             BasePose& base_pose,
                             BasePose& final_pose,
                             RobotState& robot_state,
                             Vec3 feet_world[6],
                             RobotFrameState& frame)
{
    constexpr double LOW_HOME_HOLD_TIME = 0.35;
    constexpr double SWING_TIME = 1.25;
    constexpr double LIFT_TIME = 1.20;
    constexpr double STARTUP_STEP_HEIGHT = 0.035;

    if (control.startup_phase == StartupPhase::DONE) return false;

    const auto& motion = config::Motion;
    control.startup_phase_time += dt;
    control.active_dance = DanceMode::NONE;
    double body_height = low_body_height();
    BasePose startup_pose = control.shutdown_pose;
    startup_pose.roll = 0.0;
    startup_pose.pitch = 0.0;
    startup_pose.z = body_height;

    auto finish_startup = [&] {
        control.startup_phase = StartupPhase::DONE;
        control.startup_phase_time = 0.0;
        set_command_height(control, motion.start_height);
        gait_state = GaitState{};
        BasePose standing_pose = startup_pose;
        standing_pose.z = motion.start_height;
        reset_gait_feet(gait_state, params, standing_pose);
        gait_state.current_mode = control.gait_type;
    };

    for (int i = 0; i < 6; i++) {
        gait_state.drag_distance[i] = 0.0;
    }

    switch (control.startup_phase) {
        case StartupPhase::PLACE_LOW_READY:
            if (control.startup_phase_time >= LOW_HOME_HOLD_TIME) {
                control.startup_phase = StartupPhase::SWING_FRONT_LEFT_BACK_RIGHT_FROM_LOW_HOME;
                control.startup_phase_time = 0.0;
            }
            break;
        case StartupPhase::SWING_FRONT_LEFT_BACK_RIGHT_FROM_LOW_HOME:
            if (control.startup_phase_time >= SWING_TIME) {
                control.startup_phase = StartupPhase::SWING_BACK_LEFT_FRONT_RIGHT_FROM_LOW_HOME;
                control.startup_phase_time = 0.0;
            }
            break;
        case StartupPhase::SWING_BACK_LEFT_FRONT_RIGHT_FROM_LOW_HOME:
            if (control.startup_phase_time >= SWING_TIME) {
                control.startup_phase = StartupPhase::SWING_MIDDLE_TO_IDLE;
                control.startup_phase_time = 0.0;
            }
            break;
        case StartupPhase::SWING_MIDDLE_TO_IDLE:
            if (control.startup_phase_time >= SWING_TIME) {
                control.startup_phase = StartupPhase::LIFT_BODY;
                control.startup_phase_time = 0.0;
            }
            break;
        case StartupPhase::LIFT_BODY:
            body_height = low_body_height()
                        + (motion.start_height - low_body_height())
                        * smoothstep(control.startup_phase_time / LIFT_TIME);
            if (control.startup_phase_time >= LIFT_TIME) {
                body_height = motion.start_height;
                finish_startup();
            }
            break;
        case StartupPhase::DONE:
            return false;
    }

    set_command_height(control, body_height, false);
    startup_pose.z = body_height;
    base_pose = startup_pose;
    final_pose = base_pose;

    for (int i = 0; i < 6; i++) {
        BasePose low_pose = startup_pose;
        low_pose.z = low_body_height();
        Vec3 low_home = low_home_foot_position(params, i, low_pose);
        Vec3 idle = idle_foot_position(params, i, startup_pose);
        Vec3 foot = low_home;
        bool swinging = false;
        double swing_t = 0.0;

        if (control.startup_phase == StartupPhase::SWING_FRONT_LEFT_BACK_RIGHT_FROM_LOW_HOME) {
            swing_t = smoothstep(control.startup_phase_time / SWING_TIME);
            if (is_front_left_back_right_group(i)) {
                foot = lerp(low_home, idle, swing_t);
                swinging = true;
            }
        } else if (control.startup_phase == StartupPhase::SWING_BACK_LEFT_FRONT_RIGHT_FROM_LOW_HOME) {
            swing_t = smoothstep(control.startup_phase_time / SWING_TIME);
            if (is_back_left_front_right_group(i)) {
                foot = lerp(low_home, idle, swing_t);
                swinging = true;
            } else if (is_front_left_back_right_group(i)) {
                foot = idle;
            }
        } else if (control.startup_phase == StartupPhase::SWING_MIDDLE_TO_IDLE) {
            swing_t = smoothstep(control.startup_phase_time / SWING_TIME);
            if (is_middle_leg(i)) {
                foot = lerp(low_home, idle, swing_t);
                swinging = true;
            } else {
                foot = idle;
            }
        } else if (control.startup_phase == StartupPhase::LIFT_BODY
                   || control.startup_phase == StartupPhase::DONE) {
            foot = idle;
            swing_t = 1.0;
        }

        if (swinging) {
            foot.z = std::sin(swing_t * M_PI) * STARTUP_STEP_HEIGHT;
        }
        gait_state.is_swing[i] = swinging;
        gait_state.swing_start[i] = low_home;
        feet_world[i] = project_foot_to_safe_workspace(params, i, base_pose, foot);
        gait_state.feet_world[i] = feet_world[i];
    }

    hexapod_ik_solver(base_pose, 0.0, params, feet_world, robot_state, final_pose);
    frame.current_pwm = pwm_from_robot_state(robot_state);
    frame.render_state = robot_state;
    frame.render_pwm = frame.current_pwm;
    return true;
}

void update_direct_pwm_control(const AppOptions& options,
                               const InputState& input,
                               const ControlFlags& flags,
                               RobotControlState& control,
                               GaitState& gait_state,
                               BasePose& base_pose,
                               BasePose& final_pose,
                               RobotFrameState& frame)
{
    BasePose direct_pwm_pose;
    direct_pwm_pose.z = config::Motion.start_height;

    frame.current_pwm = control.direct_pwm;
    frame.render_pwm = frame.current_pwm;
    frame.render_state = robot_state_from_pwm(frame.render_pwm);
    final_pose = direct_pwm_pose;
    base_pose = direct_pwm_pose;
    control.active_dance = DanceMode::NONE;
    for (int i = 0; i < 6; i++) {
        gait_state.is_swing[i] = false;
        gait_state.drag_distance[i] = 0.0;
    }

    (void)options;
    (void)input;
    (void)flags;
    (void)control;
}

void update_gait_selection(const InputState& input,
                           const ControlFlags& flags,
                           const RobotParams& params,
                           RobotControlState& control,
                           GaitState& gait_state)
{
    GaitType new_gait = control.gait_type;
    if (flags.wifi_enabled && input.wifi_gait_control_active) {
        new_gait = gait_type_from_wifi_id(input.wifi_gait);
    }
    if (new_gait != control.gait_type) {
        control.gait_type = new_gait;
    }
    (void)params;
    (void)gait_state;
}

void update_locomotion_control(double dt,
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
                               RobotFrameState& frame)
{
    const auto& motion = config::Motion;

    if (input.wifi_body_radius_control_active) {
        control.target_cmd.body_radius =
            std::clamp(static_cast<double>(input.wifi_body_radius),
                       config::BodyRadiusMin,
                       config::BodyRadiusMax);
    }
    const bool wifi_step_height_requested = input.wifi_available;
    const double wifi_step_height = std::clamp(static_cast<double>(input.wifi_step_height),
                                               config::StepHeightMin,
                                               config::StepHeightMax);
    if (input.wifi_speed_control_active) {
        const double linear_speed =
            std::clamp(static_cast<double>(input.wifi_speed),
                       motion.linear_speed_min,
                       motion.linear_speed_max);
        const double speed_ratio =
            (linear_speed - motion.linear_speed_min)
            / std::max(1e-6, motion.linear_speed_max - motion.linear_speed_min);
        control.current_walk_speed = linear_speed;
        control.current_strafe_speed = linear_speed;
        control.current_spin_rate =
            lerp(motion.spin_rate_min, motion.spin_rate_max, speed_ratio);
    }

    if (update_shutdown_sequence(dt, params, control, gait_state, base_pose, final_pose,
                                 robot_state, feet_world, frame)) {
        return;
    }

    if (update_startup_sequence(dt, params, control, gait_state, base_pose, final_pose,
                                robot_state, feet_world, frame)) {
        return;
    }

    update_gait_selection(input, flags, params, control, gait_state);

    double spd = 1.0;
    double walk_input = 0.0;
    double strafe_input = 0.0;
    double turn_input = 0.0;
    double height_input = 0.0;
    double speed_adjust_input = 0.0;
    double front_back_dance_speed = 0.0;
    double side_dance_speed = 0.0;
    double circle_dance_speed = 0.0;

    auto apply_axis_action = [&](WifiAxisAction action, double value) {
        switch (action) {
            case WifiAxisAction::MARCH:
                walk_input += value;
                break;
            case WifiAxisAction::STRAFE:
                strafe_input += value;
                break;
            case WifiAxisAction::TURN:
                turn_input += value;
                break;
            case WifiAxisAction::HEIGHT:
                height_input += value;
                break;
            case WifiAxisAction::FRONT_BACK:
                front_back_dance_speed += value;
                break;
            case WifiAxisAction::SIDE_SIDE:
                side_dance_speed += value;
                break;
            case WifiAxisAction::CIRCLE:
                circle_dance_speed += value;
                break;
            case WifiAxisAction::NONE:
            default:
                break;
        }
    };

    if (flags.wifi_enabled) {
        spd = 1.0;
        if (input.wifi_relay_status) {
            apply_axis_action(input.wifi_primary_x_action, -input.wifi_primary_x);
            apply_axis_action(input.wifi_primary_y_action, input.wifi_primary_y);
            apply_axis_action(input.wifi_secondary_x_action, -input.wifi_secondary_x);
            apply_axis_action(input.wifi_secondary_y_action, input.wifi_secondary_y);
        }
        if (input.wifi_height_control_active || input.wifi_position_control_active) {
            double target_height = std::clamp(static_cast<double>(input.wifi_target_height),
                                              low_body_height(),
                                              motion.height_max);
            if (input.wifi_height_control_gentle) {
                const double max_step = motion.height_rate * dt;
                if (control.desired_height < target_height) {
                    control.desired_height = std::min(control.desired_height + max_step, target_height);
                } else {
                    control.desired_height = std::max(control.desired_height - max_step, target_height);
                }
            } else {
                control.desired_height = target_height;
            }
        }
        keys.kW = walk_input > 0.05;
        keys.kS = walk_input < -0.05;
        keys.kA = strafe_input > 0.05;
        keys.kD = strafe_input < -0.05;
        keys.kQ = turn_input > 0.05;
        keys.kE = turn_input < -0.05;
    }

    control.current_walk_speed += speed_adjust_input * motion.linear_speed_adjust_rate * dt;
    control.current_strafe_speed += speed_adjust_input * motion.linear_speed_adjust_rate * dt;
    control.current_spin_rate += speed_adjust_input * motion.spin_rate_adjust_rate * dt;
    control.current_walk_speed = std::clamp(control.current_walk_speed, motion.linear_speed_min, motion.linear_speed_max);
    control.current_strafe_speed = std::clamp(control.current_strafe_speed, motion.linear_speed_min, motion.linear_speed_max);
    control.current_spin_rate = std::clamp(control.current_spin_rate, motion.spin_rate_min, motion.spin_rate_max);
    walk_input = std::clamp(walk_input, -1.0, 1.0);
    strafe_input = std::clamp(strafe_input, -1.0, 1.0);
    turn_input = std::clamp(turn_input, -1.0, 1.0);
    height_input = std::clamp(height_input, -1.0, 1.0);
    front_back_dance_speed = std::clamp(front_back_dance_speed, -1.0, 1.0);
    side_dance_speed = std::clamp(side_dance_speed, -1.0, 1.0);
    circle_dance_speed = std::clamp(circle_dance_speed, -1.0, 1.0);

    control.target_cmd.twist.linear_x  = control.current_walk_speed   * walk_input * spd;
    control.target_cmd.twist.linear_y  = control.current_strafe_speed * strafe_input * spd;
    control.target_cmd.twist.angular_z = control.current_spin_rate    * turn_input * spd;

    control.desired_height += motion.height_rate * dt * height_input * spd;
    control.desired_height = std::clamp(control.desired_height, low_body_height(), motion.height_max);
    control.target_cmd.pose.z = control.desired_height;
    control.target_cmd.gait.step_height = wifi_step_height_requested
                                        ? wifi_step_height
                                        : step_height_for_body_height(control.desired_height);

    control.target_cmd.pose.x = control.target_cmd.pose.y = 0.0;
    control.target_cmd.pose.roll = control.target_cmd.pose.pitch = control.target_cmd.pose.yaw = 0.0;
    control.active_dance = DanceMode::NONE;
    if (dance_speed_active(front_back_dance_speed)) {
        control.active_dance = DanceMode::FRONT_BACK;
    }
    if (dance_speed_active(side_dance_speed)) {
        control.active_dance = DanceMode::SIDE_SIDE;
    }
    if (dance_speed_active(circle_dance_speed)) {
        control.active_dance = DanceMode::CIRCLE;
    }
    if (control.active_dance != DanceMode::NONE) {
        auto advance_phase = [&](double& phase, double speed) {
            if (dance_speed_active(speed)) {
                phase = std::fmod(phase + dt * motion.dance_rate_hz * speed * 2.0 * M_PI,
                                  2.0 * M_PI);
                if (phase < 0.0) phase += 2.0 * M_PI;
            }
        };
        advance_phase(control.dance_front_back_phase, front_back_dance_speed);
        advance_phase(control.dance_side_phase, side_dance_speed);
        advance_phase(control.dance_circle_phase, circle_dance_speed);

        const double front_back_wave =
            std::sin(control.dance_front_back_phase) * std::fabs(front_back_dance_speed);
        const double side_wave =
            std::sin(control.dance_side_phase) * std::fabs(side_dance_speed);
        const double circle_amp = std::fabs(circle_dance_speed);

        double local_x = front_back_wave * motion.dance_shift_amount;
        double local_y = side_wave * motion.dance_shift_amount;
        double yaw = gait_state.pose_yaw + control.cmd.pose.yaw;
        double cy = std::cos(yaw);
        double sy = std::sin(yaw);
        control.target_cmd.pose.x = local_x*cy - local_y*sy;
        control.target_cmd.pose.y = local_x*sy + local_y*cy;
        control.target_cmd.pose.pitch =
            front_back_wave * motion.dance_sway_angle
            + std::cos(control.dance_circle_phase) * motion.dance_circle_angle * circle_amp;
        control.target_cmd.pose.roll =
            side_wave * motion.dance_sway_angle
            + std::sin(control.dance_circle_phase) * motion.dance_circle_angle * circle_amp;
    }

    double alpha = 1.0 - std::exp(-motion.smooth_rate * dt);
    auto smooth = [&](double& v, double tgt) { v += alpha * (tgt - v); };

    smooth(control.cmd.twist.linear_x,  control.target_cmd.twist.linear_x);
    smooth(control.cmd.twist.linear_y,  control.target_cmd.twist.linear_y);
    smooth(control.cmd.twist.angular_z, control.target_cmd.twist.angular_z);
    smooth(control.cmd.pose.x,          control.target_cmd.pose.x);
    smooth(control.cmd.pose.y,          control.target_cmd.pose.y);
    smooth(control.cmd.pose.z,          control.target_cmd.pose.z);
    smooth(control.cmd.pose.roll,       control.target_cmd.pose.roll);
    smooth(control.cmd.pose.pitch,      control.target_cmd.pose.pitch);
    smooth(control.cmd.pose.yaw,        control.target_cmd.pose.yaw);
    smooth(control.cmd.body_radius,     control.target_cmd.body_radius);
    smooth(control.cmd.gait.step_height, control.target_cmd.gait.step_height);
    smooth(control.cmd.gait.stride_scale, control.target_cmd.gait.stride_scale);

    hexapod_gait_engine(dt, control.cmd, control.gait_type,
                        gait_state, params, base_pose, feet_world);

    hexapod_ik_solver(base_pose, control.cmd.body_radius, params, feet_world, robot_state, final_pose);
    frame.current_pwm = pwm_from_robot_state(robot_state);
    frame.render_state = robot_state;
    frame.render_pwm = frame.current_pwm;
}

} // namespace

RobotControlState make_robot_control_state()
{
    const auto& motion = config::Motion;
    RobotControlState control;
    control.cmd.gait.cycle_time = motion.cycle_time;
    control.cmd.gait.step_height = motion.step_height;
    control.cmd.gait.stride_scale = 1.0;
    control.cmd.body_radius = motion.body_radius;
    control.cmd.pose.x = control.cmd.pose.y = 0.0;
    control.cmd.pose.z = low_body_height();
    control.cmd.pose.roll = control.cmd.pose.pitch = control.cmd.pose.yaw = 0.0;
    control.cmd.twist.linear_x = control.cmd.twist.linear_y = control.cmd.twist.angular_z = 0.0;
    control.target_cmd = control.cmd;
    control.desired_height = low_body_height();
    control.current_walk_speed = motion.walk_speed;
    control.current_strafe_speed = motion.strafe_speed;
    control.current_spin_rate = motion.spin_rate;
    control.direct_pwm = neutral_pwm_values();
    return control;
}

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
                          bool shutdown_requested)
{
    keys = {};
    frame = {};
    if (shutdown_requested) {
        control.shutdown_requested = true;
        control.shutdown_exits = true;
    }

    if (flags.wifi_enabled && input.wifi_position_control_active) {
        if (input.wifi_target_position <= 0) {
            if (!control.shutdown_requested) {
                reset_shutdown_sequence(control);
            }
            if (!control.shutdown_complete) {
                control.shutdown_requested = true;
                control.shutdown_exits = false;
                control.shutdown_lower_only = false;
            }
        } else if (control.shutdown_requested || control.shutdown_complete) {
            reset_shutdown_sequence(control);
            control.startup_phase = StartupPhase::PLACE_LOW_READY;
            control.startup_phase_time = 0.0;
        }
    }

    if (update_shutdown_sequence(dt, params, control, gait_state, base_pose, final_pose,
                                 robot_state, feet_world, frame)) {
        return control.shutdown_complete && control.shutdown_exits;
    }

    if (options.direct_pwm_control_enabled) {
        update_direct_pwm_control(options, input, flags, control, gait_state, base_pose, final_pose, frame);
    } else {
        update_locomotion_control(dt, input, flags, params, control, gait_state,
                                  base_pose, final_pose, robot_state, feet_world, keys, frame);
    }
    return control.shutdown_complete && control.shutdown_exits;
}
