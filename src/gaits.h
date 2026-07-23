#pragma once
// ============================================================
//  gaits.h — gait phase tables and foot trajectory engine
// ============================================================
#include "config.h"
#include "types.h"
#include "robot_params.h"
#include "kinematics.h"

#include <algorithm>
#include <array>
#include <cmath>

struct PhaseWindow {
    double start = 0.0;
    double end = 0.0;
};

static void get_phase_table(GaitType mode, std::array<PhaseWindow, 6>& table, double& swing_frac)
{
    switch (mode) {
        case GaitType::TRIPOD:
            swing_frac = 0.5;
            table = {{{0.0, 0.5}, {0.5, 1.0}, {0.0, 0.5},
                      {0.5, 1.0}, {0.0, 0.5}, {0.5, 1.0}}};
            break;
        case GaitType::RIPPLE:
            swing_frac = 4.0 / 12.0;
            table = {{{0.0/12.0, 4.0/12.0}, {8.0/12.0, 12.0/12.0},
                      {4.0/12.0, 8.0/12.0}, {4.0/12.0, 8.0/12.0},
                      {0.0/12.0, 4.0/12.0}, {8.0/12.0, 12.0/12.0}}};
            break;
        case GaitType::AMBLE:
            swing_frac = 0.25;
            table = {{{0.0/12.0, 3.0/12.0}, {8.0/12.0, 11.0/12.0},
                      {4.0/12.0, 7.0/12.0}, {6.0/12.0, 9.0/12.0},
                      {2.0/12.0, 5.0/12.0}, {10.0/12.0, 13.0/12.0}}};
            break;
        case GaitType::RIPPLE_EXT:
            swing_frac = 5.0 / 12.0;
            table = {{{0.0/12.0, 5.0/12.0}, {7.0/12.0, 12.0/12.0},
                      {2.0/12.0, 7.0/12.0}, {6.0/12.0, 11.0/12.0},
                      {1.0/12.0, 6.0/12.0}, {8.0/12.0, 13.0/12.0}}};
            break;
    }
}

static double swing_progress(double phase, double start, double end, double margin)
{
    start = std::fmod(start + margin, 1.0);
    end = std::fmod(end - margin, 1.0);
    if (start < 0.0) start += 1.0;
    if (end < 0.0) end += 1.0;

    if (start < end) {
        if (phase >= start && phase <= end) return (phase - start) / (end - start);
        return -1.0;
    }

    if (phase >= start || phase <= end) {
        double span = (1.0 - start) + end;
        if (phase < start) return (phase + 1.0 - start) / span;
        return (phase - start) / span;
    }
    return -1.0;
}

static void compute_neutral_foot_pos(int leg,
                                     const RobotParams& params,
                                     double body_radius,
                                     double& base_x,
                                     double& base_y)
{
    double middle_mount_radius = 0.5 * (params.mount_radii[1] + params.mount_radii[4]);
    double reach_from_pivot = std::max(0.0, body_radius - middle_mount_radius);

    double ma = params.mount_angles[leg];
    double pivot_x = params.mount_radii[leg] * std::cos(ma);
    double pivot_y = params.mount_radii[leg] * std::sin(ma);
    double angle = std::atan2(params.default_foot_positions[leg].y - pivot_y,
                              params.default_foot_positions[leg].x - pivot_x);
    base_x = pivot_x + reach_from_pivot * std::cos(angle);
    base_y = pivot_y + reach_from_pivot * std::sin(angle);
}

static Vec3 clamp_to_workspace(Vec3 target,
                               int leg,
                               const BasePose& base_pose,
                               const RobotParams& params,
                               const config::GaitEngineConfig& gait_params)
{
    double ma = params.mount_angles[leg];
    double fa = params.coxa_frame_angles[leg];
    double final_yaw = base_pose.yaw;

    double cx_body = params.mount_radii[leg] * std::cos(ma);
    double cy_body = params.mount_radii[leg] * std::sin(ma);

    double px = target.x - base_pose.x;
    double py = target.y - base_pose.y;
    double cy_rot = std::cos(-final_yaw);
    double sy_rot = std::sin(-final_yaw);
    double px_b = px * cy_rot - py * sy_rot;
    double py_b = px * sy_rot + py * cy_rot;

    double sh_inv = std::sin(-fa);
    double ch_inv = std::cos(-fa);
    double xl = (px_b - cx_body) * ch_inv - (py_b - cy_body) * sh_inv;
    double yl = (px_b - cx_body) * sh_inv + (py_b - cy_body) * ch_inv;
    double zl = -base_pose.z;

    double coxa_lim = gait_params.coxa_limit_deg * (M_PI / 180.0);
    double h = std::sqrt(xl*xl + yl*yl);
    if (h > 1e-6) {
        double ca = std::clamp(std::atan2(yl, xl), -coxa_lim, coxa_lim);
        xl = h * std::cos(ca);
        yl = h * std::sin(ca);
    }

    double ft_reach = (params.dims.femur_len + params.dims.tibia_len)
                    * gait_params.femur_reach_margin;
    double ik_max = params.dims.coxa_len
                  + std::sqrt(std::max(0.0, ft_reach*ft_reach - zl*zl));
    double ik_min = params.dims.coxa_len + 0.01;

    h = std::sqrt(xl*xl + yl*yl);
    if (h > 1e-6) {
        double hc = std::clamp(h, ik_min, ik_max);
        xl *= hc / h;
        yl *= hc / h;
    }

    double tx_b = (xl * std::cos(fa) - yl * std::sin(fa)) + cx_body;
    double ty_b = (xl * std::sin(fa) + yl * std::cos(fa)) + cy_body;

    double cfy = std::cos(final_yaw);
    double sfy = std::sin(final_yaw);
    return {
        base_pose.x + tx_b * cfy - ty_b * sfy,
        base_pose.y + tx_b * sfy + ty_b * cfy,
        0.0
    };
}

static Vec3 swing_trajectory(const Vec3& from, const Vec3& to, double t, double height)
{
    double te = std::sin(t * M_PI * 0.5);
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    double lift_frac = std::min(0.5, height / (2.0 * height + dist + 1e-6));

    Vec3 plateau;
    if (te < lift_frac) {
        double u = te / std::max(lift_frac, 1e-6);
        plateau = {from.x, from.y, from.z + height * u};
    } else if (te < 1.0 - lift_frac) {
        double u = (te - lift_frac) / std::max(1.0 - 2.0 * lift_frac, 1e-6);
        plateau = {from.x + dx * u, from.y + dy * u, from.z + height};
    } else {
        double u = (te - (1.0 - lift_frac)) / std::max(lift_frac, 1e-6);
        plateau = {to.x, to.y, from.z + height * (1.0 - u)};
    }

    Vec3 arc = {from.x + dx * te, from.y + dy * te, from.z + std::sin(M_PI * te) * height};
    return {(plateau.x + arc.x) * 0.5,
            (plateau.y + arc.y) * 0.5,
            (plateau.z + arc.z) * 0.5};
}

inline void hexapod_gait_engine(double dt,
                                const Command& cmd,
                                GaitType requested_mode,
                                GaitState& gs,
                                const RobotParams& params,
                                BasePose& base_pose,
                                Vec3 feet_world[6])
{
    constexpr double REALIGNMENT_THRESHOLD = 0.018;
    constexpr double REALIGNMENT_FAST_CYCLE = 1.4;
    constexpr double MOVING_SPEED_THRESHOLD = 0.003;
    constexpr double EPSILON = 1e-6;
    const auto& gait_params = config::GaitEngine;

    if (!gs.initialized) {
        gs.initialized = true;
        gs.master_phase = 0.0;
        gs.current_mode = requested_mode;
        gs.pose_x = gs.pose_y = gs.pose_yaw = 0.0;
        gs.smooth_vx = gs.smooth_vy = gs.smooth_wz = 0.0;
        gs.sway_roll = gs.sway_pitch = 0.0;
        gs.prev_vx_w = gs.prev_vy_w = 0.0;
        for (int i = 0; i < 6; i++) {
            gs.is_swing[i] = false;
            gs.swing_start[i] = {};
            gs.feet_world[i] = params.default_foot_positions[i];
            gs.drag_distance[i] = 0.0;
        }
    }

    double vx_cmd = std::clamp(cmd.twist.linear_x,
                               -gait_params.max_linear_speed,
                                gait_params.max_linear_speed);
    double vy_cmd = std::clamp(cmd.twist.linear_y,
                               -gait_params.max_linear_speed,
                                gait_params.max_linear_speed);
    double wz_cmd = std::clamp(cmd.twist.angular_z,
                               -gait_params.max_angular_speed,
                                gait_params.max_angular_speed);

    double alpha = std::min(1.0, gait_params.cmd_smooth_hz * dt);
    gs.smooth_vx += (vx_cmd - gs.smooth_vx) * alpha;
    gs.smooth_vy += (vy_cmd - gs.smooth_vy) * alpha;
    gs.smooth_wz += (wz_cmd - gs.smooth_wz) * alpha;

    double yaw = gs.pose_yaw;
    double vx_w = gs.smooth_vx * std::cos(yaw) - gs.smooth_vy * std::sin(yaw);
    double vy_w = gs.smooth_vx * std::sin(yaw) + gs.smooth_vy * std::cos(yaw);
    double wz = gs.smooth_wz;

    double dt_safe = std::max(dt, 1e-4);
    double ax_w = (vx_w - gs.prev_vx_w) / dt_safe;
    double ay_w = (vy_w - gs.prev_vy_w) / dt_safe;
    gs.prev_vx_w = vx_w;
    gs.prev_vy_w = vy_w;

    double base_pos_x = gs.pose_x + cmd.pose.x;
    double base_pos_y = gs.pose_y + cmd.pose.y;
    double body_yaw = gs.pose_yaw + cmd.pose.yaw;

    double max_strain_sq = 0.0;
    for (int i = 0; i < 6; i++) {
        if (gs.is_swing[i]) continue;

        double base_x = 0.0;
        double base_y = 0.0;
        compute_neutral_foot_pos(i, params, cmd.body_radius, base_x, base_y);

        double ideal_x = base_pos_x + base_x * std::cos(body_yaw) - base_y * std::sin(body_yaw);
        double ideal_y = base_pos_y + base_x * std::sin(body_yaw) + base_y * std::cos(body_yaw);
        double dx = gs.feet_world[i].x - ideal_x;
        double dy = gs.feet_world[i].y - ideal_y;
        max_strain_sq = std::max(max_strain_sq, dx*dx + dy*dy);
        gs.drag_distance[i] = std::sqrt(dx*dx + dy*dy);
    }
    bool needs_realignment = max_strain_sq > REALIGNMENT_THRESHOLD * REALIGNMENT_THRESHOLD;

    std::array<PhaseWindow, 6> phase_table;
    double swing_frac = 0.5;
    get_phase_table(gs.current_mode, phase_table, swing_frac);
    double stance_frac = 1.0 - swing_frac;

    double linear_stride_speed = std::sqrt(vx_w*vx_w + vy_w*vy_w);
    double spin_stride_speed = std::abs(wz) * cmd.body_radius;
    double stride_speed = linear_stride_speed + spin_stride_speed;
    double spin_mix = spin_stride_speed / std::max(EPSILON, linear_stride_speed + spin_stride_speed);
    double spin_blend = spin_mix * spin_mix * (3.0 - 2.0 * spin_mix);
    double safe_stride = gait_params.max_safe_stride
                       + (gait_params.spin_safe_stride - gait_params.max_safe_stride) * spin_blend;
    double safe_reach = safe_stride * 0.95;

    bool is_moving = stride_speed > MOVING_SPEED_THRESHOLD || needs_realignment;
    double dynamic_cycle = cmd.gait.cycle_time;
    if (is_moving && stride_speed > EPSILON) {
        double requested_stride = stride_speed * (cmd.gait.cycle_time * stance_frac);
        if (requested_stride > safe_reach) {
            dynamic_cycle = safe_reach / (stride_speed * stance_frac);
        }
    } else if (needs_realignment) {
        dynamic_cycle = REALIGNMENT_FAST_CYCLE;
    }

    double speed_ratio = std::min(1.0, stride_speed / gait_params.step_height_full_speed);
    double height_scale = gait_params.step_height_min_factor
                        + (1.0 - gait_params.step_height_min_factor) * speed_ratio * speed_ratio;
    double dynamic_step_height = cmd.gait.step_height * height_scale;

    double old_phase = gs.master_phase;
    if (is_moving || [&]{ for (bool b : gs.is_swing) if (b) return true; return false; }()) {
        double p = gs.master_phase + dt / dynamic_cycle;
        gs.master_phase = p - std::floor(p);
        if (gs.master_phase < old_phase) {
            gs.current_mode = requested_mode;
        }
    } else {
        gs.current_mode = requested_mode;
    }

    get_phase_table(gs.current_mode, phase_table, swing_frac);
    double stance_time = dynamic_cycle * (1.0 - swing_frac);

    if (is_moving) {
        gs.pose_x += vx_w * dt;
        gs.pose_y += vy_w * dt;
        gs.pose_yaw += wz * dt;
    }

    double sway_hz = is_moving ? gait_params.sway_response_hz : gait_params.sway_response_hz * 3.5;
    double sway_alpha = std::min(1.0, sway_hz * dt);
    double ax_body = ax_w * std::cos(yaw) + ay_w * std::sin(yaw);
    double centripetal_ay = wz * gs.smooth_vx;
    double phase_roll = std::cos(2.0 * M_PI * gs.master_phase)
                      * gait_params.phase_roll_amp * speed_ratio;

    double target_pitch = std::clamp(-ax_body * gait_params.pitch_accel_gain,
                                     -gait_params.max_sway_angle,
                                      gait_params.max_sway_angle);
    double target_roll = std::clamp((centripetal_ay + gs.smooth_vy) * gait_params.roll_centripetal_gain
                                  + gs.smooth_vy * gait_params.roll_lateral_gain
                                  + phase_roll,
                                    -gait_params.max_sway_angle,
                                     gait_params.max_sway_angle);
    gs.sway_roll += (target_roll - gs.sway_roll) * sway_alpha;
    gs.sway_pitch += (target_pitch - gs.sway_pitch) * sway_alpha;

    base_pose.x = gs.pose_x + cmd.pose.x;
    base_pose.y = gs.pose_y + cmd.pose.y;
    base_pose.z = cmd.pose.z;
    base_pose.roll = cmd.pose.roll + gs.sway_roll;
    base_pose.pitch = cmd.pose.pitch + gs.sway_pitch;
    base_pose.yaw = gs.pose_yaw + cmd.pose.yaw;

    body_yaw = base_pose.yaw;
    BasePose solver_pose = base_pose;
    for (int i = 0; i < 6; i++) {
        double sw = swing_progress(gs.master_phase, phase_table[i].start, phase_table[i].end, 0.03);
        if (sw >= 0.0 && (is_moving || gs.is_swing[i])) {
            gs.drag_distance[i] = 0.0;
            if (!gs.is_swing[i]) {
                gs.is_swing[i] = true;
                gs.swing_start[i] = gs.feet_world[i];
            }

            double base_x = 0.0;
            double base_y = 0.0;
            compute_neutral_foot_pos(i, params, cmd.body_radius, base_x, base_y);

            double cx_w = base_pose.x + base_x * std::cos(body_yaw) - base_y * std::sin(body_yaw);
            double cy_w = base_pose.y + base_x * std::sin(body_yaw) + base_y * std::cos(body_yaw);

            double stride_trans_x = vx_w * (stance_time * 0.5) * cmd.gait.stride_scale
                                  * gait_params.reach_multiplier;
            double stride_trans_y = vy_w * (stance_time * 0.5) * cmd.gait.stride_scale
                                  * gait_params.reach_multiplier;

            double dx_rel = cx_w - base_pose.x;
            double dy_rel = cy_w - base_pose.y;
            double stride_rot_x = -wz * dy_rel * (stance_time * 0.5) * cmd.gait.stride_scale;
            double stride_rot_y =  wz * dx_rel * (stance_time * 0.5) * cmd.gait.stride_scale;

            Vec3 target = {cx_w + stride_trans_x + stride_rot_x,
                           cy_w + stride_trans_y + stride_rot_y,
                           0.0};
            target = clamp_to_workspace(target, i, base_pose, params, gait_params);
            Vec3 candidate = swing_trajectory(gs.swing_start[i], target, sw, dynamic_step_height);
            feet_world[i] = project_foot_to_safe_workspace(params, i, solver_pose, candidate);
        } else {
            gs.is_swing[i] = false;
            Vec3 foot_target = {gs.feet_world[i].x, gs.feet_world[i].y, 0.0};
            feet_world[i] = project_foot_to_safe_workspace(params, i, solver_pose, foot_target);
            gs.drag_distance[i] = 0.0;
        }
    }

    for (int i = 0; i < 6; i++) {
        gs.feet_world[i] = feet_world[i];
    }
}
