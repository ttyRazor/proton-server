#pragma once
// ============================================================
//  robot_params.h — hexapod physical parameters
// ============================================================
#include "config.h"
#include "types.h"
#include <algorithm>
#include <cmath>

// ---- Link dimensions -------------------------------------------
struct RobotDims {
    double coxa_len     = config::CoxaLength;
    double femur_len    = config::FemurLength;
    double tibia_len    = config::TibiaLength;
};

// ---- Body geometry ---------------------------------------------
struct BodyShape {
    double length  = config::BodyLength;
    double width   = config::BodyWidth;
    double chamfer = config::BodyChamfer;
};

// ---- Full robot parameter set ----------------------------------
struct RobotParams {
    RobotDims dims;
    BodyShape body;
    double mount_angles[6];               // body centre to coxa pivot, radians
    double mount_radii[6];                // distance from body centre to coxa pivot, m
    double coxa_frame_angles[6];          // neutral coxa direction, radians
    Vec3   default_foot_positions[6];     // default foot positions in world frame
    double l_horiz = 0.0;                  // neutral horizontal reach from coxa pivot
};

// ---- Factory function ------------------------------------------
//
//  Leg layout is loaded from config. Mount angles are measured from +X
//  forward, with positive rotation toward the left side of the robot.
//
inline RobotParams get_robot_params()
{
    RobotParams p;

    const double D2R = M_PI / 180.0;

    auto& d = p.dims;
    double tibia_target = (config::NeutralStanceKneeAngleDeg - 180.0) * D2R;
    double lcz = std::sqrt(d.femur_len*d.femur_len + d.tibia_len*d.tibia_len
                           + 2.0*d.femur_len*d.tibia_len*std::cos(tibia_target));
    double lc_sq = lcz*lcz - config::NeutralStanceBodyHeight*config::NeutralStanceBodyHeight;
    p.l_horiz = std::sqrt(std::max(0.0, lc_sq)) + d.coxa_len;

    for (int i = 0; i < 6; i++) {
        p.mount_angles[i] = config::MountAnglesDeg[i] * D2R;
        p.mount_radii[i] = config::MountRadii[i];
        p.coxa_frame_angles[i] = p.mount_angles[i];

        double neutral_coxa_angle = p.mount_angles[i] + config::CoxaOffsetsDeg[i] * D2R;
        double pivot_x = p.mount_radii[i] * std::cos(p.mount_angles[i]);
        double pivot_y = p.mount_radii[i] * std::sin(p.mount_angles[i]);
        p.default_foot_positions[i].x = pivot_x + p.l_horiz * std::cos(neutral_coxa_angle);
        p.default_foot_positions[i].y = pivot_y + p.l_horiz * std::sin(neutral_coxa_angle);
        p.default_foot_positions[i].z = 0.0;
    }

    return p;
}
