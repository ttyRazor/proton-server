#pragma once
// ============================================================
//  kinematics.h — inverse and forward kinematics helpers
// ============================================================
#include "config.h"
#include "types.h"
#include "robot_params.h"
#include <cmath>
#include <algorithm>

// ============================================================
//  4×4 column-major homogeneous transform matrix
//  Element (row, col) = m[col*4 + row]  (same layout as OpenGL)
// ============================================================
struct Mat4 {
    double m[16];

    // Identity constructor
    Mat4() {
        std::fill(m, m+16, 0.0);
        m[0] = m[5] = m[10] = m[15] = 1.0;
    }

    // Named constructors ---------------------------------------------------

    // Tr(x,y,z) — pure translation
    static Mat4 translate(double x, double y, double z) {
        Mat4 r;
        r.m[12] = x;  r.m[13] = y;  r.m[14] = z;
        return r;
    }

    // Rz(th) — rotation about Z axis
    // [cos -sin 0 0; sin cos 0 0; 0 0 1 0; 0 0 0 1]
    static Mat4 rotZ(double th) {
        Mat4 r;
        r.m[0]  =  std::cos(th);  r.m[4] = -std::sin(th);
        r.m[1]  =  std::sin(th);  r.m[5] =  std::cos(th);
        return r;
    }

    // Ry(th) — rotation about Y axis
    // [cos 0 sin 0; 0 1 0 0; -sin 0 cos 0; 0 0 0 1]
    static Mat4 rotY(double th) {
        Mat4 r;
        r.m[0]  =  std::cos(th);  r.m[8]  =  std::sin(th);
        r.m[2]  = -std::sin(th);  r.m[10] =  std::cos(th);
        return r;
    }

    // Rx(th) — rotation about X axis
    // [1 0 0 0; 0 cos -sin 0; 0 sin cos 0; 0 0 0 1]
    static Mat4 rotX(double th) {
        Mat4 r;
        r.m[5]  =  std::cos(th);  r.m[9]  = -std::sin(th);
        r.m[6]  =  std::sin(th);  r.m[10] =  std::cos(th);
        return r;
    }

    // Matrix multiply (result = this * rhs)
    Mat4 operator*(const Mat4& b) const {
        Mat4 r;
        std::fill(r.m, r.m+16, 0.0);
        for (int c = 0; c < 4; c++)
            for (int row = 0; row < 4; row++)
                for (int k = 0; k < 4; k++)
                    r.m[c*4+row] += m[k*4+row] * b.m[c*4+k];
        return r;
    }

    // Transform a 3D point (w=1) — returns 3D result
    Vec3 transform(double x, double y, double z) const {
        return {
            m[0]*x + m[4]*y + m[8]*z  + m[12],
            m[1]*x + m[5]*y + m[9]*z  + m[13],
            m[2]*x + m[6]*y + m[10]*z + m[14]
        };
    }
    Vec3 transform(const Vec3& v) const { return transform(v.x, v.y, v.z); }

    // Return float[16] copy (for OpenGL upload)
    void toFloat(float out[16]) const {
        for (int i = 0; i < 16; i++) out[i] = static_cast<float>(m[i]);
    }
};

// ============================================================
//  FK: 5 world-frame points per leg
//  Mirrors compute_leg_fk.m exactly.
// ============================================================
struct LegPoints {
    Vec3 pts[5];  // [mount, coxa_pivot, femur_joint, tibia_joint, foot_tip]
};

inline LegPoints compute_leg_fk(const RobotParams& params,
                                 const BasePose&    body_pose,
                                 const LegJoints&   joints,
                                 int                leg_index)
{
    auto Tr = Mat4::translate;
    auto Rx = Mat4::rotX;
    auto Ry = Mat4::rotY;
    auto Rz = Mat4::rotZ;

    double lc = params.dims.coxa_len;
    double lf = params.dims.femur_len;
    double lt = params.dims.tibia_len;
    double ma = params.mount_angles[leg_index];
    double mr = params.mount_radii[leg_index];
    double ca = params.coxa_frame_angles[leg_index];

    // Body → world
    // T_body = Tr(x,y,z) * Rz(yaw) * Ry(pitch) * Rx(roll)
    Mat4 T_body = Tr(body_pose.x, body_pose.y, body_pose.z)
                * Rz(body_pose.yaw)
                * Ry(body_pose.pitch)
                * Rx(body_pose.roll);

    // Mount point (= coxa pivot, no hip joint on this platform).
    Mat4 T_mount = T_body
                 * Tr(mr * std::cos(ma), mr * std::sin(ma), 0.0)
                 * Rz(ca);

    Mat4 T_coxa = T_mount * Rz(joints.coxa);

    // Femur joint
    // T_femur = T_coxa * Tr(lc,0,0) * Rx(pi/2) * Rz(joints.femur)
    Mat4 T_femur = T_coxa
                 * Tr(lc, 0.0, 0.0)
                 * Rx(M_PI / 2.0)
                 * Rz(joints.femur);

    // Tibia joint
    // T_tibia = T_femur * Tr(lf,0,0) * Rz(joints.tibia)
    Mat4 T_tibia = T_femur
                 * Tr(lf, 0.0, 0.0)
                 * Rz(joints.tibia);

    // Foot tip
    // T_foot = T_tibia * Tr(lt,0,0)
    Mat4 T_foot = T_tibia * Tr(lt, 0.0, 0.0);

    LegPoints pts;
    pts.pts[0] = T_mount.transform(0.0, 0.0, 0.0);
    pts.pts[1] = T_coxa.transform (0.0, 0.0, 0.0);
    pts.pts[2] = T_femur.transform (0.0, 0.0, 0.0);
    pts.pts[3] = T_tibia.transform (0.0, 0.0, 0.0);
    pts.pts[4] = T_foot.transform  (0.0, 0.0, 0.0);
    return pts;
}

// ============================================================
//  IK: solve one leg — mirrors solve_inverse_kinematics() in
//  hexapod_ik_solver.m
// ============================================================
static double clamp_value(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

static double radius_for_tibia_angle(const RobotDims& dims, double tibia_angle)
{
    double r2 = dims.femur_len*dims.femur_len
              + dims.tibia_len*dims.tibia_len
              + 2.0*dims.femur_len*dims.tibia_len*std::cos(tibia_angle);
    return std::sqrt(std::max(0.0, r2));
}

static void foot_from_leg_joint_angles(const RobotDims& dims,
                                       double coxa_angle,
                                       double femur_angle,
                                       double tibia_angle,
                                       double& x,
                                       double& y,
                                       double& z)
{
    double reach_x = dims.femur_len * std::cos(femur_angle)
                   + dims.tibia_len * std::cos(femur_angle + tibia_angle);
    double reach_z = dims.femur_len * std::sin(femur_angle)
                   + dims.tibia_len * std::sin(femur_angle + tibia_angle);
    double horiz = std::max(1e-6, dims.coxa_len + reach_x);

    x = horiz * std::cos(coxa_angle);
    y = horiz * std::sin(coxa_angle);
    z = reach_z;
}

static void project_leg_target_to_safe_workspace(const RobotDims& dims,
                                                  double& x,
                                                  double& y,
                                                  double& z)
{
    const double D2R = M_PI / 180.0;

    // Stay inside the hard solver limits so hardware commands do not ride
    // the mechanical stops when the animation asks for an unreachable foot.
    const double SAFE_COXA_LIMIT_DEG       = config::SafeCoxaLimitDeg;
    const double SAFE_TIBIA_FOLDED_DEG     = config::SafeTibiaFoldedDeg;
    const double SAFE_TIBIA_EXTENDED_DEG   = config::SafeTibiaExtendedDeg;

    double horiz = std::sqrt(x*x + y*y);
    double coxa_angle = (horiz <= 1e-9) ? 0.0 : std::atan2(y, x);
    coxa_angle = clamp_value(coxa_angle,
                             -SAFE_COXA_LIMIT_DEG * D2R,
                              SAFE_COXA_LIMIT_DEG * D2R);

    double l_coxa = horiz - dims.coxa_len;
    double l_cz   = std::sqrt(l_coxa*l_coxa + z*z);

    double safe_min_r = radius_for_tibia_angle(dims, SAFE_TIBIA_FOLDED_DEG * D2R);
    double safe_max_r = radius_for_tibia_angle(dims, SAFE_TIBIA_EXTENDED_DEG * D2R);
    double safe_r     = clamp_value(l_cz, safe_min_r, safe_max_r);

    if (l_cz > 1e-9) {
        double scale = safe_r / l_cz;
        l_coxa *= scale;
        z      *= scale;
    } else {
        l_coxa = safe_min_r;
        z = 0.0;
    }

    if (dims.coxa_len + l_coxa < 1e-6) {
        l_coxa = 1e-6 - dims.coxa_len;
        double z_mag = std::sqrt(std::max(0.0, safe_min_r*safe_min_r - l_coxa*l_coxa));
        z = (z < 0.0) ? -z_mag : z_mag;
    }

    l_cz = std::sqrt(l_coxa*l_coxa + z*z);
    double term1 = clamp_value((l_cz*l_cz + dims.femur_len*dims.femur_len - dims.tibia_len*dims.tibia_len)
                               / (2.0 * dims.femur_len * l_cz), -1.0, 1.0);
    double term2 = clamp_value((l_cz*l_cz - dims.femur_len*dims.femur_len - dims.tibia_len*dims.tibia_len)
                               / (2.0 * dims.femur_len * dims.tibia_len), -1.0, 1.0);

    double femur_angle = std::atan2(z, l_coxa) + std::acos(term1);
    double tibia_angle = -std::acos(term2);
    femur_angle = clamp_value(femur_angle, config::FemurMinDeg * D2R, config::FemurMaxDeg * D2R);
    tibia_angle = clamp_value(tibia_angle,
                              SAFE_TIBIA_FOLDED_DEG * D2R,
                              SAFE_TIBIA_EXTENDED_DEG * D2R);

    foot_from_leg_joint_angles(dims, coxa_angle, femur_angle, tibia_angle, x, y, z);
}

static Vec3 world_to_leg_local(double mount_angle,
                               double mount_radius,
                               double coxa_frame_angle,
                               const BasePose& body_pose,
                               const Vec3& foot_world)
{
    double px = foot_world.x - body_pose.x;
    double py = foot_world.y - body_pose.y;
    double pz = foot_world.z - body_pose.z;

    double sy = std::sin(-body_pose.yaw),   cy = std::cos(-body_pose.yaw);
    double sp = std::sin(-body_pose.pitch), cp = std::cos(-body_pose.pitch);
    double sr = std::sin(-body_pose.roll),  cr = std::cos(-body_pose.roll);

    double px1 = px*cy - py*sy;
    double py1 = px*sy + py*cy;
    double pz1 = pz;

    double px2 = px1*cp + pz1*sp;
    double py2 = py1;
    double pz2 = -px1*sp + pz1*cp;

    double txb = px2;
    double tyb = py2*cr - pz2*sr;
    double tzb = py2*sr + pz2*cr;

    double cx_piv = mount_radius * std::cos(mount_angle);
    double cy_piv = mount_radius * std::sin(mount_angle);

    double dx = txb - cx_piv;
    double dy = tyb - cy_piv;

    double s_leg = std::sin(-coxa_frame_angle);
    double c_leg = std::cos(-coxa_frame_angle);

    return {
        dx*c_leg - dy*s_leg,
        dx*s_leg + dy*c_leg,
        tzb
    };
}

static Vec3 leg_local_to_world(double mount_angle,
                               double mount_radius,
                               double coxa_frame_angle,
                               const BasePose& body_pose,
                               const Vec3& foot_leg)
{
    double s_mount = std::sin(mount_angle);
    double c_mount = std::cos(mount_angle);
    double s_frame = std::sin(coxa_frame_angle);
    double c_frame = std::cos(coxa_frame_angle);

    double txb = foot_leg.x*c_frame - foot_leg.y*s_frame
               + mount_radius*c_mount;
    double tyb = foot_leg.x*s_frame + foot_leg.y*c_frame
               + mount_radius*s_mount;
    double tzb = foot_leg.z;

    double sr = std::sin(body_pose.roll),  cr = std::cos(body_pose.roll);
    double sp = std::sin(body_pose.pitch), cp = std::cos(body_pose.pitch);
    double sy = std::sin(body_pose.yaw),   cy = std::cos(body_pose.yaw);

    double px1 = txb;
    double py1 = tyb*cr - tzb*sr;
    double pz1 = tyb*sr + tzb*cr;

    double px2 = px1*cp + pz1*sp;
    double py2 = py1;
    double pz2 = -px1*sp + pz1*cp;

    return {
        body_pose.x + px2*cy - py2*sy,
        body_pose.y + px2*sy + py2*cy,
        body_pose.z + pz2
    };
}

[[maybe_unused]] static Vec3 project_foot_to_safe_workspace(const RobotParams& params,
                                                            int leg,
                                                            const BasePose& body_pose,
                                                            const Vec3& foot_world)
{
    Vec3 foot_leg = world_to_leg_local(params.mount_angles[leg],
                                       params.mount_radii[leg],
                                       params.coxa_frame_angles[leg],
                                       body_pose,
                                       foot_world);
    project_leg_target_to_safe_workspace(params.dims, foot_leg.x, foot_leg.y, foot_leg.z);
    return leg_local_to_world(params.mount_angles[leg],
                              params.mount_radii[leg],
                              params.coxa_frame_angles[leg],
                              body_pose,
                              foot_leg);
}

static bool solve_leg_ik(const RobotDims& dims,
                          double mount_angle,
                          double mount_radius,
                          double coxa_frame_angle,
                          const BasePose& body_pose,
                          const Vec3& foot_world,
                          LegJoints& joints)
{
    // ---- 1. World → leg-local frame -------------------------------
    Vec3 foot_leg = world_to_leg_local(mount_angle, mount_radius, coxa_frame_angle,
                                       body_pose, foot_world);
    double x = foot_leg.x;
    double y = foot_leg.y;
    double z = foot_leg.z;

    // ---- 3. Planar IK (coxa angle) -------------------------------
    double coxa_angle = (x*x + y*y <= 1e-12) ? 0.0 : std::atan2(y, x);

    // ---- 4. Sagittal-plane IK (femur + tibia) --------------------
    double l       = std::sqrt(x*x + y*y);
    double l_coxa  = l - dims.coxa_len;
    double l_cz    = std::sqrt(l_coxa*l_coxa + z*z);

    double max_r = dims.femur_len + dims.tibia_len;
    double min_r = std::abs(dims.femur_len - dims.tibia_len);

    if (l_cz > max_r) {
        double scale = (max_r - 1e-4) / std::max(l_cz, 1e-9);
        l_coxa *= scale;
        z *= scale;
    } else if (l_cz < min_r && l_cz > 1e-6) {
        double scale = (min_r + 1e-4) / l_cz;
        l_coxa *= scale;
        z *= scale;
    }
    l_cz = std::sqrt(l_coxa*l_coxa + z*z);

    double term1 = clamp_value((l_cz*l_cz + dims.femur_len*dims.femur_len - dims.tibia_len*dims.tibia_len)
                               / (2.0 * dims.femur_len * l_cz), -1.0, 1.0);
    double term2 = clamp_value((l_cz*l_cz - dims.femur_len*dims.femur_len - dims.tibia_len*dims.tibia_len)
                               / (2.0 * dims.femur_len * dims.tibia_len), -1.0, 1.0);

    double femur_angle = std::atan2(z, l_coxa) + std::acos(term1);
    double tibia_angle = -std::acos(term2);

    // ---- 5. Joint limit check ------------------------------------
    const double D2R = M_PI / 180.0;
    coxa_angle  = clamp_value(coxa_angle, -config::SafeCoxaLimitDeg*D2R, config::SafeCoxaLimitDeg*D2R);
    femur_angle = clamp_value(femur_angle, config::FemurMinDeg*D2R, config::FemurMaxDeg*D2R);
    tibia_angle = clamp_value(tibia_angle, config::SafeTibiaFoldedDeg*D2R, config::SafeTibiaExtendedDeg*D2R);

    joints.hip   = 0.0;
    joints.coxa  = coxa_angle;
    joints.femur = femur_angle;
    joints.tibia = tibia_angle;
    return true;
}

// ---- Full hexapod IK (all 6 legs) --------------------------------
// Mirrors hexapod_ik_solver.m
inline void hexapod_ik_solver(const BasePose&    base_pose,
                               double             body_radius,
                               const RobotParams& params,
                               const Vec3         feet_world[6],
                               RobotState&        state,
                               BasePose&          final_pose)
{
    final_pose = base_pose;

    for (int i = 0; i < 6; i++) {
        bool ok = solve_leg_ik(params.dims,
                               params.mount_angles[i],
                               params.mount_radii[i],
                               params.coxa_frame_angles[i],
                               base_pose, feet_world[i],
                               state.legs[i]);
        if (!ok) {
            // Keep the last valid command instead of snapping to a tucked pose.
            // The projection above should handle ordinary unreachable targets.
        }
    }
    (void)body_radius;
}
