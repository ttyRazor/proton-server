#include "config.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

namespace config {
namespace {

std::string trim(std::string value)
{
    auto is_space = [](unsigned char ch) { return std::isspace(ch) != 0; };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(),
                                            [&](char ch) { return !is_space(ch); }));
    value.erase(std::find_if(value.rbegin(), value.rend(),
                             [&](char ch) { return !is_space(ch); }).base(),
                value.end());
    return value;
}

std::string lowercase(std::string value)
{
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

std::vector<std::string> split(std::string value, char delimiter)
{
    std::vector<std::string> parts;
    std::stringstream stream(value);
    std::string part;
    while (std::getline(stream, part, delimiter)) {
        parts.push_back(trim(part));
    }
    return parts;
}

bool parse_double(const std::string& text, double& out)
{
    try {
        size_t used = 0;
        out = std::stod(text, &used);
        return used == trim(text).size();
    } catch (...) {
        return false;
    }
}

bool parse_int(const std::string& text, int& out)
{
    try {
        size_t used = 0;
        out = std::stoi(text, &used);
        return used == trim(text).size();
    } catch (...) {
        return false;
    }
}

bool parse_bool(const std::string& text, bool& out)
{
    const std::string value = lowercase(trim(text));
    if (value == "true" || value == "yes" || value == "on" || value == "reversed" || value == "reverse") {
        out = true;
        return true;
    }
    if (value == "false" || value == "no" || value == "off" || value == "normal") {
        out = false;
        return true;
    }
    return false;
}

bool parse_double_list(const std::string& text, std::array<double, 6>& out)
{
    const auto parts = split(text, ',');
    if (parts.size() != out.size()) return false;
    for (size_t i = 0; i < out.size(); i++) {
        if (!parse_double(parts[i], out[i])) return false;
    }
    return true;
}

bool parse_mm_list(const std::string& text, std::array<double, 6>& out)
{
    if (!parse_double_list(text, out)) return false;
    for (double& value : out) value /= 1000.0;
    return true;
}

bool parse_leg_name(std::string name, int& leg)
{
    name = lowercase(trim(name));
    static constexpr const char* names[6] = {"r1", "r2", "r3", "l1", "l2", "l3"};
    for (int i = 0; i < 6; i++) {
        if (name == names[i]) {
            leg = i;
            return true;
        }
    }
    return false;
}

bool parse_joint_name(std::string name, int& joint)
{
    name = lowercase(trim(name));
    if (name == "coxa") {
        joint = 0;
        return true;
    }
    if (name == "femur") {
        joint = 1;
        return true;
    }
    if (name == "tibia") {
        joint = 2;
        return true;
    }
    return false;
}

bool parse_servo_pin(const std::string& text, int& leg, int& joint, bool& reversed)
{
    const auto parts = split(text, ',');
    if (parts.empty()) return false;

    const auto target = split(parts[0], '.');
    if (target.size() != 2 || !parse_leg_name(target[0], leg) || !parse_joint_name(target[1], joint)) {
        return false;
    }

    if (parts.size() >= 2 && !parse_bool(parts[1], reversed)) {
        return false;
    }
    return true;
}

bool parse_pin_index(const std::string& key, int& pin)
{
    constexpr const char* prefix = "servo2040.pin";
    if (key.rfind(prefix, 0) != 0) return false;

    std::string number = key.substr(std::char_traits<char>::length(prefix));
    if (!number.empty() && number[0] == '_') number.erase(number.begin());
    return parse_int(number, pin) && pin >= 0 && pin < 18;
}

double to_degrees(double radians)
{
    constexpr double RadToDeg = 180.0 / 3.14159265358979323846;
    return radians * RadToDeg;
}

void derive_body_layout_from_frame()
{
    const double half_length = FrontRearCoxaLength * 0.5;
    const double half_front_width = FrontRearCoxaWidth * 0.5;
    const double half_middle_width = MiddleCoxaWidth * 0.5;

    const std::array<double, 6> mount_x = {
        half_length, 0.0, -half_length,
        half_length, 0.0, -half_length
    };
    const std::array<double, 6> mount_y = {
        -half_front_width, -half_middle_width, -half_front_width,
        half_front_width, half_middle_width, half_front_width
    };
    for (size_t leg = 0; leg < MountAnglesDeg.size(); leg++) {
        MountAnglesDeg[leg] = to_degrees(std::atan2(mount_y[leg], mount_x[leg]));
        MountRadii[leg] = std::hypot(mount_x[leg], mount_y[leg]);
    }
}

void assign_positive_mm(double& target, const std::string& value, LoadResult& result, int line)
{
    double mm = 0.0;
    if (!parse_double(value, mm) || mm <= 0.0) {
        result.errors.push_back("line " + std::to_string(line) + ": expected a positive millimetre value");
        return;
    }
    target = mm / 1000.0;
}

void assign_double(double& target, const std::string& value, LoadResult& result, int line)
{
    if (!parse_double(value, target)) {
        result.errors.push_back("line " + std::to_string(line) + ": expected a number");
    }
}

void assign_int(int& target, const std::string& value, LoadResult& result, int line)
{
    if (!parse_int(value, target)) {
        result.errors.push_back("line " + std::to_string(line) + ": expected an integer");
    }
}

void validate_config(LoadResult& result)
{
    Servo2040FlipPwmSum = 2 * PwmNeutral;

    auto require_positive = [&](const char* name, double value) {
        if (value <= 0.0) result.errors.push_back(std::string(name) + " must be greater than zero");
    };

    require_positive("coxa_length_mm", CoxaLength);
    require_positive("femur_length_mm", FemurLength);
    require_positive("tibia_length_mm", TibiaLength);
    require_positive("l1_to_r1_mm", FrontRearCoxaWidth);
    require_positive("l2_to_r2_mm", MiddleCoxaWidth);
    require_positive("l1_to_l3_mm", FrontRearCoxaLength);
    require_positive("standing_height_mm", Motion.start_height);
    require_positive("sitting_height_mm", SitBodyHeight);
    require_positive("footprint_design_height_mm", NeutralStanceBodyHeight);
    require_positive("servo_type_deg", ServoAngleRangeDeg);

    if (PwmMin >= PwmNeutral || PwmNeutral >= PwmMax) {
        result.errors.push_back("servo_pwm_min_us < servo_pwm_neutral_us < servo_pwm_max_us is required");
    }
    if (SafeTibiaFoldedDeg >= SafeTibiaExtendedDeg) {
        result.errors.push_back("safe_tibia_folded_deg must be less than safe_tibia_extended_deg");
    }
    if (FemurMinDeg >= FemurMaxDeg) {
        result.errors.push_back("femur_min_deg must be less than femur_max_deg");
    }
    if (FrontRearCoxaWidth > MiddleCoxaWidth) {
        result.errors.push_back("l1_to_r1_mm must be less than or equal to l2_to_r2_mm");
    }
    if (CornerLegAngleDeg <= 0.0 || CornerLegAngleDeg >= 90.0) {
        result.errors.push_back("corner_leg_angle_deg must be greater than 0 and less than 90");
    }
    if (NeutralStanceKneeAngleDeg <= 0.0 || NeutralStanceKneeAngleDeg >= 180.0) {
        result.errors.push_back("footprint_design_knee_angle_deg must be greater than 0 and less than 180");
    }
    if (StepHeightMin > StepHeightMax) {
        result.errors.push_back("step_height_min_mm must be less than or equal to step_height_max_mm");
    }
    if (BodyRadiusMin > BodyRadiusMax) {
        result.errors.push_back("body_radius_min_mm must be less than or equal to body_radius_max_mm");
    }
    double middle_mount_radius = 0.5 * (MountRadii[1] + MountRadii[4]);
    if (BodyRadiusMin <= middle_mount_radius) {
        result.errors.push_back("body_radius_min_mm must be greater than the middle coxa mount radius");
    }
    if (SitBodyHeight > Motion.start_height || Motion.start_height > Motion.height_max) {
        result.warnings.push_back("standing_height_mm is outside the configured walking height range");
    }
    if (Motion.linear_speed_min > Motion.linear_speed_max) {
        result.errors.push_back("speed_min_mps must be less than or equal to speed_max_mps");
    }
    if (Motion.body_radius < BodyRadiusMin || Motion.body_radius > BodyRadiusMax) {
        result.warnings.push_back("body_radius_mm is outside the configured stance radius range");
    }
    if (Motion.step_height < StepHeightMin || Motion.step_height > StepHeightMax) {
        result.warnings.push_back("step_height_mm is outside the configured step height range");
    }

    const double max_reach = CoxaLength + FemurLength + TibiaLength;
    constexpr double D2R = 3.14159265358979323846 / 180.0;
    double reach_from_pivot = BodyRadiusMax - middle_mount_radius;
    for (int i = 0; i < 6; i++) {
        require_positive(("leg mount radius " + std::to_string(i)).c_str(), MountRadii[i]);
        double pivot_x = MountRadii[i] * std::cos(MountAnglesDeg[i] * D2R);
        double pivot_y = MountRadii[i] * std::sin(MountAnglesDeg[i] * D2R);
        double neutral_angle = (MountAnglesDeg[i] + CoxaOffsetsDeg[i]) * D2R;
        double foot_x = pivot_x + reach_from_pivot * std::cos(neutral_angle);
        double foot_y = pivot_y + reach_from_pivot * std::sin(neutral_angle);
        double reach = std::hypot(foot_x - pivot_x, foot_y - pivot_y);
        if (reach > max_reach) {
            result.warnings.push_back("leg " + std::to_string(i)
                                      + " stance radius may be unreachable with the configured link lengths");
        }
    }
    for (int pin = 0; pin < 18; pin++) {
        if (Servo2040PinLeg[pin] < 0 || Servo2040PinLeg[pin] >= 6
            || Servo2040PinJoint[pin] < 0 || Servo2040PinJoint[pin] >= 3) {
            result.errors.push_back("servo2040.pin" + std::to_string(pin) + " maps outside the valid leg/joint range");
        }
    }

    result.ok = result.errors.empty();
}

} // namespace

LoadResult load_user_config(const std::string& path, bool required)
{
    LoadResult result;
    std::ifstream file(path);
    if (!file) {
        result.ok = !required;
        if (required) result.errors.push_back("could not open config file: " + path);
        return result;
    }

    result.loaded = true;
    bool frame_layout_changed = false;
    bool advanced_layout_changed = false;
    std::string line;
    for (int line_number = 1; std::getline(file, line); line_number++) {
        const size_t comment = line.find_first_of("#;");
        if (comment != std::string::npos) line.erase(comment);
        line = trim(line);
        if (line.empty()) continue;

        const size_t equals = line.find('=');
        if (equals == std::string::npos) {
            result.errors.push_back("line " + std::to_string(line_number) + ": expected key = value");
            continue;
        }

        const std::string key = lowercase(trim(line.substr(0, equals)));
        const std::string value = trim(line.substr(equals + 1));
        int pin = -1;

        if (key == "coxa_length_mm") assign_positive_mm(CoxaLength, value, result, line_number);
        else if (key == "femur_length_mm") assign_positive_mm(FemurLength, value, result, line_number);
        else if (key == "tibia_length_mm") assign_positive_mm(TibiaLength, value, result, line_number);
        else if (key == "l1_to_r1" || key == "l1_to_r1_mm" || key == "front_rear_coxa_width_mm") {
            assign_positive_mm(FrontRearCoxaWidth, value, result, line_number);
            frame_layout_changed = true;
        } else if (key == "l2_to_r2" || key == "l2_to_r2_mm" || key == "middle_coxa_width_mm") {
            assign_positive_mm(MiddleCoxaWidth, value, result, line_number);
            frame_layout_changed = true;
        } else if (key == "l1_to_l3" || key == "l1_to_l3_mm" || key == "front_rear_coxa_length_mm") {
            assign_positive_mm(FrontRearCoxaLength, value, result, line_number);
            frame_layout_changed = true;
        } else if (key == "corner_leg_angle" || key == "corner_leg_angle_deg") {
            assign_double(CornerLegAngleDeg, value, result, line_number);
            frame_layout_changed = true;
        } else if (key == "body_length_mm") {
            assign_positive_mm(BodyLength, value, result, line_number);
            advanced_layout_changed = true;
        } else if (key == "body_width_mm") {
            assign_positive_mm(BodyWidth, value, result, line_number);
            advanced_layout_changed = true;
        } else if (key == "body_chamfer_mm") {
            assign_positive_mm(BodyChamfer, value, result, line_number);
            advanced_layout_changed = true;
        } else if (key == "standing_height_mm") assign_positive_mm(Motion.start_height, value, result, line_number);
        else if (key == "sitting_height_mm") assign_positive_mm(SitBodyHeight, value, result, line_number);
        else if (key == "footprint_design_height_mm" || key == "neutral_stance_body_height_mm") {
            assign_positive_mm(NeutralStanceBodyHeight, value, result, line_number);
        } else if (key == "footprint_design_knee_angle_deg" || key == "neutral_stance_knee_angle_deg"
                   || key == "neutral_knee_angle_deg") {
            assign_double(NeutralStanceKneeAngleDeg, value, result, line_number);
        } else if (key == "neutral_tibia_external_angle_deg") {
            result.warnings.push_back("line " + std::to_string(line_number)
                                      + ": ignored deprecated key " + key);
        }
        else if (key == "body_radius_mm") assign_positive_mm(Motion.body_radius, value, result, line_number);
        else if (key == "step_height_mm") assign_positive_mm(Motion.step_height, value, result, line_number);
        else if (key == "height_min_mm") {
            result.warnings.push_back("line " + std::to_string(line_number)
                                      + ": ignored deprecated key height_min_mm; use sitting_height_mm");
        }
        else if (key == "voltage_warn_v") assign_double(Servo2040VoltageWarn, value, result, line_number);
        else if (key == "voltage_critical_v") assign_double(Servo2040VoltageCritical, value, result, line_number);
        else if (key == "height_max_mm") assign_positive_mm(Motion.height_max, value, result, line_number);
        else if (key == "speed_min_mps") assign_double(Motion.linear_speed_min, value, result, line_number);
        else if (key == "speed_max_mps") assign_double(Motion.linear_speed_max, value, result, line_number);
        else if (key == "height_rate_mps") assign_double(Motion.height_rate, value, result, line_number);
        else if (key == "walk_speed_mps") assign_double(Motion.walk_speed, value, result, line_number);
        else if (key == "strafe_speed_mps") assign_double(Motion.strafe_speed, value, result, line_number);
        else if (key == "spin_rate_radps") assign_double(Motion.spin_rate, value, result, line_number);
        else if (key == "servo_type_deg" || key == "servo_angle_range_deg") assign_double(ServoAngleRangeDeg, value, result, line_number);
        else if (key == "servo_pwm_min_us") assign_int(PwmMin, value, result, line_number);
        else if (key == "servo_pwm_max_us") assign_int(PwmMax, value, result, line_number);
        else if (key == "servo_pwm_neutral_us") assign_int(PwmNeutral, value, result, line_number);
        else if (key == "servo_center_coxa_deg") assign_double(ServoCoxaCenterDeg, value, result, line_number);
        else if (key == "servo_center_femur_deg") assign_double(ServoFemurCenterDeg, value, result, line_number);
        else if (key == "servo_center_tibia_deg") assign_double(ServoTibiaCenterDeg, value, result, line_number);
        else if (key == "safe_coxa_limit_deg") assign_double(SafeCoxaLimitDeg, value, result, line_number);
        else if (key == "safe_tibia_folded_deg") assign_double(SafeTibiaFoldedDeg, value, result, line_number);
        else if (key == "safe_tibia_extended_deg") assign_double(SafeTibiaExtendedDeg, value, result, line_number);
        else if (key == "femur_min_deg") assign_double(FemurMinDeg, value, result, line_number);
        else if (key == "femur_max_deg") assign_double(FemurMaxDeg, value, result, line_number);
        else if (key == "body_radius_min_mm") assign_positive_mm(BodyRadiusMin, value, result, line_number);
        else if (key == "body_radius_max_mm") assign_positive_mm(BodyRadiusMax, value, result, line_number);
        else if (key == "step_height_min_mm") assign_positive_mm(StepHeightMin, value, result, line_number);
        else if (key == "step_height_max_mm") assign_positive_mm(StepHeightMax, value, result, line_number);
        else if (key == "mount_angles_deg") {
            advanced_layout_changed = true;
            if (!parse_double_list(value, MountAnglesDeg)) {
                result.errors.push_back("line " + std::to_string(line_number) + ": expected six comma-separated angles");
            }
        } else if (key == "mount_radii_mm") {
            advanced_layout_changed = true;
            if (!parse_mm_list(value, MountRadii)) {
                result.errors.push_back("line " + std::to_string(line_number) + ": expected six comma-separated millimetre values");
            }
        } else if (key == "coxa_offsets_deg") {
            if (!parse_double_list(value, CoxaOffsetsDeg)) {
                result.errors.push_back("line " + std::to_string(line_number) + ": expected six comma-separated angles");
            }
        } else if (parse_pin_index(key, pin)) {
            int leg = 0, joint = 0;
            bool reversed = Servo2040FlipForHardware[pin];
            if (!parse_servo_pin(value, leg, joint, reversed)) {
                result.errors.push_back("line " + std::to_string(line_number)
                                        + ": expected servo2040.pinN = R1.coxa, normal|reversed");
            } else {
                Servo2040PinLeg[pin] = leg;
                Servo2040PinJoint[pin] = joint;
                Servo2040FlipForHardware[pin] = reversed;
            }
        } else {
            bool handled_leg_key = false;
            for (int leg = 0; leg < 6; leg++) {
                static constexpr const char* names[6] = {"r1", "r2", "r3", "l1", "l2", "l3"};
                const std::string prefix = std::string("leg.") + names[leg] + ".";
                if (key.rfind(prefix, 0) != 0) continue;
                handled_leg_key = true;
                const std::string field = key.substr(prefix.size());
                if (field == "mount_angle_deg") {
                    assign_double(MountAnglesDeg[leg], value, result, line_number);
                    advanced_layout_changed = true;
                } else if (field == "mount_radius_mm") {
                    assign_positive_mm(MountRadii[leg], value, result, line_number);
                    advanced_layout_changed = true;
                } else if (field == "coxa_offset_deg") {
                    assign_double(CoxaOffsetsDeg[leg], value, result, line_number);
                } else result.errors.push_back("line " + std::to_string(line_number) + ": unknown leg field " + field);
                break;
            }
            if (!handled_leg_key) {
                result.warnings.push_back("line " + std::to_string(line_number) + ": ignored unknown key " + key);
            }
        }
    }

    if (frame_layout_changed) {
        if (advanced_layout_changed) {
            result.warnings.push_back("frame measurements override advanced body and leg mount layout keys");
        }
        derive_body_layout_from_frame();
    }

    validate_config(result);
    return result;
}

} // namespace config
