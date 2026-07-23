#include "wifi_controller.h"

#include "config.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <csignal>
#include <optional>
#include <random>
#include <sstream>
#include <vector>

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    using NativeSocket = SOCKET;
    using SocketLength = int;
#else
    #include <arpa/inet.h>
    #include <cerrno>
    #include <fcntl.h>
    #include <netinet/in.h>
    #include <sys/select.h>
    #include <sys/socket.h>
    #include <unistd.h>
    using NativeSocket = int;
    using SocketLength = socklen_t;
#endif

namespace {

constexpr double ACTIVE_DEADZONE = 0.05;
constexpr double CONNECTED_TIMEOUT_SECONDS = 2.0;
constexpr double ACTIVE_TIMEOUT_SECONDS = 0.35;
constexpr double HEIGHT_TARGET_TOLERANCE = 0.002;
constexpr double MOTION_TARGET_TOLERANCE = 0.002;
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

using Clock = std::chrono::steady_clock;

NativeSocket native_socket(WifiSocket socket)
{
    return static_cast<NativeSocket>(socket);
}

WifiSocket wifi_socket(NativeSocket socket)
{
    return static_cast<WifiSocket>(socket);
}

bool valid_socket(WifiSocket socket)
{
    return socket != InvalidWifiSocket;
}

void close_wifi_socket(WifiSocket socket)
{
    if (!valid_socket(socket)) return;
#ifdef _WIN32
    closesocket(native_socket(socket));
#else
    close(socket);
#endif
}

void shutdown_wifi_socket(WifiSocket socket)
{
    if (!valid_socket(socket)) return;
#ifdef _WIN32
    shutdown(native_socket(socket), SD_BOTH);
#else
    shutdown(socket, SHUT_RDWR);
#endif
}

std::string socket_error_message()
{
#ifdef _WIN32
    return "Winsock error " + std::to_string(WSAGetLastError());
#else
    return std::strerror(errno);
#endif
}

bool start_socket_runtime()
{
#ifdef _WIN32
    WSADATA data = {};
    return WSAStartup(MAKEWORD(2, 2), &data) == 0;
#else
    return true;
#endif
}

void stop_socket_runtime()
{
#ifdef _WIN32
    WSACleanup();
#endif
}

int select_width(WifiSocket socket)
{
#ifdef _WIN32
    (void)socket;
    return 0;
#else
    return socket + 1;
#endif
}

double clamp_axis(double value)
{
    return std::clamp(value, -1.0, 1.0);
}

double clamp_body_radius(double value)
{
    return std::clamp(value, config::BodyRadiusMin, config::BodyRadiusMax);
}

double clamp_step_height(double value)
{
    return std::clamp(value, config::StepHeightMin, config::StepHeightMax);
}

double clamp_speed(double value)
{
    return std::clamp(value, config::Motion.linear_speed_min, config::Motion.linear_speed_max);
}

double default_speed()
{
    return clamp_speed(config::Motion.walk_speed);
}

bool joystick_active(const WifiJoystick& joystick)
{
    return std::fabs(joystick.x) > ACTIVE_DEADZONE
        || std::fabs(joystick.y) > ACTIVE_DEADZONE;
}

bool height_axis_mapped(const WifiControllerSnapshot& snapshot)
{
    return snapshot.primary_x_action == WifiAxisAction::HEIGHT
        || snapshot.primary_y_action == WifiAxisAction::HEIGHT
        || snapshot.secondary_x_action == WifiAxisAction::HEIGHT
        || snapshot.secondary_y_action == WifiAxisAction::HEIGHT;
}

bool is_json_key_separator(char ch)
{
    return ch == '{' || ch == ',';
}

std::optional<std::size_t> json_key_position(const std::string& body,
                                             const std::string& key,
                                             std::size_t start = 0)
{
    std::size_t search = start;
    while (true) {
        const auto key_position = body.find(key, search);
        if (key_position == std::string::npos) return std::nullopt;

        const auto previous = key_position == 0
                            ? std::string::npos
                            : body.find_last_not_of(" \t\r\n", key_position - 1);
        const auto colon = body.find_first_not_of(" \t\r\n", key_position + key.size());
        if ((previous == std::string::npos || is_json_key_separator(body[previous]))
            && colon != std::string::npos
            && body[colon] == ':') {
            return key_position;
        }
        search = key_position + key.size();
    }
}

const char* axis_action_name(WifiAxisAction action)
{
    switch (action) {
        case WifiAxisAction::NONE: return "none";
        case WifiAxisAction::MARCH: return "march";
        case WifiAxisAction::STRAFE: return "strafe";
        case WifiAxisAction::TURN: return "spin";
        case WifiAxisAction::HEIGHT: return "height";
        case WifiAxisAction::FRONT_BACK: return "fwbk";
        case WifiAxisAction::SIDE_SIDE: return "side";
        case WifiAxisAction::CIRCLE: return "circle";
    }
    return "none";
}

std::string lowercase(std::string value)
{
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

std::optional<WifiAxisAction> axis_action_from_string(std::string value)
{
    value = lowercase(value);
    if (value == "none" || value == "off" || value == "disabled") return WifiAxisAction::NONE;
    if (value == "march" || value == "walk" || value == "fwdback") return WifiAxisAction::MARCH;
    if (value == "strafe") return WifiAxisAction::STRAFE;
    if (value == "turn" || value == "spin") return WifiAxisAction::TURN;
    if (value == "height" || value == "lift") return WifiAxisAction::HEIGHT;
    if (value == "fwbk" || value == "front_back" || value == "frontback") {
        return WifiAxisAction::FRONT_BACK;
    }
    if (value == "side" || value == "side_side" || value == "sideside") {
        return WifiAxisAction::SIDE_SIDE;
    }
    if (value == "circle") return WifiAxisAction::CIRCLE;
    return std::nullopt;
}

template <typename T, typename... Rest>
std::optional<T> first_present(const std::optional<T>& first, const Rest&... rest)
{
    if (first) return first;
    if constexpr (sizeof...(rest) == 0) {
        return std::nullopt;
    } else {
        return first_present<T>(rest...);
    }
}

template <typename... Values>
bool any_present(const Values&... values)
{
    return (... || values.has_value());
}

bool control_active(const WifiControllerSnapshot& snapshot)
{
    const bool directional_control_enabled =
        snapshot.relay_status && snapshot.target_relay_status;
    return (directional_control_enabled
            && (joystick_active(snapshot.primary)
                || joystick_active(snapshot.secondary)))
        || snapshot.height_control_active
        || snapshot.body_radius_control_active
        || snapshot.step_height_control_active
        || snapshot.speed_control_active
        || snapshot.position_control_active
        || snapshot.gait_control_active
        || snapshot.relay_control_active;
}

std::string json_escape(const std::string& value)
{
    std::string escaped;
    escaped.reserve(value.size());
    for (char ch : value) {
        switch (ch) {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20) {
                    escaped += ' ';
                } else {
                    escaped.push_back(ch);
                }
                break;
        }
    }
    return escaped;
}

double seconds_since(Clock::time_point time)
{
    if (time == Clock::time_point::min()) return 1e9;
    return std::chrono::duration<double>(Clock::now() - time).count();
}

std::string base64_encode(const unsigned char* data, std::size_t length)
{
    static constexpr char alphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string encoded;
    encoded.reserve(((length + 2) / 3) * 4);

    for (std::size_t i = 0; i < length; i += 3) {
        std::uint32_t octet_a = data[i];
        std::uint32_t octet_b = i + 1 < length ? data[i + 1] : 0;
        std::uint32_t octet_c = i + 2 < length ? data[i + 2] : 0;
        std::uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        encoded.push_back(alphabet[(triple >> 18) & 0x3f]);
        encoded.push_back(alphabet[(triple >> 12) & 0x3f]);
        encoded.push_back(i + 1 < length ? alphabet[(triple >> 6) & 0x3f] : '=');
        encoded.push_back(i + 2 < length ? alphabet[triple & 0x3f] : '=');
    }

    return encoded;
}

std::uint32_t left_rotate(std::uint32_t value, int bits)
{
    return (value << bits) | (value >> (32 - bits));
}

std::array<unsigned char, 20> sha1_digest(const std::string& input)
{
    std::vector<unsigned char> data(input.begin(), input.end());
    std::uint64_t bit_length = static_cast<std::uint64_t>(data.size()) * 8;
    data.push_back(0x80);
    while ((data.size() % 64) != 56) data.push_back(0);
    for (int shift = 56; shift >= 0; shift -= 8) {
        data.push_back(static_cast<unsigned char>((bit_length >> shift) & 0xff));
    }

    std::uint32_t h0 = 0x67452301;
    std::uint32_t h1 = 0xEFCDAB89;
    std::uint32_t h2 = 0x98BADCFE;
    std::uint32_t h3 = 0x10325476;
    std::uint32_t h4 = 0xC3D2E1F0;

    for (std::size_t chunk = 0; chunk < data.size(); chunk += 64) {
        std::uint32_t w[80] = {};
        for (int i = 0; i < 16; i++) {
            std::size_t j = chunk + static_cast<std::size_t>(i) * 4;
            w[i] = (static_cast<std::uint32_t>(data[j]) << 24)
                 | (static_cast<std::uint32_t>(data[j + 1]) << 16)
                 | (static_cast<std::uint32_t>(data[j + 2]) << 8)
                 | static_cast<std::uint32_t>(data[j + 3]);
        }
        for (int i = 16; i < 80; i++) {
            w[i] = left_rotate(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }

        std::uint32_t a = h0;
        std::uint32_t b = h1;
        std::uint32_t c = h2;
        std::uint32_t d = h3;
        std::uint32_t e = h4;

        for (int i = 0; i < 80; i++) {
            std::uint32_t f = 0;
            std::uint32_t k = 0;
            if (i < 20) {
                f = (b & c) | ((~b) & d);
                k = 0x5A827999;
            } else if (i < 40) {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1;
            } else if (i < 60) {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDC;
            } else {
                f = b ^ c ^ d;
                k = 0xCA62C1D6;
            }
            std::uint32_t temp = left_rotate(a, 5) + f + e + k + w[i];
            e = d;
            d = c;
            c = left_rotate(b, 30);
            b = a;
            a = temp;
        }

        h0 += a;
        h1 += b;
        h2 += c;
        h3 += d;
        h4 += e;
    }

    std::array<unsigned char, 20> digest = {};
    std::uint32_t words[5] = {h0, h1, h2, h3, h4};
    for (int i = 0; i < 5; i++) {
        digest[i * 4] = static_cast<unsigned char>((words[i] >> 24) & 0xff);
        digest[i * 4 + 1] = static_cast<unsigned char>((words[i] >> 16) & 0xff);
        digest[i * 4 + 2] = static_cast<unsigned char>((words[i] >> 8) & 0xff);
        digest[i * 4 + 3] = static_cast<unsigned char>(words[i] & 0xff);
    }
    return digest;
}

std::string http_response(const std::string& body, const std::string& content_type)
{
    std::ostringstream out;
    out << "HTTP/1.1 200 OK\r\n"
        << "Content-Type: " << content_type << "\r\n"
        << "Content-Length: " << body.size() << "\r\n"
        << "Access-Control-Allow-Origin: *\r\n"
        << "Access-Control-Allow-Headers: Content-Type\r\n"
        << "Connection: close\r\n\r\n"
        << body;
    return out.str();
}

std::string http_error_response(int status_code,
                                const std::string& reason,
                                const std::string& body)
{
    std::ostringstream out;
    out << "HTTP/1.1 " << status_code << " " << reason << "\r\n"
        << "Content-Type: application/json\r\n"
        << "Content-Length: " << body.size() << "\r\n"
        << "Access-Control-Allow-Origin: *\r\n"
        << "Access-Control-Allow-Headers: Content-Type, X-Proton-Power-Key\r\n"
        << "Connection: close\r\n\r\n"
        << body;
    return out.str();
}

std::string no_content_response()
{
    return "HTTP/1.1 204 No Content\r\n"
           "Access-Control-Allow-Origin: *\r\n"
           "Access-Control-Allow-Headers: Content-Type, X-Proton-Power-Key\r\n"
           "Connection: close\r\n\r\n";
}

std::string not_found_response()
{
    return "HTTP/1.1 404 Not Found\r\n"
           "Content-Type: text/plain\r\n"
           "Content-Length: 10\r\n"
           "Connection: close\r\n\r\n"
           "Not found\n";
}

bool run_power_command(const std::string& action)
{
#ifdef _WIN32
    (void)action;
    return false;
#else
    const char* command = nullptr;
    if (action == "shutdown") {
        command = "/usr/bin/sudo -n /usr/sbin/shutdown -h now";
    } else if (action == "restart" || action == "reboot") {
        command = "/usr/bin/sudo -n /usr/sbin/reboot";
    } else {
        return false;
    }

    return std::system(command) == 0;
#endif
}

std::string random_power_key()
{
    static constexpr char hex[] = "0123456789abcdef";
    std::random_device random;
    std::string key;
    key.reserve(48);
    for (int i = 0; i < 24; i++) {
        const unsigned int value = random() & 0xff;
        key.push_back(hex[value >> 4]);
        key.push_back(hex[value & 0x0f]);
    }
    return key;
}

bool send_all(WifiSocket client, const std::string& data)
{
    const char* cursor = data.data();
    std::size_t remaining = data.size();
    while (remaining > 0) {
        const int chunk = static_cast<int>(
            std::min<std::size_t>(remaining, static_cast<std::size_t>(std::numeric_limits<int>::max()))
        );
#ifdef _WIN32
        const int sent = send(native_socket(client), cursor, chunk, 0);
#else
        const ssize_t sent = send(client, cursor, static_cast<std::size_t>(chunk), 0);
#endif
        if (sent <= 0) return false;
        cursor += sent;
        remaining -= static_cast<std::size_t>(sent);
    }
    return true;
}

bool recv_exact(WifiSocket client, void* data, std::size_t length)
{
    char* cursor = static_cast<char*>(data);
    std::size_t remaining = length;
    while (remaining > 0) {
        const int chunk = static_cast<int>(
            std::min<std::size_t>(remaining, static_cast<std::size_t>(std::numeric_limits<int>::max()))
        );
#ifdef _WIN32
        const int received = recv(native_socket(client), cursor, chunk, 0);
#else
        const ssize_t received = recv(client, cursor, static_cast<std::size_t>(chunk), 0);
#endif
        if (received <= 0) return false;
        cursor += received;
        remaining -= static_cast<std::size_t>(received);
    }
    return true;
}

std::string header_value(const std::string& request, const std::string& header)
{
    const auto position = request.find(header);
    if (position == std::string::npos) return "";

    const auto value_start = position + header.size();
    const auto value_end = request.find("\r\n", value_start);
    if (value_end == std::string::npos) return "";

    return request.substr(value_start, value_end - value_start);
}

std::string websocket_accept_key(const std::string& client_key)
{
    const std::string combined = client_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    const auto digest = sha1_digest(combined);
    return base64_encode(digest.data(), digest.size());
}

std::string websocket_handshake_response(const std::string& client_key)
{
    std::ostringstream handshake;
    handshake << "HTTP/1.1 101 Switching Protocols\r\n"
              << "Upgrade: websocket\r\n"
              << "Connection: Upgrade\r\n"
              << "Sec-WebSocket-Accept: " << websocket_accept_key(client_key) << "\r\n\r\n";
    return handshake.str();
}

std::string websocket_frame(const std::string& payload, unsigned char opcode = 0x1)
{
    std::string frame;
    frame.push_back(static_cast<char>(0x80 | opcode));

    if (payload.size() <= 125) {
        frame.push_back(static_cast<char>(payload.size()));
    } else if (payload.size() <= 65535) {
        frame.push_back(static_cast<char>(126));
        frame.push_back(static_cast<char>((payload.size() >> 8) & 0xff));
        frame.push_back(static_cast<char>(payload.size() & 0xff));
    } else {
        frame.push_back(static_cast<char>(127));
        for (int shift = 56; shift >= 0; shift -= 8) {
            frame.push_back(static_cast<char>((payload.size() >> shift) & 0xff));
        }
    }

    frame += payload;
    return frame;
}

bool receive_websocket_payload(WifiSocket client, std::string& payload)
{
    unsigned char header[2] = {};
    if (!recv_exact(client, header, sizeof(header))) return false;

    const unsigned char opcode = header[0] & 0x0f;
    const bool masked = (header[1] & 0x80) != 0;
    std::uint64_t payload_length = header[1] & 0x7f;

    if (payload_length == 126) {
        unsigned char length_bytes[2] = {};
        if (!recv_exact(client, length_bytes, sizeof(length_bytes))) return false;
        payload_length = (static_cast<std::uint64_t>(length_bytes[0]) << 8)
                       | static_cast<std::uint64_t>(length_bytes[1]);
    } else if (payload_length == 127) {
        unsigned char length_bytes[8] = {};
        if (!recv_exact(client, length_bytes, sizeof(length_bytes))) return false;
        payload_length = 0;
        for (unsigned char byte : length_bytes) {
            payload_length = (payload_length << 8) | static_cast<std::uint64_t>(byte);
        }
    }

    if (payload_length > 65536) return false;

    unsigned char mask[4] = {};
    if (masked && !recv_exact(client, mask, sizeof(mask))) return false;

    payload.assign(static_cast<std::size_t>(payload_length), '\0');
    if (payload_length > 0
        && !recv_exact(client, payload.data(), static_cast<std::size_t>(payload_length))) {
        return false;
    }

    if (masked) {
        for (std::size_t i = 0; i < payload.size(); i++) {
            payload[i] = static_cast<char>(payload[i] ^ mask[i % 4]);
        }
    }

    if (opcode == 0x8) return false;
    if (opcode == 0x9) {
        send_all(client, websocket_frame(payload, 0xA));
        payload.clear();
    }

    return true;
}

std::optional<double> number_after_key(const std::string& body,
                                       const std::string& key,
                                       std::size_t start)
{
    const auto key_position = json_key_position(body, key, start);
    if (!key_position) return std::nullopt;

    const auto colon_position = body.find(':', *key_position + key.size());
    if (colon_position == std::string::npos) return std::nullopt;

    const auto value_start = body.find_first_of("-0123456789.", colon_position + 1);
    if (value_start == std::string::npos) return std::nullopt;

    std::size_t value_end = value_start;
    while (value_end < body.size()
           && std::string("-0123456789.eE+").find(body[value_end]) != std::string::npos) {
        value_end++;
    }

    try {
        return std::stod(body.substr(value_start, value_end - value_start));
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<bool> bool_after_key(const std::string& body, const std::string& key)
{
    const auto key_position = json_key_position(body, key);
    if (!key_position) return std::nullopt;

    const auto colon_position = body.find(':', *key_position + key.size());
    if (colon_position == std::string::npos) return std::nullopt;

    const auto value_start = body.find_first_not_of(" \t\r\n", colon_position + 1);
    if (value_start == std::string::npos) return std::nullopt;

    if (body.compare(value_start, 4, "true") == 0) return true;
    if (body.compare(value_start, 5, "false") == 0) return false;
    if (body[value_start] == '1') return true;
    if (body[value_start] == '0') return false;
    return std::nullopt;
}

std::optional<std::string> string_after_key(const std::string& body, const std::string& key)
{
    const auto key_position = json_key_position(body, key);
    if (!key_position) return std::nullopt;

    const auto colon_position = body.find(':', *key_position + key.size());
    if (colon_position == std::string::npos) return std::nullopt;

    const auto value_start = body.find_first_not_of(" \t\r\n", colon_position + 1);
    if (value_start == std::string::npos || body[value_start] != '"') return std::nullopt;

    std::string value;
    for (std::size_t i = value_start + 1; i < body.size(); i++) {
        if (body[i] == '"') return value;
        if (body[i] == '\\' && i + 1 < body.size()) {
            value.push_back(body[++i]);
        } else {
            value.push_back(body[i]);
        }
    }
    return std::nullopt;
}

std::optional<WifiAxisAction> axis_action_after_keys(const std::string& body,
                                                     std::initializer_list<const char*> keys)
{
    for (const char* key : keys) {
        const auto value = string_after_key(body, key);
        if (!value) continue;
        return axis_action_from_string(*value);
    }
    return std::nullopt;
}

std::optional<int> integer_after_key(const std::string& body,
                                     const std::string& key,
                                     std::size_t start)
{
    const auto number = number_after_key(body, key, start);
    if (!number) return std::nullopt;
    return static_cast<int>(std::llround(*number));
}

std::optional<WifiJoystick> joystick_after_key(const std::string& body, const std::string& key)
{
    const auto joystick_position = json_key_position(body, key);
    if (!joystick_position) return std::nullopt;

    const auto x = number_after_key(body, "\"x\"", *joystick_position);
    const auto y = number_after_key(body, "\"y\"", *joystick_position);
    if (!x || !y) return std::nullopt;

    return WifiJoystick{clamp_axis(*x), clamp_axis(*y)};
}

struct ControlRequest {
    std::optional<WifiJoystick> primary;
    std::optional<WifiJoystick> secondary;
    std::optional<bool> relay;
    std::optional<double> body_height;
    std::optional<double> body_radius;
    std::optional<double> step_height;
    std::optional<double> speed;
    std::optional<int> position;
    std::optional<int> gait;
    std::optional<double> voltage;
    std::optional<double> current;
    std::optional<WifiAxisAction> primary_x_action;
    std::optional<WifiAxisAction> primary_y_action;
    std::optional<WifiAxisAction> secondary_x_action;
    std::optional<WifiAxisAction> secondary_y_action;
};

std::optional<ControlRequest> parse_control_request(const std::string& body)
{
    ControlRequest request;
    request.primary = joystick_after_key(body, "\"primary\"");
    request.secondary = joystick_after_key(body, "\"secondary\"");
    request.relay = first_present<bool>(
        bool_after_key(body, "\"relay_status\""),
        bool_after_key(body, "\"relayStatus\""),
        bool_after_key(body, "\"relay status\""),
        bool_after_key(body, "\"relay\""));
    request.body_height = first_present<double>(
        number_after_key(body, "\"body_height\"", 0),
        number_after_key(body, "\"bodyHeight\"", 0),
        number_after_key(body, "\"body height\"", 0),
        number_after_key(body, "\"height\"", 0));
    request.body_radius = first_present<double>(
        number_after_key(body, "\"body_radius\"", 0),
        number_after_key(body, "\"bodyRadius\"", 0),
        number_after_key(body, "\"body radius\"", 0));
    request.step_height = first_present<double>(
        number_after_key(body, "\"step_height\"", 0),
        number_after_key(body, "\"stepHeight\"", 0),
        number_after_key(body, "\"step height\"", 0));
    request.speed = number_after_key(body, "\"speed\"", 0);
    request.position = integer_after_key(body, "\"position\"", 0);
    request.gait = integer_after_key(body, "\"gait\"", 0);
    request.voltage = number_after_key(body, "\"voltage\"", 0);
    request.current = number_after_key(body, "\"current\"", 0);
    request.primary_x_action = axis_action_after_keys(
        body, {"\"primary_x\"", "\"primaryX\"", "\"Primary X\""});
    request.primary_y_action = axis_action_after_keys(
        body, {"\"primary_y\"", "\"primaryY\"", "\"Primary Y\""});
    request.secondary_x_action = axis_action_after_keys(
        body, {"\"secondary_x\"", "\"secondaryX\"", "\"Secondary X\"", "\"seconary_x\"", "\"Seconary X\""});
    request.secondary_y_action = axis_action_after_keys(
        body, {"\"secondary_y\"", "\"secondaryY\"", "\"Secondary Y\"", "\"seconary_y\"", "\"Seconary Y\""});

    if (!any_present(request.primary, request.secondary, request.relay,
                     request.body_height, request.body_radius, request.step_height,
                     request.speed, request.position, request.gait,
                     request.voltage, request.current,
                     request.primary_x_action, request.primary_y_action,
                     request.secondary_x_action, request.secondary_y_action)) {
        return std::nullopt;
    }
    return request;
}

std::string snapshot_json(const WifiControllerSnapshot& snapshot, const std::string& power_key = "")
{
    std::ostringstream body;
    body << "{"
         << "\"connected\":" << (snapshot.client_connected ? "true" : "false") << ","
         << "\"active\":" << (snapshot.active ? "true" : "false") << ","
         << "\"updates\":" << snapshot.update_count << ","
         << "\"relay_status\":" << (snapshot.relay_status ? 1 : 0) << ","
         << "\"height\":" << snapshot.height << ","
         << "\"height_min\":" << config::SitBodyHeight << ","
         << "\"height_max\":" << config::Motion.height_max << ","
         << "\"body_height\":" << snapshot.height << ","
         << "\"body_radius\":" << snapshot.body_radius << ","
         << "\"body_radius_min\":" << config::BodyRadiusMin << ","
         << "\"body_radius_max\":" << config::BodyRadiusMax << ","
         << "\"step_height\":" << snapshot.step_height << ","
         << "\"step_height_min\":" << config::StepHeightMin << ","
         << "\"step_height_max\":" << config::StepHeightMax << ","
         << "\"speed\":" << snapshot.speed << ","
         << "\"speed_min\":" << config::Motion.linear_speed_min << ","
         << "\"speed_max\":" << config::Motion.linear_speed_max << ","
         << "\"position\":" << snapshot.position << ","
         << "\"gait\":" << snapshot.gait << ","
         << "\"voltage\":" << snapshot.voltage << ","
         << "\"current\":" << snapshot.current << ","
         << "\"primary_x\":\"" << axis_action_name(snapshot.primary_x_action) << "\","
         << "\"primary_y\":\"" << axis_action_name(snapshot.primary_y_action) << "\","
         << "\"secondary_x\":\"" << axis_action_name(snapshot.secondary_x_action) << "\","
         << "\"secondary_y\":\"" << axis_action_name(snapshot.secondary_y_action) << "\","
         << "\"primary\":{\"x\":" << snapshot.primary.x << ",\"y\":" << snapshot.primary.y << "},"
         << "\"secondary\":{\"x\":" << snapshot.secondary.x << ",\"y\":" << snapshot.secondary.y << "}";
    if (!power_key.empty()) {
        body << ",\"power_key\":\"" << power_key << "\"";
    }
    body << "}";
    return body.str();
}

void append_vec3_json(std::ostringstream& body, const Vec3& point)
{
    body << "[" << point.x << "," << point.y << "," << point.z << "]";
}

std::string fixed_value(double value, int precision)
{
    std::ostringstream body;
    body.setf(std::ios::fixed);
    body.precision(precision);
    body << value;
    return body.str();
}

void append_json_string_array(std::ostringstream& body, const std::vector<std::string>& lines)
{
    body << "[";
    for (std::size_t i = 0; i < lines.size(); i++) {
        if (i > 0) body << ",";
        body << "\"" << json_escape(lines[i]) << "\"";
    }
    body << "]";
}

std::string visualizer_snapshot_json(const VisualizerSnapshot& snapshot)
{
    std::ostringstream body;
    body << "{"
         << "\"type\":\"visualizer\","
         << "\"available\":" << (snapshot.available ? "true" : "false");

    if (snapshot.available) {
        body << ",\"seq\":" << snapshot.sequence
             << ",\"time\":" << snapshot.time
             << ",\"pose\":{"
             << "\"x\":" << snapshot.pose.x << ","
             << "\"y\":" << snapshot.pose.y << ","
             << "\"z\":" << snapshot.pose.z << ","
             << "\"roll\":" << snapshot.pose.roll << ","
             << "\"pitch\":" << snapshot.pose.pitch << ","
             << "\"yaw\":" << snapshot.pose.yaw
             << "},"
             << "\"legs\":[";

        constexpr const char* leg_names[6] = {"R1", "R2", "R3", "L1", "L2", "L3"};
        for (int i = 0; i < 6; i++) {
            const VisualizerLegSnapshot& leg = snapshot.legs[i];
            if (i > 0) body << ",";
            body << "{"
                 << "\"id\":" << i << ","
                 << "\"name\":\"" << leg_names[i] << "\","
                 << "\"swing\":" << (leg.swing ? "true" : "false") << ","
                 << "\"joints\":[" << leg.joints.coxa << ","
                                      << leg.joints.femur << ","
                                      << leg.joints.tibia << "],"
                 << "\"pwm\":[" << leg.pwm.coxa << ","
                                 << leg.pwm.femur << ","
                                 << leg.pwm.tibia << "],"
                 << "\"points\":[";
            for (int point = 0; point < 5; point++) {
                if (point > 0) body << ",";
                append_vec3_json(body, leg.points[point]);
            }
            body << "],\"target\":";
            append_vec3_json(body, leg.target);
            body << "}";
        }
        body << "]";
    }

    body << "}";
    return body.str();
}

std::string logs_snapshot_json(const WifiControllerSnapshot& state,
                               const VisualizerSnapshot& visualizer,
                               const std::string& server_status)
{
    std::vector<std::string> hud_lines;
    std::vector<std::string> terminal_lines;

    hud_lines.push_back(std::string("Input   : Wi-Fi :") + std::to_string(state.port)
                        + (state.client_connected ? " connected" : " waiting"));
    hud_lines.push_back("Height  : " + fixed_value(state.height, 3) + " m");
    hud_lines.push_back("Stance r: " + fixed_value(state.body_radius, 3) + " m");
    hud_lines.push_back("Step hgt: " + fixed_value(state.step_height, 3) + " m");
    hud_lines.push_back("Speed   : " + fixed_value(state.speed, 3) + " m/s");
    hud_lines.push_back("Relay   : " + std::string(state.relay_status ? "on" : "off"));
    hud_lines.push_back("Position: " + std::string(state.position == 1 ? "standing" : "sitting"));
    hud_lines.push_back("Gait    : " + std::to_string(state.gait));
    hud_lines.push_back("Voltage : " + (state.voltage > 0.0 ? fixed_value(state.voltage, 2) + " V" : "--"));
    hud_lines.push_back("Current : " + (state.current > 0.0 ? fixed_value(state.current, 2) + " A" : "--"));

    if (visualizer.available) {
        hud_lines.push_back("Yaw     : " + fixed_value(visualizer.pose.yaw * RAD_TO_DEG, 1) + " deg");
        hud_lines.push_back("--- IK angles (deg, tibia bend) ---");
        constexpr const char* leg_names[6] = {"R1", "R2", "R3", "L1", "L2", "L3"};
        for (int i = 0; i < 6; i++) {
            const LegJoints& joints = visualizer.legs[i].joints;
            std::ostringstream line;
            line.setf(std::ios::fixed);
            line.precision(1);
            line << leg_names[i]
                 << "  cx=" << joints.coxa * RAD_TO_DEG
                 << "  fm=" << joints.femur * RAD_TO_DEG
                 << "  tb=" << -joints.tibia * RAD_TO_DEG
                 << (visualizer.legs[i].swing ? " ^" : "  ");
            hud_lines.push_back(line.str());
        }

        hud_lines.push_back("--- Servo angles (deg) ---");
        for (int i = 0; i < 6; i++) {
            ServoAngles servo = to_servo_angles(i, visualizer.legs[i].joints);
            std::ostringstream line;
            line.setf(std::ios::fixed);
            line.precision(1);
            line << leg_names[i]
                 << "  cx=" << servo.coxa
                 << "  fm=" << servo.femur
                 << "  tb=" << servo.tibia;
            hud_lines.push_back(line.str());
        }

        hud_lines.push_back("--- PWM values ---");
        for (int i = 0; i < 6; i++) {
            const PWMValues& pwm = visualizer.legs[i].pwm;
            std::ostringstream line;
            line << leg_names[i]
                 << "  cx=" << pwm.coxa
                 << "  fm=" << pwm.femur
                 << "  tb=" << pwm.tibia;
            hud_lines.push_back(line.str());
        }
    } else {
        hud_lines.push_back("HUD frame unavailable");
        hud_lines.push_back("Angle data unavailable until the first robot frame is produced");
    }

    terminal_lines.push_back("proton-server: " + server_status);
    terminal_lines.push_back("control clients: " + std::string(state.client_connected ? "connected" : "none"));
    terminal_lines.push_back("updates: " + std::to_string(state.update_count));
    if (!visualizer.available) {
        terminal_lines.push_back("WARNING: HUD/angle frame is not available; showing control status only");
    }
    if (state.voltage > 0.0 && state.voltage < config::Servo2040VoltageCritical) {
        terminal_lines.push_back("ERROR: servo voltage is below critical threshold ("
                                 + fixed_value(state.voltage, 2) + " V)");
    } else if (state.voltage > 0.0 && state.voltage < config::Servo2040VoltageWarn) {
        terminal_lines.push_back("WARNING: servo voltage is below warning threshold ("
                                 + fixed_value(state.voltage, 2) + " V)");
    }
    if (state.relay_control_active) {
        terminal_lines.push_back("relay request pending: "
                                 + std::string(state.target_relay_status ? "on" : "off"));
    }
    if (state.position_control_active) {
        terminal_lines.push_back("position request pending: "
                                 + std::to_string(state.target_position));
    }

    std::ostringstream body;
    body << "{"
         << "\"ok\":true,"
         << "\"hud\":";
    append_json_string_array(body, hud_lines);
    body << ",\"terminal\":";
    append_json_string_array(body, terminal_lines);
    body << "}";
    return body.str();
}

} // namespace

WifiControllerServer::~WifiControllerServer()
{
    stop();
}

bool WifiControllerServer::start(int port)
{
    if (running_) return true;

#ifndef _WIN32
    std::signal(SIGPIPE, SIG_IGN);
#endif

    if (!start_socket_runtime()) {
        status_ = "socket runtime failed";
        return false;
    }
    socket_runtime_started_ = true;

    const NativeSocket server_socket = socket(AF_INET, SOCK_STREAM, 0);
    server_fd_ = wifi_socket(server_socket);
    if (!valid_socket(server_fd_)) {
        status_ = std::string("socket failed: ") + socket_error_message();
        stop_socket_runtime();
        socket_runtime_started_ = false;
        return false;
    }

    int reuse = 1;
    setsockopt(native_socket(server_fd_), SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&reuse), sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(static_cast<uint16_t>(port));

    if (bind(native_socket(server_fd_), reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        status_ = std::string("bind failed: ") + socket_error_message();
        close_wifi_socket(server_fd_);
        server_fd_ = InvalidWifiSocket;
        stop_socket_runtime();
        socket_runtime_started_ = false;
        return false;
    }

    if (listen(native_socket(server_fd_), 8) < 0) {
        status_ = std::string("listen failed: ") + socket_error_message();
        close_wifi_socket(server_fd_);
        server_fd_ = InvalidWifiSocket;
        stop_socket_runtime();
        socket_runtime_started_ = false;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = {};
        state_.server_enabled = true;
        state_.port = port;
        state_.height = config::Motion.start_height;
        state_.target_height = config::Motion.start_height;
        state_.user_target_height = config::Motion.start_height;
        state_.body_radius = config::Motion.body_radius;
        state_.target_body_radius = config::Motion.body_radius;
        state_.step_height = config::Motion.step_height;
        state_.target_step_height = config::Motion.step_height;
        state_.speed = default_speed();
        state_.target_speed = state_.speed;
        state_.position = 1;
        state_.target_position = 1;
        state_.relay_status = true;
        state_.target_relay_status = true;
        state_.gait = 1;
        state_.target_gait = 1;
    }
    running_ = true;
    start_mdns(port);
    status_ = mdns_responder_.running() ? "listening (mDNS advertised)" : "listening";
    server_thread_ = std::thread(&WifiControllerServer::serve_loop, this);
    return true;
}

void WifiControllerServer::stop()
{
    running_ = false;
    stop_mdns();
    std::vector<WifiSocket> visualizer_clients;
    {
        std::lock_guard<std::mutex> lock(visualizer_mutex_);
        visualizer_clients = visualizer_clients_;
        visualizer_clients_.clear();
    }
    for (WifiSocket client : visualizer_clients) {
        shutdown_wifi_socket(client);
        close_wifi_socket(client);
    }

    if (valid_socket(server_fd_)) {
        shutdown_wifi_socket(server_fd_);
        close_wifi_socket(server_fd_);
        server_fd_ = InvalidWifiSocket;
    }
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    if (socket_runtime_started_) {
        stop_socket_runtime();
        socket_runtime_started_ = false;
    }
    status_ = "stopped";
}

void WifiControllerServer::start_mdns(int port)
{
    if (!mdns_responder_.start(port)) {
        std::fprintf(stderr, "Wi-Fi controller: mDNS advertise failed\n");
    }
}

void WifiControllerServer::stop_mdns()
{
    mdns_responder_.stop();
}

WifiControllerSnapshot WifiControllerServer::snapshot() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    WifiControllerSnapshot snapshot = state_;
    snapshot.seconds_since_update = seconds_since(last_update_time_);
    const bool websocket_connected = websocket_clients_.load() > 0;
    snapshot.client_connected = websocket_connected
        || snapshot.seconds_since_update <= CONNECTED_TIMEOUT_SECONDS;
    const bool recent_control = snapshot.client_connected
        && snapshot.seconds_since_update <= ACTIVE_TIMEOUT_SECONDS;
    snapshot.active = control_active(snapshot)
        && (websocket_connected
            || recent_control
            || snapshot.height_control_active
            || snapshot.position_control_active
            || snapshot.gait_control_active
            || snapshot.relay_control_active);
    return snapshot;
}

void WifiControllerServer::update_robot_status(double height, int position, int gait)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.height = std::max(0.0, height);
    const int reported_position = std::clamp(position, 0, 1);
    if (state_.position_control_active) {
        if (state_.target_position == 1 && reported_position == 1) {
            state_.position = 1;
            state_.target_position = 1;
            state_.position_control_active = false;
        }
    }
    state_.gait = std::clamp(gait, 1, 4);

    if (state_.height_control_active
        && std::fabs(state_.height - state_.target_height) <= HEIGHT_TARGET_TOLERANCE) {
        state_.height_control_active = false;
        state_.height_control_gentle = false;
        if (state_.relay_control_active
            && !state_.target_relay_status
            && state_.target_height <= config::SitBodyHeight + HEIGHT_TARGET_TOLERANCE
            && !(state_.position_control_active && state_.target_position == 0)) {
            state_.relay_switch_ready = true;
        }
    }
    if (state_.gait_control_active && state_.gait == state_.target_gait) {
        state_.gait_control_active = false;
    }
}

void WifiControllerServer::update_motion_status(double body_radius, double step_height, double speed)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.body_radius = clamp_body_radius(body_radius);
    state_.step_height = clamp_step_height(step_height);
    state_.speed = clamp_speed(speed);

    if (state_.body_radius_control_active
        && std::fabs(state_.body_radius - state_.target_body_radius) <= MOTION_TARGET_TOLERANCE) {
        state_.body_radius_control_active = false;
    }
    if (state_.step_height_control_active
        && std::fabs(state_.step_height - state_.target_step_height) <= MOTION_TARGET_TOLERANCE) {
        state_.step_height_control_active = false;
    }
    if (state_.speed_control_active
        && std::fabs(state_.speed - state_.target_speed) <= MOTION_TARGET_TOLERANCE) {
        state_.speed_control_active = false;
    }
}

void WifiControllerServer::update_shutdown_complete(bool complete)
{
    if (!complete) return;

    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.height_control_active = false;
    if (state_.position_control_active && state_.target_position == 0) {
        state_.position_control_active = false;
        state_.position = 0;
        state_.target_position = 0;
    }
    if (state_.relay_control_active && !state_.target_relay_status) {
        state_.relay_switch_ready = true;
    }
}

void WifiControllerServer::request_relay_status(bool enabled, bool restore_default_height)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.target_relay_status = enabled;
    state_.relay_control_active = true;
    state_.relay_restore_default_height = false;
    if (enabled) {
        state_.relay_switch_ready = true;
    } else {
        state_.relay_switch_ready = false;
        state_.relay_restore_default_height = false;
        state_.relay_asleep_from_inactivity = !restore_default_height;
        state_.target_height = config::SitBodyHeight;
        state_.height_control_active = true;
        state_.height_control_gentle = true;
        state_.primary = {};
        state_.secondary = {};
    }
    state_.update_count++;
    last_update_time_ = Clock::now();
}

void WifiControllerServer::update_relay_status(bool enabled, bool complete_request)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.relay_status = enabled;
    if (!complete_request) {
        return;
    }

    state_.target_relay_status = enabled;
    state_.relay_control_active = false;
    state_.relay_switch_ready = false;
    if (enabled) {
        state_.target_height = state_.user_target_height;
        state_.height_control_active = true;
        state_.height_control_gentle = true;
    }
    state_.relay_restore_default_height = false;
    if (enabled) {
        state_.relay_asleep_from_inactivity = false;
    }
}

void WifiControllerServer::update_voltage(double voltage)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.voltage = std::max(0.0, voltage);
}

void WifiControllerServer::update_current(double current)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.current = current;
}

void WifiControllerServer::update_visualizer_frame(const BasePose& pose,
                                                   const LegPoints fk_points[6],
                                                   const Vec3 feet_world[6],
                                                   const bool is_swing[6],
                                                   const RobotState& render_state,
                                                   const std::array<PWMValues, 6>& render_pwm)
{
    std::vector<WifiSocket> clients;
    std::string payload;

    {
        std::lock_guard<std::mutex> lock(visualizer_mutex_);
        visualizer_.available = true;
        visualizer_.sequence++;
        visualizer_.time = std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
        visualizer_.pose = pose;
        for (int i = 0; i < 6; i++) {
            visualizer_.legs[i].joints = render_state.legs[i];
            visualizer_.legs[i].pwm = render_pwm[i];
            visualizer_.legs[i].target = feet_world[i];
            visualizer_.legs[i].swing = is_swing[i];
            for (int point = 0; point < 5; point++) {
                visualizer_.legs[i].points[point] = fk_points[i].pts[point];
            }
        }
        payload = visualizer_snapshot_json(visualizer_);
        clients = visualizer_clients_;
    }

    const std::string frame = websocket_frame(payload);
    for (WifiSocket client : clients) {
        if (!send_all(client, frame)) {
            if (remove_visualizer_client(client)) {
                close_wifi_socket(client);
            }
        }
    }
}

std::string WifiControllerServer::visualizer_json() const
{
    std::lock_guard<std::mutex> lock(visualizer_mutex_);
    return visualizer_snapshot_json(visualizer_);
}

std::string WifiControllerServer::logs_json() const
{
    WifiControllerSnapshot state_snapshot;
    VisualizerSnapshot visualizer_snapshot;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_snapshot = state_;
        state_snapshot.seconds_since_update = seconds_since(last_update_time_);
        const bool websocket_connected = websocket_clients_.load() > 0;
        state_snapshot.client_connected = websocket_connected
            || state_snapshot.seconds_since_update <= CONNECTED_TIMEOUT_SECONDS;
        const bool recent_control = state_snapshot.client_connected
            && state_snapshot.seconds_since_update <= ACTIVE_TIMEOUT_SECONDS;
        state_snapshot.active = control_active(state_snapshot)
            && (websocket_connected
                || recent_control
                || state_snapshot.height_control_active
                || state_snapshot.position_control_active
                || state_snapshot.gait_control_active
                || state_snapshot.relay_control_active);
    }
    {
        std::lock_guard<std::mutex> lock(visualizer_mutex_);
        visualizer_snapshot = visualizer_;
    }

    return logs_snapshot_json(snapshot(), visualizer_snapshot, status_);
}

void WifiControllerServer::serve_loop()
{
    while (running_) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(native_socket(server_fd_), &read_fds);

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 200000;

        int ready = select(select_width(server_fd_), &read_fds, nullptr, nullptr, &timeout);
        if (ready <= 0) continue;

        sockaddr_in client_address{};
        SocketLength client_length = sizeof(client_address);
        NativeSocket client = accept(native_socket(server_fd_),
                                     reinterpret_cast<sockaddr*>(&client_address),
                                     &client_length);
#ifdef _WIN32
        if (client == INVALID_SOCKET) continue;
#else
        if (client < 0) continue;
#endif

        std::thread(&WifiControllerServer::handle_client, this, wifi_socket(client)).detach();
    }
}

void WifiControllerServer::handle_client(WifiSocket client)
{
    char buffer[8192] = {};
#ifdef _WIN32
    const int received = recv(native_socket(client), buffer, static_cast<int>(sizeof(buffer) - 1), 0);
#else
    ssize_t received = recv(client, buffer, sizeof(buffer) - 1, 0);
#endif
    if (received <= 0) {
        close_wifi_socket(client);
        return;
    }

    std::string request(buffer, static_cast<std::size_t>(received));
    if (request.rfind("OPTIONS ", 0) == 0) {
        send_all(client, no_content_response());
    } else if ((request.rfind("GET /visualizer/ws", 0) == 0
                || request.rfind("GET /visualizer", 0) == 0)
               && request.find("Upgrade: websocket") != std::string::npos) {
        handle_visualizer_websocket(client, request);
        return;
    } else if ((request.rfind("GET /control/ws", 0) == 0
                || request.rfind("GET /ws", 0) == 0)
               && request.find("Upgrade: websocket") != std::string::npos) {
        handle_websocket(client, request);
        return;
    } else if (request.rfind("GET /visualizer.json", 0) == 0) {
        send_all(client, http_response(visualizer_json(), "application/json"));
    } else if (request.rfind("GET /logs.json", 0) == 0) {
        send_all(client, http_response(logs_json(), "application/json"));
    } else if (request.rfind("POST /system/shutdown", 0) == 0) {
        send_all(client, handle_system_power_request(request, "shutdown"));
    } else if (request.rfind("POST /system/restart", 0) == 0) {
        send_all(client, handle_system_power_request(request, "restart"));
    } else if (request.rfind("POST /system/reboot", 0) == 0) {
        send_all(client, handle_system_power_request(request, "reboot"));
    } else if (request.rfind("GET /control.json", 0) == 0
               || request.rfind("GET /status.json", 0) == 0) {
        send_all(client, http_response(snapshot_json(snapshot()), "application/json"));
    } else if (request.rfind("GET / ", 0) == 0 || request.rfind("GET /?", 0) == 0) {
        send_all(client, http_response(snapshot_json(snapshot()), "application/json"));
    } else {
        send_all(client, not_found_response());
    }

    close_wifi_socket(client);
}

std::string WifiControllerServer::handle_system_power_request(const std::string& request,
                                                              const std::string& action)
{
    const std::string power_key = header_value(request, "X-Proton-Power-Key: ");
    if (power_key.empty() || !power_key_is_active(power_key)) {
        return http_error_response(
            403,
            "Forbidden",
            "{\"ok\":false,\"error\":\"Power requests require an active control WebSocket key.\"}"
        );
    }

    if (!run_power_command(action)) {
        return http_error_response(
            403,
            "Forbidden",
            "{\"ok\":false,\"error\":\"System power command was not permitted. Re-run scripts/install_raspbian.sh so sudoers allows proton-server to shutdown or reboot.\"}"
        );
    }

    return http_response("{\"ok\":true}", "application/json");
}

void WifiControllerServer::handle_visualizer_websocket(WifiSocket client, const std::string& request)
{
    const std::string client_key = header_value(request, "Sec-WebSocket-Key: ");
    if (client_key.empty()) {
        close_wifi_socket(client);
        return;
    }

    if (!send_all(client, websocket_handshake_response(client_key))) {
        close_wifi_socket(client);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(visualizer_mutex_);
        visualizer_clients_.push_back(client);
        send_all(client, websocket_frame(visualizer_snapshot_json(visualizer_)));
    }

    std::string payload;
    while (running_ && receive_websocket_payload(client, payload)) {
        payload.clear();
    }

    if (remove_visualizer_client(client)) {
        close_wifi_socket(client);
    }
}

void WifiControllerServer::handle_websocket(WifiSocket client, const std::string& request)
{
    const std::string client_key = header_value(request, "Sec-WebSocket-Key: ");
    if (client_key.empty()) {
        close_wifi_socket(client);
        return;
    }

    if (!send_all(client, websocket_handshake_response(client_key))) {
        close_wifi_socket(client);
        return;
    }

    const std::string power_key = register_power_key();
    websocket_clients_.fetch_add(1);
    send_all(client, websocket_frame(snapshot_json(snapshot(), power_key)));

    std::string payload;
    while (running_ && receive_websocket_payload(client, payload)) {
        if (payload.empty()) continue;

        bool ok = update_coordinates(payload);
        send_all(client, websocket_frame(ok ? snapshot_json(snapshot())
                                            : "{\"ok\":false,\"error\":\"Invalid coordinates\"}"));
    }

    websocket_clients_.fetch_sub(1);
    remove_power_key(power_key);
    close_wifi_socket(client);
}

bool WifiControllerServer::remove_visualizer_client(WifiSocket client)
{
    std::lock_guard<std::mutex> lock(visualizer_mutex_);
    auto it = std::find(visualizer_clients_.begin(), visualizer_clients_.end(), client);
    if (it == visualizer_clients_.end()) return false;
    visualizer_clients_.erase(it);
    return true;
}

std::string WifiControllerServer::register_power_key()
{
    std::string key;
    {
        std::lock_guard<std::mutex> lock(power_key_mutex_);
        do {
            key = random_power_key();
        } while (std::find(active_power_keys_.begin(), active_power_keys_.end(), key) != active_power_keys_.end());
        active_power_keys_.push_back(key);
    }
    return key;
}

void WifiControllerServer::remove_power_key(const std::string& key)
{
    std::lock_guard<std::mutex> lock(power_key_mutex_);
    auto it = std::find(active_power_keys_.begin(), active_power_keys_.end(), key);
    if (it != active_power_keys_.end()) {
        active_power_keys_.erase(it);
    }
}

bool WifiControllerServer::power_key_is_active(const std::string& key) const
{
    std::lock_guard<std::mutex> lock(power_key_mutex_);
    return std::find(active_power_keys_.begin(), active_power_keys_.end(), key) != active_power_keys_.end();
}

bool WifiControllerServer::update_coordinates(const std::string& body)
{
    const auto request = parse_control_request(body);
    if (!request) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        bool changed = false;
        if (request->relay) {
            if (*request->relay) {
                state_.target_relay_status = true;
                state_.relay_control_active = true;
                state_.relay_switch_ready = true;
                state_.relay_restore_default_height = false;
                changed = true;
            } else {
                state_.target_relay_status = false;
                state_.relay_control_active = true;
                state_.relay_switch_ready = false;
                state_.relay_restore_default_height = false;
                state_.relay_asleep_from_inactivity = false;
                state_.target_height = config::SitBodyHeight;
                state_.height_control_active = true;
                state_.height_control_gentle = true;
                state_.primary = {};
                state_.secondary = {};
                changed = true;
            }
        }
        const bool directional_control_enabled =
            state_.relay_status && state_.target_relay_status;
        if (directional_control_enabled) {
            if (request->primary) {
                state_.primary = *request->primary;
                changed = true;
            }
            if (request->secondary) {
                state_.secondary = *request->secondary;
                changed = true;
            }
        } else if ((request->primary || request->secondary)
                   && (joystick_active(state_.primary)
                       || joystick_active(state_.secondary))) {
            state_.primary = {};
            state_.secondary = {};
            changed = true;
        }
        if (request->primary_x_action) {
            state_.primary_x_action = *request->primary_x_action;
            changed = true;
        }
        if (request->primary_y_action) {
            state_.primary_y_action = *request->primary_y_action;
            changed = true;
        }
        if (request->secondary_x_action) {
            state_.secondary_x_action = *request->secondary_x_action;
            changed = true;
        }
        if (request->secondary_y_action) {
            state_.secondary_y_action = *request->secondary_y_action;
            changed = true;
        }
        auto height_axis_value = [](WifiAxisAction action, double value) {
            return action == WifiAxisAction::HEIGHT && std::fabs(value) > ACTIVE_DEADZONE;
        };
        const bool height_axis_requested =
            (request->primary
             && (height_axis_value(state_.primary_x_action, request->primary->x)
                 || height_axis_value(state_.primary_y_action, request->primary->y)))
            || (request->secondary
                && (height_axis_value(state_.secondary_x_action, request->secondary->x)
                    || height_axis_value(state_.secondary_y_action, request->secondary->y)));
        if (height_axis_requested && !directional_control_enabled) {
            state_.primary = {};
            state_.secondary = {};
            if (request->primary) {
                if (state_.primary_x_action == WifiAxisAction::HEIGHT) state_.primary.x = request->primary->x;
                if (state_.primary_y_action == WifiAxisAction::HEIGHT) state_.primary.y = request->primary->y;
            }
            if (request->secondary) {
                if (state_.secondary_x_action == WifiAxisAction::HEIGHT) state_.secondary.x = request->secondary->x;
                if (state_.secondary_y_action == WifiAxisAction::HEIGHT) state_.secondary.y = request->secondary->y;
            }
            changed = true;
        }
        if (request->body_height && !height_axis_mapped(state_)) {
            state_.user_target_height =
                std::clamp(*request->body_height, config::SitBodyHeight, config::Motion.height_max);
            state_.target_height = state_.user_target_height;
            state_.height_control_active = true;
            state_.height_control_gentle = false;
            changed = true;
        }
        if (request->body_radius) {
            state_.target_body_radius = clamp_body_radius(*request->body_radius);
            state_.body_radius = state_.target_body_radius;
            state_.body_radius_control_active = true;
            changed = true;
        }
        if (request->step_height) {
            state_.target_step_height = clamp_step_height(*request->step_height);
            state_.step_height = state_.target_step_height;
            state_.step_height_control_active = true;
            changed = true;
        }
        if (request->speed) {
            state_.target_speed = clamp_speed(*request->speed);
            state_.speed = state_.target_speed;
            state_.speed_control_active = true;
            changed = true;
        }
        if (request->position) {
            const int clamped_position = *request->position <= 0 ? 0 : 1;
            const bool shutdown_in_progress =
                state_.position_control_active && state_.target_position == 0;
            if (!(shutdown_in_progress && clamped_position == 1)) {
                state_.target_position = clamped_position;
                state_.position_control_active = true;
                if (clamped_position == 0) {
                    state_.target_height = config::SitBodyHeight;
                } else {
                    state_.target_height = config::Motion.start_height;
                    state_.height_control_active = true;
                    state_.height_control_gentle = false;
                }
                changed = true;
            }
        }
        if (request->gait) {
            const int clamped_gait = std::clamp(*request->gait, 1, 4);
            state_.target_gait = clamped_gait;
            state_.gait = clamped_gait;
            state_.gait_control_active = true;
            changed = true;
        }
        if (request->voltage) {
            state_.voltage = std::max(0.0, *request->voltage);
            changed = true;
        }
        if (request->current) {
            state_.current = std::max(0.0, *request->current);
            changed = true;
        }
        if (changed) {
            state_.update_count++;
            last_update_time_ = Clock::now();
        }
    }
    return true;
}
