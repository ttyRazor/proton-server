#!/bin/sh
set -eu

PROTON_CONF="/etc/proton-server/proton.conf"
HOSTAPD_CONF="/etc/hostapd/proton-server.conf"
DNSMASQ_CONF="/etc/dnsmasq.d/proton-server.conf"
HOTSPOT_RUNNER="/usr/local/sbin/proton-hotspot-run"
HOTSPOT_SERVICE="/etc/systemd/system/proton-hotspot.service"
NETWORKMANAGER_CONF="/etc/NetworkManager/conf.d/proton-server-unmanaged.conf"

SERVICE_NAME="proton-server"
BRANCH="headless"
BRANCH_SET=0
APP_ARGS="--config ${PROTON_CONF}}"
SERVICE_USER="${SUDO_USER:-$(id -un)}"
START_SERVICE=1
VERBOSE=0

HOTSPOT_SSID="Proton Server"
AP_IFACE="uap0"
WIFI_IFACE="wlan0"
HOTSPOT_CHANNEL="7"

usage() {
    cat <<EOF
Usage: $0 [main|headless] [options]

Installs or updates Raspberry Pi OS / Raspbian dependencies, checks out the
requested branch, builds Proton Server, creates the Proton Server hotspot, and
enables startup services. Re-run this script to update older installs.

Options:
  --servo2040 PORT              Pass a Servo2040 serial port to the service.
  --servo2040-port PORT         Same as --servo2040.
  --pwm-control                 Start in direct PWM control mode.
  --pwm-control-servo2040 PORT  Start direct PWM control with Servo2040 output.
  --servo2040-pwm-sim           Mirror Servo2040 PWM packets into simulation.
  --pwm-sim                     Same as --servo2040-pwm-sim.
  --port PORT                   Wi-Fi controller port, default 8080.
  --service-user USER           Run the startup service as USER.
  --hotspot-ssid NAME           Hotspot SSID, default "Proton Server".
  --wifi-iface IFACE            Wi-Fi interface used to create the hotspot, default wlan0.
  --ap-iface IFACE              Hotspot virtual AP interface, default uap0.
  --hotspot-channel CHANNEL     Hotspot channel, default 7.
  --no-start                    Enable at boot, but do not start now.
  -v, -verbose, --verbose       Show command output.
  -h, --help                    Show this help.

Examples:
  $0 headless --servo2040 /dev/ttyACM0
  $0 main --port 8081 -v
EOF
}

banner() {
    cat <<'EOF'
                     __
_____________  _____/  |_  ____   ____
\____ \_  __ \/  _ \   __\/  _ \ /    \
|  |_> >  | \(  <_> )  | (  <_> )   |  \
|   __/|__|   \____/|__|  \____/|___|  /
|__|                                 \/
EOF
    echo
}

section() {
    echo
    echo "==> $*"
}

die() {
    echo "install_raspbian.sh: $*" >&2
    exit 1
}

append_arg() {
    case "$1" in
        *" "*) die "arguments with spaces are not supported: $1" ;;
    esac
    if [ -z "$APP_ARGS" ]; then
        APP_ARGS="$1"
    else
        APP_ARGS="$APP_ARGS $1"
    fi
}

run() {
    if [ "$VERBOSE" -eq 1 ]; then
        "$@"
        return
    fi

    LOG_FILE=$(mktemp "${TMPDIR:-/tmp}/proton-install.XXXXXX")
    if "$@" >"$LOG_FILE" 2>&1; then
        rm -f "$LOG_FILE"
    else
        STATUS=$?
        echo "Command failed: $*" >&2
        echo "Run again with -v for full output, or inspect: $LOG_FILE" >&2
        echo "Last output:" >&2
        tail -40 "$LOG_FILE" >&2 || true
        exit "$STATUS"
    fi
}

run_optional() {
    if [ "$VERBOSE" -eq 1 ]; then
        "$@" || true
    else
        "$@" >/dev/null 2>&1 || true
    fi
}

run_repo() {
    if [ "$(id -u)" -eq 0 ] && [ "$SERVICE_USER" != "root" ]; then
        run sudo -u "$SERVICE_USER" "$@"
    else
        run "$@"
    fi
}

repo_command_succeeds() {
    if [ "$(id -u)" -eq 0 ] && [ "$SERVICE_USER" != "root" ]; then
        sudo -u "$SERVICE_USER" "$@" >/dev/null 2>&1
    else
        "$@" >/dev/null 2>&1
    fi
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        main|headless)
            [ "$BRANCH_SET" -eq 0 ] || die "branch was already set to $BRANCH"
            BRANCH="$1"
            BRANCH_SET=1
            shift
            ;;
        --servo2040|--servo2040-port|--pwm-control-servo2040|--port)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            append_arg "$1"
            append_arg "$2"
            shift 2
            ;;
        --pwm-control|--servo2040-pwm-sim|--pwm-sim)
            append_arg "$1"
            shift
            ;;
        --service-user)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            SERVICE_USER="$2"
            shift 2
            ;;
        --hotspot-ssid)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            HOTSPOT_SSID="$2"
            shift 2
            ;;
        --wifi-iface)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            WIFI_IFACE="$2"
            shift 2
            ;;
        --ap-iface)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            AP_IFACE="$2"
            shift 2
            ;;
        --hotspot-channel)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            HOTSPOT_CHANNEL="$2"
            shift 2
            ;;
        --no-start)
            START_SERVICE=0
            shift
            ;;
        -v|-verbose|--verbose)
            VERBOSE=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            die "unknown option: $1"
            ;;
    esac
done

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
BUILD_DIR="$REPO_DIR/build"
if [ "$BRANCH" = "headless" ]; then
    BUILD_DIR="$REPO_DIR/build-headless"
fi

if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

write_hotspot_config() {
    COUNTRY_CODE=$(iw reg get 2>/dev/null | awk '/country/ { print substr($2, 1, 2); exit }' || true)
    if [ -z "$COUNTRY_CODE" ] || [ "$COUNTRY_CODE" = "00" ]; then
        COUNTRY_CODE="US"
    fi

    run $SUDO mkdir -p /etc/hostapd /etc/dnsmasq.d /etc/proton-server /etc/NetworkManager/conf.d /usr/local/sbin

    {
        echo "[keyfile]"
        echo "unmanaged-devices=interface-name:$AP_IFACE"
    } | $SUDO tee "$NETWORKMANAGER_CONF" >/dev/null

    {
        echo "interface=$AP_IFACE"
        echo "driver=nl80211"
        echo "ssid=$HOTSPOT_SSID"
        echo "country_code=$COUNTRY_CODE"
        echo "ieee80211d=1"
        echo "hw_mode=g"
        echo "channel=$HOTSPOT_CHANNEL"
        echo "wmm_enabled=1"
        echo "auth_algs=1"
        echo "ignore_broadcast_ssid=0"
        echo "ctrl_interface=/run/hostapd"
        echo "ctrl_interface_group=0"
    } | $SUDO tee "$HOSTAPD_CONF" >/dev/null

    {
        echo "interface=$AP_IFACE"
        echo "bind-interfaces"
        echo "domain-needed"
        echo "bogus-priv"
        echo "dhcp-range=10.42.0.10,10.42.0.200,255.255.255.0,12h"
        echo "dhcp-option=3,10.42.0.1"
        echo "dhcp-option=6,1.1.1.1,8.8.8.8"
    } | $SUDO tee "$DNSMASQ_CONF" >/dev/null

    {
        echo "#!/bin/sh"
        echo "set -u"
        echo
        echo "AP_IFACE=$AP_IFACE"
        echo "WIFI_IFACE=$WIFI_IFACE"
        echo "HOSTAPD_CONF=$HOSTAPD_CONF"
        echo "DNSMASQ_CONF=$DNSMASQ_CONF"
        echo "DNSMASQ_CHILD="
        echo "HOSTAPD_CHILD="
        echo
        echo "cleanup() {"
        echo "    if [ -n \"\$HOSTAPD_CHILD\" ]; then kill \"\$HOSTAPD_CHILD\" >/dev/null 2>&1 || true; fi"
        echo "    if [ -n \"\$DNSMASQ_CHILD\" ]; then kill \"\$DNSMASQ_CHILD\" >/dev/null 2>&1 || true; fi"
        echo "    ip link set \"\$AP_IFACE\" down >/dev/null 2>&1 || true"
        echo "    iw dev \"\$AP_IFACE\" del >/dev/null 2>&1 || true"
        echo "}"
        echo
        echo "fail() {"
        echo "    echo \"proton-hotspot-run: \$*\" >&2"
        echo "    exit 1"
        echo "}"
        echo
        echo "trap cleanup EXIT INT TERM"
        echo "cleanup"
        echo "rfkill unblock wifi >/dev/null 2>&1 || true"
        echo
        echo "found_wifi=0"
        echo "for _ in 1 2 3 4 5 6 7 8 9 10; do"
        echo "    if iw dev \"\$WIFI_IFACE\" info >/dev/null 2>&1; then"
        echo "        found_wifi=1"
        echo "        break"
        echo "    fi"
        echo "    sleep 1"
        echo "done"
        echo "[ \"\$found_wifi\" -eq 1 ] || fail \"Wi-Fi interface \$WIFI_IFACE was not found\""
        echo
        echo "iw dev \"\$WIFI_IFACE\" interface add \"\$AP_IFACE\" type __ap \\"
        echo "    || fail \"could not create \$AP_IFACE from \$WIFI_IFACE; this Wi-Fi adapter may not support AP mode\""
        echo "ip addr flush dev \"\$AP_IFACE\" >/dev/null 2>&1 || true"
        echo "ip addr add 10.42.0.1/24 dev \"\$AP_IFACE\" || fail \"could not assign hotspot address\""
        echo "ip link set \"\$AP_IFACE\" up || fail \"could not bring up \$AP_IFACE\""
        echo
        echo "dnsmasq --no-daemon --conf-file=\"\$DNSMASQ_CONF\" &"
        echo "DNSMASQ_CHILD=\$!"
        echo "hostapd \"\$HOSTAPD_CONF\" &"
        echo "HOSTAPD_CHILD=\$!"
        echo "wait \"\$HOSTAPD_CHILD\""
    } | $SUDO tee "$HOTSPOT_RUNNER" >/dev/null
    run $SUDO chmod 755 "$HOTSPOT_RUNNER"

    {
        echo "[Unit]"
        echo "Description=Proton Server Wi-Fi hotspot"
        echo "After=network.target"
        echo "Wants=network.target"
        echo "Before=$SERVICE_NAME.service"
        echo
        echo "[Service]"
        echo "Type=simple"
        echo "ExecStart=$HOTSPOT_RUNNER"
        echo "Restart=on-failure"
        echo "RestartSec=3"
        echo
        echo "[Install]"
        echo "WantedBy=multi-user.target"
    } | $SUDO tee "$HOTSPOT_SERVICE" >/dev/null

}

write_server_service() {
    SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"
    {
        echo "[Unit]"
        echo "Description=Proton server ($BRANCH)"
        echo "After=network-online.target proton-hotspot.service"
        echo "Wants=network-online.target proton-hotspot.service"
        echo
        echo "[Service]"
        echo "Type=simple"
        echo "User=$SERVICE_USER"
        echo "WorkingDirectory=$REPO_DIR"
        echo "ExecStart=$EXECUTABLE $APP_ARGS"
        echo "Restart=on-failure"
        echo "RestartSec=3"
        echo "Environment=HEXAPOD_SIM_BRANCH=$BRANCH"
        if [ "$BRANCH" = "main" ]; then
            echo "Environment=DISPLAY=:0"
            echo "Environment=XAUTHORITY=/home/$SERVICE_USER/.Xauthority"
        fi
        echo
        echo "[Install]"
        echo "WantedBy=multi-user.target"
    } | $SUDO tee "$SERVICE_FILE" >/dev/null
    cp $REPO_DIR/proton.conf "$PROTON_CONF"
}

cd "$REPO_DIR"
banner

section "Checking install target"
[ -d .git ] || die "run this from inside a cloned proton-server git repo"
id "$SERVICE_USER" >/dev/null 2>&1 || die "service user does not exist: $SERVICE_USER"
echo "Branch: $BRANCH"
echo "Service user: $SERVICE_USER"
echo "Hotspot: $HOTSPOT_SSID on $AP_IFACE from $WIFI_IFACE"

section "Installing dependencies"
run $SUDO apt-get update
PACKAGES="ca-certificates git build-essential cmake pkg-config hostapd dnsmasq iw wireless-tools rfkill iproute2 procps"
if [ "$BRANCH" = "main" ]; then
    PACKAGES="$PACKAGES xorg-dev libasound2-dev mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libudev-dev"
fi
run $SUDO apt-get install -y $PACKAGES

section "Checking out $BRANCH"
if repo_command_succeeds git show-ref --verify --quiet "refs/heads/$BRANCH"; then
    run_repo git checkout "$BRANCH"
elif repo_command_succeeds git show-ref --verify --quiet "refs/remotes/origin/$BRANCH"; then
    run_repo git checkout -b "$BRANCH" "origin/$BRANCH"
else
    die "branch not found locally or at origin: $BRANCH"
fi

section "Updating user permissions"
for group in dialout input video render; do
    if getent group "$group" >/dev/null 2>&1; then
        run $SUDO usermod -a -G "$group" "$SERVICE_USER"
    fi
done

section "Configuring Proton Server hotspot"
write_hotspot_config

section "Building Proton Server"
run_repo cmake -S "$REPO_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
run_repo cmake --build "$BUILD_DIR" -j "$(getconf _NPROCESSORS_ONLN)"

EXECUTABLE="$BUILD_DIR/proton-server"
[ -x "$EXECUTABLE" ] || die "build finished, but executable was not found: $EXECUTABLE"

section "Installing startup services"
write_server_service
run $SUDO systemctl daemon-reload
run_optional $SUDO systemctl unmask hostapd
run_optional $SUDO systemctl disable --now hostapd
run_optional $SUDO systemctl disable --now dnsmasq
run_optional $SUDO rm -f /etc/proton-server/hotspot-enabled
run $SUDO systemctl enable proton-hotspot.service
run $SUDO systemctl enable "$SERVICE_NAME.service"

if [ "$START_SERVICE" -eq 1 ]; then
    section "Starting services"
    run $SUDO systemctl restart proton-hotspot.service
    sleep 2
    if ! $SUDO systemctl is-active --quiet proton-hotspot.service; then
        echo "proton-hotspot.service failed to start. Recent logs:" >&2
        $SUDO journalctl -u proton-hotspot.service -n 80 --no-pager >&2 || true
        exit 1
    fi
    run $SUDO systemctl restart "$SERVICE_NAME.service"
else
    section "Skipping service start"
    echo "Services are enabled for boot and were not started because --no-start was used."
fi

section "Done"
echo "Installed $BRANCH build:"
echo "  $EXECUTABLE"
echo
echo "Startup services:"
echo "  $SERVICE_NAME.service"
echo "  proton-hotspot.service (SSID: $HOTSPOT_SSID)"
echo
echo "Useful commands:"
echo "  sudo systemctl status $SERVICE_NAME.service"
echo "  sudo systemctl status proton-hotspot.service"
echo "  sudo journalctl -u proton-hotspot.service -f"
echo "  sudo journalctl -u $SERVICE_NAME.service -f"
if [ "$BRANCH" = "main" ]; then
    echo
    echo "Note: the main branch opens a local raylib window and expects Raspberry Pi OS Desktop/X on display :0."
    echo "Use the headless branch for Raspberry Pi OS Lite."
fi
