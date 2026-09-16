#!/usr/bin/env bash
# Push pi3/ to the rover's Raspberry Pi 3, run things there, and bring the logs back.
#
#   ./pi.sh key            one-time: install the SSH key, so no more password prompts
#   ./pi.sh setup          push, then venv + ultralytics/pyserial on the Pi, camera check
#   ./pi.sh hotspot        one-time: RoverCam hotspot as a fallback - the Pi starts it by
#                          itself wherever home Wi-Fi is out of range (the field)
#   ./pi.sh push           copy the code to the Pi
#   ./pi.sh run [args]     push, run follow_rover.py with output live here, pull logs
#                          (Ctrl+C stops it; args go to follow_rover.py)
#   ./pi.sh start [args]   push, run follow_rover.py in the background on the Pi
#   ./pi.sh stop           stop the follower (it sends the motors a stop on the way out)
#   ./pi.sh restart        push, then restart the boot-service follower with the new code
#   ./pi.sh calibrate [dist]  stand that far from the camera (default 1m), facing it
#                          and hold still: it saves the calibration for measuring distance
#                          (no sudo, no stop). e.g. ./pi.sh calibrate 60in
#   ./pi.sh autostart on|off|status   run the follower at every power-on (systemd service);
#                          its options live in rover.env on the Pi (default: --port auto)
#   ./pi.sh drivetest [pwm]  wheels off the ground: 1 s forward, then 1 s backward (default pwm 110)
#   ./pi.sh logs           watch the Pi's latest log live
#   ./pi.sh pull           copy every Pi log into pi_logs/ and print the latest
#   ./pi.sh status         temperature, hotspot, webcam, serial ports, follower
#   ./pi.sh view [args]    live camera window on the Pi's desktop (watch it in VNC)
#   ./pi.sh detect [args]  same, with YOLO person boxes and fps (detectview.py)
#   ./pi.sh watch          on the laptop: what the running follower sees, in a cv2 window
#                          (needs OpenCV: VIEW_PY=/path/to/python; or open http://<pi>:8080/)
#   ./pi.sh camtest        Pi ribbon camera not detected? probes it and says why
#   ./pi.sh usbtest        USB webcam check: lists it and its formats, reads 5 s of video
#   ./pi.sh sh [cmd]       shell on the Pi, or run one command in the project folder
#
# Everything run on the Pi is saved there as logs/<name>-<time>.log, with
# logs/latest.log pointing at the newest.  Another Pi: PI=user@host ./pi.sh ...

set -euo pipefail
export YOLO_OFFLINE=1  # ultralytics: no update/online checks - the rover works without internet

PI="${PI:-rasp@100.100.69.124}"  # the rover's Pi 3, over Tailscale
[[ ${1:-} == pi3 ]] && shift       # ./pi.sh pi3 <command> still works
DIR="${PI_DIR:-HumanFollowing}"  # on the Pi, relative to home
KEY="$HOME/.ssh/id_ed25519_pibench"
# The Pi's own Wi-Fi in the field, for the laptop/phone to reach it at 10.42.0.1
HOTSPOT_SSID=RoverCam
HOTSPOT_PASS=rovercam123
SERVICE=rover-follow
FOLLOW_RE='^[^ ]*python[0-9.]* -u follow_rover\.py'
EXCLUDES=(--exclude __pycache__ --exclude logs --exclude pi_logs --exclude snapshots --exclude calibration.json)

# ---------------------------------------------------------------- on the Pi
# Commands starting with _ are what the local commands invoke over SSH.

_logged() {  # _logged NAME CMD...: run CMD, show its output, keep a copy in logs/
  local name=$1; shift
  mkdir -p logs
  local log="logs/$name-$(date +%Y%m%d-%H%M%S).log"
  ln -sfn "$(basename "$log")" logs/latest.log
  # keep the newest 60: a boot service restarting in a loop must not fill the SD card
  ls -1t logs/*.log 2>/dev/null | grep -v latest.log | tail -n +61 | xargs -r rm -f
  { echo "# $name  $(date '+%F %T')  $(hostname)  $(vcgencmd measure_temp 2>/dev/null)"
    echo "# \$ $*"; } | tee "$log"
  set +e
  trap : INT  # let Ctrl+C reach the command, but survive it to write the footer
  "$@" 2>&1 | tee -i -a "$log"
  local rc=${PIPESTATUS[0]}
  echo "# exit $rc  $(date '+%F %T')" | tee -a "$log"
  return "$rc"
}

_not_running() {
  if systemctl is-active --quiet "$SERVICE" 2>/dev/null; then
    echo "the follower is running as the boot service - ./pi.sh stop (until next boot) or ./pi.sh autostart off"
    exit 1
  fi
  if pgrep -f "$FOLLOW_RE" >/dev/null; then
    echo "follower already running (pid $(pgrep -f "$FOLLOW_RE" | tr '\n' ' ')) - ./pi.sh stop first"
    exit 1
  fi
}

_follow() {
  _not_running
  _logged follow "$HOME/rover-venv/bin/python" -u follow_rover.py "$@"
}

_start() {
  _not_running
  mkdir -p logs
  setsid nohup bash pi.sh _logged follow "$HOME/rover-venv/bin/python" -u follow_rover.py "$@" \
    >/dev/null 2>&1 </dev/null &
  # wait for startup to finish (camera, model, link) or fail
  for _ in $(seq 120); do
    sleep 0.5
    if grep -qE '^(rover link|no rover link)' logs/latest.log 2>/dev/null; then
      echo "running in background, log: $DIR/logs/$(readlink logs/latest.log)"
      return
    fi
    if ! pgrep -f "$FOLLOW_RE" >/dev/null; then
      sleep 0.3
      echo "follower exited during startup:"; cat logs/latest.log; exit 1
    fi
  done
  echo "still starting after 60 s, check ./pi.sh logs"
}

_drivetest() {  # check "forward" really is forward before trusting the follower
  _not_running
  "$HOME/rover-venv/bin/python" -u - "${1:-110}" <<'PY'
import sys, time
from follow_rover import RoverLink
pwm = int(sys.argv[1])
link = RoverLink("auto", 1.5)
print(f"ESP32 on {link.port}")
try:
    for label, v in (("FORWARD", pwm), ("stop", 0), ("BACKWARD", -pwm)):
        print(f"{label} (M{v},{v}) for 1 s", flush=True)
        link.drive(v, v)
        time.sleep(1.0)
finally:
    link.close()
    print("stopped")
PY
}

_stop() {
  if systemctl is-active --quiet "$SERVICE" 2>/dev/null; then
    sudo systemctl stop "$SERVICE"  # SIGTERM: the follower sends the motor stop on the way out
    echo "boot service stopped (it starts again at next power-on; ./pi.sh autostart off to disable)"
    return
  fi
  if ! pgrep -f "$FOLLOW_RE" >/dev/null; then echo "follower not running"; return; fi
  pkill -TERM -f "$FOLLOW_RE"
  for _ in $(seq 50); do
    pgrep -f "$FOLLOW_RE" >/dev/null || { echo "stopped"; return; }
    sleep 0.1
  done
  pkill -KILL -f "$FOLLOW_RE"
  echo "did not exit within 5 s, killed (the ESP32 timeout stops the motors)"
}

_setup() {
  echo "--- rsync"
  command -v rsync >/dev/null || sudo apt-get install -y rsync
  echo "--- camera"
  _cameras
  echo "--- venv"
  [[ -d ~/rover-venv ]] || python3 -m venv --system-site-packages ~/rover-venv
  local pip=~/rover-venv/bin/pip
  # Skip /etc/pip.conf: its piwheels mirror serves wheels with broken metadata, and
  # PyPI has 64-bit ARM wheels anyway.
  export PIP_CONFIG_FILE=/dev/null
  # Keep the apt numpy that picamera2 was built against; a pip numpy 2 breaks it on Bookworm.
  local np; np=$(python3 -c "import numpy; print(numpy.__version__)")
  echo "numpy==$np" > ~/rover-venv/constraints.txt
  export PIP_CONSTRAINT=~/rover-venv/constraints.txt
  echo "(numpy pinned to the system's $np)"
  echo "--- torch, CPU build (the default aarch64 wheel pulls ~2 GB of NVIDIA CUDA)"
  $pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    --extra-index-url https://pypi.org/simple  # deps from PyPI; +cpu still wins on version
  echo "--- ultralytics (keeps the CPU torch above)"
  $pip install ultralytics lap pyserial onnxruntime  # onnxruntime: 3x faster yolo11n on the Pi 3
  $pip uninstall -y ncnn 2>/dev/null || true  # its aarch64 wheel dies with a bus error on import on the Pi 3
  echo "--- remove CUDA leftovers"
  # nvidia-ml-py is a tiny pure-Python ultralytics dependency, not CUDA - keep it
  local cuda; cuda=$($pip list --format=freeze | grep -E '^(nvidia-|triton)' | grep -v '^nvidia-ml-py' | cut -d= -f1 || true)
  if [[ -n $cuda ]]; then $pip uninstall -y $cuda; else echo "none installed"; fi
  $pip cache purge || true  # downloaded wheels, including any CUDA ones; GBs on an SD card
  echo "--- import check"
  ~/rover-venv/bin/python -c "import picamera2, ultralytics, serial, cv2, torch; \
print('picamera2 ok | ultralytics', ultralytics.__version__, '| torch', torch.__version__, '| cv2', cv2.__version__)"
  echo "--- serial access"
  groups | grep -qw dialout && echo "$USER is in dialout" \
    || echo "WARNING: not in dialout; run: sudo usermod -aG dialout $USER && sudo reboot"
}

_view() {  # _view NAME SCRIPT ARGS: run SCRIPT on the Pi's own desktop, so its window shows in VNC
  local name=$1 script=$2; shift 2
  local py=python3; [[ -x ~/rover-venv/bin/python ]] && py=~/rover-venv/bin/python
  $py -c "import cv2" 2>/dev/null || sudo apt-get install -y python3-opencv
  export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}
  export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0} DISPLAY=${DISPLAY:-:0}
  if [[ ! -S $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY && ! -S /tmp/.X11-unix/X${DISPLAY#:} ]]; then
    echo "no desktop session running on the Pi - log in to the desktop (VNC) first"; exit 1
  fi
  _logged "$name" $py -u "$script" "$@"
}

_restart() {  # new code into the running boot service, no sudo: it restarts itself when the follower exits
  if ! systemctl is-active --quiet "$SERVICE"; then
    echo "boot service not running - ./pi.sh autostart on (or ./pi.sh run for a one-off)"; exit 1
  fi
  pkill -TERM -f "$FOLLOW_RE" && echo "follower stopping (motors get a stop); the service starts it again in ~10 s"
  sleep 14
  echo "running: $(pgrep -af "$FOLLOW_RE" | cut -c1-100 || echo 'not yet - check ./pi.sh logs')"
}

_calibrate() {  # _calibrate [dist]: ask the running follower to calibrate distance, standing at dist
  local pid; pid=$(pgrep -f "$FOLLOW_RE" | head -1 || true)
  if [[ -z $pid ]]; then
    echo "no follower running - use: ./pi.sh run --calibrate"; exit 1
  fi
  # a follower started before the latest push does not know calibration requests yet
  if (( $(stat -c %Y /proc/$pid) < $(stat -c %Y follow_rover.py) )); then
    echo "follower predates the new code - restarting it"
    pkill -TERM -f "$FOLLOW_RE"
    for _ in $(seq 60); do
      sleep 2
      pid=$(pgrep -f "$FOLLOW_RE" | head -1 || true)
      [[ -n $pid ]] && (( $(stat -c %Y /proc/$pid) >= $(stat -c %Y follow_rover.py) )) \
        && grep -qE "rover link|no rover link" logs/latest.log 2>/dev/null && break
    done
  fi
  if [[ -n ${1:-} ]]; then
    printf '{"dist": "%s"}\n' "$1" > calibrate.request
    echo "stand $1 from the camera and hold still..."
  else
    echo '{}' > calibrate.request
    echo "stand 1 m from the camera, facing it, and hold still..."
  fi
  local log; log=logs/$(readlink logs/latest.log)
  timeout 120 tail -n 0 -F "$log" 2>/dev/null | while IFS= read -r line; do
    case "$line" in
      *calibrat*) echo "$line" ;;
    esac
    case "$line" in
      *"following again"*|*"calibration cancelled"*) pkill -P $$ tail; break ;;
    esac
  done || true
}

_autostart() {  # _autostart on|off|status: the follower as a systemd service started at boot
  case "${1:-status}" in
    on)
      [[ -f rover.env ]] || echo 'ROVER_ARGS="--port auto"' > rover.env
      sudo tee /etc/systemd/system/$SERVICE.service >/dev/null <<UNIT
[Unit]
Description=Human-following rover (follow_rover.py)
After=multi-user.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$PWD
Environment=YOLO_OFFLINE=1 PYTHONUNBUFFERED=1
# options to follow_rover.py, e.g. ROVER_ARGS="--port auto --stop-dist 1.5"
EnvironmentFile=-$PWD/rover.env
ExecStart=/bin/bash $PWD/pi.sh _logged follow $HOME/rover-venv/bin/python -u follow_rover.py \$ROVER_ARGS
# come back whenever it exits: ESP32/webcam not ready at boot, a crash, or ./pi.sh restart
# (systemctl stop, i.e. ./pi.sh stop, still stops it for good until the next boot)
Restart=always
RestartSec=10
# SIGTERM makes the follower send the motors a stop before it exits
KillSignal=SIGTERM
TimeoutStopSec=15

[Install]
WantedBy=multi-user.target
UNIT
      pkill -TERM -f "$FOLLOW_RE" 2>/dev/null && sleep 2  # a manual run would hold the camera
      sudo systemctl daemon-reload
      sudo systemctl enable "$SERVICE"
      sudo systemctl restart "$SERVICE"  # also picks up new code if it was already running
      sleep 3
      echo "autostart ON: the follower starts at every power-on. Options: $(cat rover.env)"
      systemctl --no-pager --lines=0 status "$SERVICE" | head -3
      ;;
    off)
      sudo systemctl disable --now "$SERVICE" 2>/dev/null || true
      echo "autostart OFF: nothing runs at power-on; use ./pi.sh run / start"
      ;;
    *)
      echo "enabled: $(systemctl is-enabled "$SERVICE" 2>/dev/null || echo no)   running: $(systemctl is-active "$SERVICE" 2>/dev/null || true)"
      echo "options: $(cat rover.env 2>/dev/null || echo 'none (rover.env missing)')"
      ;;
  esac
}

_hotspot() {  # RoverCam access point on wlan0, used whenever no known Wi-Fi is in range
  local country; country=$(sudo raspi-config nonint get_wifi_country 2>/dev/null || true)
  if [[ -z $country ]]; then
    echo "Wi-Fi country is not set, so the radio stays off. Set it first, e.g.:"
    echo "  ./pi.sh sh 'sudo raspi-config nonint do_wifi_country NP'   (your 2-letter country code)"
    exit 1
  fi
  sudo rfkill unblock wifi
  sudo nmcli connection delete "$HOTSPOT_SSID" >/dev/null 2>&1 || true
  # WPA2-AES only and 2.4 GHz: what the Pi 3 radio handles reliably. PMF off: NetworkManager
  # advertises it, the Pi 3's brcmfmac chip can't do it in AP mode, and clients then drop the
  # connection before the password is checked (wpa_supplicant: DISCONNECTED reason=17).
  # Lowest autoconnect priority: home Wi-Fi (priority 0) wins when in range, and
  # NetworkManager falls back to this hotspot when it isn't.
  sudo nmcli connection add type wifi ifname wlan0 con-name "$HOTSPOT_SSID" autoconnect yes \
    connection.autoconnect-priority -10 \
    ssid "$HOTSPOT_SSID" 802-11-wireless.mode ap 802-11-wireless.band bg \
    ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method disabled \
    wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp \
    wifi-sec.pmf disable \
    wifi-sec.psk "$HOTSPOT_PASS"
  if ip route show default | grep -q "dev wlan0"; then
    # starting it now would drop this connection (one radio: client or hotspot, not both)
    echo "hotspot $HOTSPOT_SSID saved as a fallback (country $country)."
    echo "It starts by itself when home Wi-Fi is out of range - boot the Pi in the field."
  else
    sudo nmcli connection up "$HOTSPOT_SSID"
    echo "hotspot $HOTSPOT_SSID is up on 10.42.0.1 (country $country)"
  fi
}

_usbtest() {  # is the USB webcam working for the Pi, and at what rate?
  _not_running
  if command -v v4l2-ctl >/dev/null; then
    echo "--- video devices"
    v4l2-ctl --list-devices 2>&1 || true
  else
    echo "(install v4l-utils for the device and format list: sudo apt install v4l-utils)"
  fi
  echo "--- reading 5 s through camera.py (source usb)"
  "$HOME/rover-venv/bin/python" -u - <<'PY'
import time
from camera import open_camera
t = time.time()
cam = open_camera("usb")
print(f"first frame after {time.time() - t:.1f} s")
c0, t0 = cam.count, time.time()
time.sleep(5)
f = cam.read()
print(f"{(cam.count - c0) / (time.time() - t0):.1f} fps, frame {None if f is None else f.shape}, "
      f"newest frame {cam.age() * 1000:.0f} ms old")
cam.release()
PY
}

_cameras() { rpicam-hello --list-cameras 2>&1 || libcamera-hello --list-cameras 2>&1 || true; }

_camtest() {  # force-load sensor drivers and report what the sensor did
  echo "$(tr -d '\0' </proc/device-tree/model), kernel $(uname -r), $(uname -m)"
  if _cameras | grep -qE '^[0-9]+ : '; then
    echo "camera detected:"; _cameras | grep -E '^[0-9]+ : '; return
  fi
  local sensor log
  for sensor in ov5647 imx219 imx708 imx477; do  # v1.3 / v2 / v3 / HQ cameras
    sudo dmesg -C
    sudo dtoverlay "$sensor" 2>/dev/null || { echo "$sensor: overlay not available"; continue; }
    sleep 4
    log=$(sudo dmesg | grep -E "$sensor [0-9]+-00" || true)
    if _cameras | grep -q "$sensor"; then
      echo "$sensor: WORKS (reboot and it will be auto-detected)"
    elif grep -q "error -121" <<<"$log"; then
      echo "$sensor: no answer (-121) - not this sensor, or no contact"
    elif grep -q "error -5" <<<"$log"; then
      echo "$sensor: bus error (-5) - connected but the lines are wrong: ribbon reversed, not fully seated, or the wrong cable"
    else
      echo "$sensor: no probe result"; [[ $log ]] && echo "$log"
    fi
    sudo dtoverlay -r "$sensor" >/dev/null 2>&1 || true
    sleep 1
  done
  echo "(test overlays removed; config.txt untouched)"
}

_status() {
  echo "host      $(hostname)  $(uname -m)  up $(uptime -p | sed 's/^up //')"
  if groups | grep -qw video; then
    echo "temp      $(vcgencmd measure_temp 2>/dev/null)   $(vcgencmd get_throttled 2>/dev/null) (0x0 = never throttled)"
  else
    echo "temp      temp=$(( $(cat /sys/class/thermal/thermal_zone0/temp) / 1000 ))'C"
    echo "WARNING   $USER is not in the video group: the camera needs sudo. Fix: sudo usermod -aG video,render $USER"
  fi
  echo "memory    $(free -h | awk '/Mem:/ {print $3 " used of " $2}')"
  if nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx "$HOTSPOT_SSID"; then
    echo "hotspot   on: $HOTSPOT_SSID, 10.42.0.1"
  else
    if nmcli -t -f NAME connection show 2>/dev/null | grep -qx "$HOTSPOT_SSID"; then
      echo "hotspot   saved, not active (normal on home Wi-Fi; starts by itself in the field)"
    else
      echo "hotspot   not set up - run ./pi.sh hotspot"
    fi
  fi
  local cams="" dev
  for dev in /dev/video*; do
    [[ -e $dev ]] || continue
    local name; name=$(cat "/sys/class/video4linux/${dev##*/}/name" 2>/dev/null)
    [[ ${name,,} =~ bcm2835|unicam|rpivid|pispbe|codec|isp ]] || cams+="$dev ($name)  "
  done
  echo "webcam    ${cams:-none found - is the USB webcam plugged into the Pi?}"
  local ports; ports=$(ls /dev/ttyUSB* /dev/ttyACM* /dev/rfcomm* 2>/dev/null | tr '\n' ' ' || true)
  echo "serial    ${ports:-none (ESP32 not plugged in?)}"
  echo "follower  $(pgrep -af "$FOLLOW_RE" || echo 'not running')"
  echo "autostart $(systemctl is-enabled "$SERVICE" 2>/dev/null || echo off) ($(systemctl is-active "$SERVICE" 2>/dev/null || true))"
  echo "power     $(vcgencmd get_throttled 2>/dev/null) (0x0 = fine; 0x50005 etc. = undervoltage: weak power supply)"
  echo "calib     $(cat calibration.json 2>/dev/null | tr -d '\n ' || echo 'none - run ./pi.sh calibrate')"
  echo "logs      $(ls logs 2>/dev/null | grep -vc latest || true) on the Pi, latest: $(readlink logs/latest.log 2>/dev/null || echo none)"
}

# ---------------------------------------------------------------- here

HERE="$(cd "$(dirname "$0")" && pwd)"
LOGS="$HERE/pi_logs"

# One SSH connection is reused for every call, so a password is asked at most once.
SSH_OPTS=(-o ControlMaster=auto -o "ControlPath=$HOME/.ssh/cm-%r@%h-%p"
          -o ControlPersist=10m -o ConnectTimeout=10)
[[ -f $KEY ]] && SSH_OPTS+=(-i "$KEY")

pi()     { ssh "${SSH_OPTS[@]}" "$PI" "$@"; }
pi_tty() { ssh -t "${SSH_OPTS[@]}" "$PI" "$@"; }
quote()  { (( $# )) && printf '%q ' "$@" || true; }

push() {
  if pi 'command -v rsync >/dev/null'; then
    rsync -az -e "ssh ${SSH_OPTS[*]}" "${EXCLUDES[@]}" "$HERE/pi3/" "$HERE/pi.sh" "$PI:$DIR/"
  else
    tar "${EXCLUDES[@]}" -czf - -C "$HERE/pi3" . -C "$HERE" pi.sh | pi "mkdir -p $DIR && tar -xzf - -C $DIR"
  fi
  echo ">> pushed pi3/ to $PI:$DIR"
}

pull() {
  mkdir -p "$LOGS"
  if ! pi "test -d $DIR/logs"; then echo ">> no logs on the Pi yet"; return 1; fi
  if pi 'command -v rsync >/dev/null'; then
    rsync -az -e "ssh ${SSH_OPTS[*]}" "$PI:$DIR/logs/" "$LOGS/"
  else
    pi "tar -C $DIR/logs -czf - ." | tar -xzf - -C "$LOGS"
  fi
  echo ">> pulled $(ls "$LOGS"/*.log | grep -vc latest) logs into pi_logs/"
}

case "${1:-}" in
  key)
    [[ -f $KEY ]] || ssh-keygen -t ed25519 -N '' -f "$KEY"
    ssh-copy-id -i "$KEY.pub" "$PI"
    ;;
  setup)
    push
    pi_tty "cd $DIR && bash pi.sh _logged setup bash pi.sh _setup" || true
    pull >/dev/null
    ;;
  push)
    push
    ;;
  run)
    shift
    push
    pi_tty "cd $DIR && bash pi.sh _follow $(quote "$@")" || true
    pull && echo ">> this run: pi_logs/$(readlink "$LOGS/latest.log")"
    ;;
  start)
    shift
    push
    pi "cd $DIR && bash pi.sh _start $(quote "$@")"
    echo ">> ./pi.sh logs to watch, ./pi.sh stop to stop"
    ;;
  drivetest)
    shift
    push
    pi_tty "cd $DIR && bash pi.sh _logged drivetest bash pi.sh _drivetest $(quote "$@")" || true
    ;;
  stop)
    pi "cd $DIR && bash pi.sh _stop"
    pull >/dev/null || true
    ;;
  logs)
    pi_tty "tail -n 40 -F $DIR/logs/latest.log" || true
    ;;
  pull)
    pull
    echo "=================== pi_logs/$(readlink "$LOGS/latest.log")"
    cat "$LOGS/latest.log"
    ;;
  status)
    pi "mkdir -p $DIR && cd $DIR && bash -s _status" < "$0"
    ;;
  camtest)
    pi "bash -s _camtest" < "$0"
    ;;
  usbtest)
    push
    pi_tty "cd $DIR && bash pi.sh _logged usbtest bash pi.sh _usbtest" || true
    ;;
  calibrate)
    shift
    push
    pi "cd $DIR && bash pi.sh _calibrate $(quote "$@")"
    ;;
  restart)
    push
    pi "cd $DIR && bash pi.sh _restart"
    ;;
  autostart)
    shift
    push
    pi_tty "cd $DIR && bash pi.sh _autostart ${1:-status}" || true
    ;;
  hotspot)
    push
    pi_tty "cd $DIR && bash pi.sh _hotspot"
    ;;
  watch)
    url="http://${PI#*@}:${VIEW_PORT:-8080}/"
    py=${VIEW_PY:-$HOME/miniforge3/envs/fl/bin/python}
    if ! "$py" -c "import cv2" 2>/dev/null; then
      echo "no OpenCV in $py - set VIEW_PY=/path/to/python, or open $url in a browser"; exit 1
    fi
    echo ">> watching $url  (q to quit; the follower must be running with its --view port)"
    cd "$HERE/pi3" && "$py" camview.py --source "$url"
    ;;
  view|detect)
    script=camview.py; [[ $1 == detect ]] && script=detectview.py
    name=$1; shift
    push
    pi_tty "cd $DIR && bash pi.sh _view $name $script $(quote "$@")" || true
    pull >/dev/null || true
    ;;
  sh)
    shift
    if (( $# )); then pi_tty "cd $DIR && $*"; else pi_tty; fi
    ;;
  _*)
    cmd=$1; shift
    "$cmd" "$@"
    ;;
  *)
    awk 'NR > 1 && !/^#/ {exit} NR > 1 {sub(/^# ?/, ""); print}' "$0"
    ;;
esac
