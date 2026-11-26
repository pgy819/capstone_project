#!/usr/bin/env bash
# RCWS Stage 1 - Raspberry Pi 5 deploy: packet v2 link to the STM32 + operator web page + launcher.
#
#   UART: /dev/ttyAMA0 (GPIO14/15, needs dtparam=uart0=on). Override: RCWS_UART=/dev/xxx bash deploy.sh
#   PC:  ssh -t pgy819@rcwspi.local "curl -fsSL https://raw.githubusercontent.com/pgy819/capstone_project/main/rcws_stage1/tools/pi/deploy.sh | bash"
#   Pi:  bash deploy.sh [branch]        (default: main)
#
# Installs into ~/rcws_live (earlier deployments ~/rcws, ~/run3 are left alone):
#   ~/rcws_live/src         git checkout
#   ~/rcws_live/go.sh       one run: camera + Hailo + STM32 link + web page on :8080 (Ctrl-C saves log/clip)
#   ~/rcws_live/logs        rlog / rclip / summary per run
#   launcher                http://<pi>:8000/  start / stop without SSH (systemd user service)
set -euo pipefail

BRANCH=${1:-main}
REPO=https://github.com/pgy819/capstone_project.git
DEST="$HOME/rcws_live"
SRC="$DEST/src"
RPI="$SRC/rcws_stage1/rpi"

say() { printf '\n\033[1;34m[rcws]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[rcws] WARNING:\033[0m %s\n' "$*"; }
die() { printf '\n\033[1;31m[rcws] ERROR:\033[0m %s\n' "$*" >&2; exit 1; }

# ------------------------------------------------------------------ 1. code
say "1/6 Fetch code ($BRANCH)"
mkdir -p "$DEST/logs"
if [ -d "$SRC/.git" ]; then
  git -C "$SRC" fetch --depth 1 origin "$BRANCH"
  git -C "$SRC" checkout -f --detach FETCH_HEAD
  git -C "$SRC" clean -fdq
else
  git clone --branch "$BRANCH" --depth 1 "$REPO" "$SRC" || die "Could not fetch from GitHub (is branch $BRANCH pushed? Pi internet?)."
fi
COMMIT=$(git -C "$SRC" rev-parse --short HEAD)
say "Code version: $COMMIT"

# ------------------------------------------------------------------ 2. model
say "2/6 Detector model (HEF)"
mkdir -p "$RPI/models"
HEF="$RPI/models/yolov8s_h8.hef"
if [ ! -f "$HEF" ]; then
  for c in "$HOME/run3/src/rcws_stage1/rpi/models/yolov8s_h8.hef" "$HOME/rcws/rpi/models/yolov8s_h8.hef" \
           /usr/share/hailo-models/yolov8s_h8.hef; do
    if [ -f "$c" ]; then cp "$c" "$HEF"; say "HEF copied: $c"; break; fi
  done
fi
[ -f "$HEF" ] || die "yolov8s_h8.hef not found. Copy it to $HEF and run again."

# ------------------------------------------------------------------ 3. build
say "3/6 Build (a few minutes)"
cd "$RPI"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release | grep -E "rcws:" || true
cmake --build build -j4
ldd build/rcws_vision | grep libhailort >/dev/null || die "Built without HailoRT: sudo apt install hailo-all, rm -rf $RPI/build, then run again."
ldd build/rcws_vision | grep libcamera >/dev/null || die "Built without libcamera: sudo apt install libcamera-dev, rm -rf $RPI/build, then run again."
sudo setcap cap_sys_nice+ep build/rcws_vision || warn "setcap failed: runs without real-time priority."

# ------------------------------------------------------------------ 4. tests
say "4/6 Unit tests"
./build/rcws_tests 2>&1 | tee "$DEST/tests.txt" | grep --line-buffered -E "^\[ FAIL|CHECK|tests," || true
if ! grep -q " 0 failed checks" "$DEST/tests.txt"; then
  if [ "${RCWS_ALLOW_TEST_FAIL:-0}" = 1 ]; then
    warn "Continuing despite test failures (RCWS_ALLOW_TEST_FAIL=1). Details: $DEST/tests.txt"
  else
    die "Tests failed. Send $DEST/tests.txt. (To continue anyway: RCWS_ALLOW_TEST_FAIL=1)"
  fi
fi

# ------------------------------------------------------------------ 5. UART to the STM32
say "5/6 STM32 UART check (Pi GPIO14 TXD -> STM32 PC7, GPIO15 RXD <- PC6, common GND)"
# Pi 5: GPIO14/15 = PL011 UART0 = /dev/ttyAMA0, only with 'dtparam=uart0=on'.
#        /dev/serial0 defaults to /dev/ttyAMA10 = the 3-pin DEBUG connector, NOT the GPIO header.
UART="${RCWS_UART:-}"
if [ -z "$UART" ] && [ -e /dev/ttyAMA0 ]; then UART=/dev/ttyAMA0; fi
if [ -n "$UART" ]; then
  say "UART: $UART (GPIO14/15)"
else
  warn "/dev/ttyAMA0 (GPIO14/15) not found. /dev/serial0 is $(readlink -f /dev/serial0 2>/dev/null || echo none),"
  warn "which on the Pi 5 is the debug connector (ttyAMA10), not the GPIO header. Do this, reboot, run deploy again:"
  warn "  echo 'dtparam=uart0=on' | sudo tee -a /boot/firmware/config.txt && sudo reboot"
  warn "Until then go.sh runs without the STM32 (no servo drive)."
fi
if [ -n "$UART" ] && grep -Eq "console=(ttyAMA0|$(basename "$UART"))" /proc/cmdline; then
  warn "The serial console uses $UART (boot messages go to the STM32). Remove console=ttyAMA0,... from /boot/firmware/cmdline.txt and reboot."
fi
id -nG | grep -qw dialout || { warn "User is not in the dialout group: sudo usermod -aG dialout $USER, then log in again."; }

# ------------------------------------------------------------------ 6. run script + launcher
say "6/6 Run script and launcher (:8000)"
MEM_GB=$(awk '/MemTotal/ {printf "%d", $2/1048576}' /proc/meminfo)
CLIP=$([ "$MEM_GB" -ge 6 ] && echo 3 || echo 2)
UART_ARGS=""
[ -n "$UART" ] && UART_ARGS="--uart $UART --uart-baud 115200"
cat > "$DEST/go.sh" <<EOS
#!/usr/bin/env bash
# one tracking run (from the launcher page or by hand). Ctrl-C / web "Stop" saves log and clip.
cd "$RPI"
CFG=\${1:-config/rcws.ini}
NAME=live_\$(date +%m%d_%H%M%S)
echo "[rcws] code $COMMIT, config \$CFG, log $DEST/logs/\$NAME.rlog"
thr=\$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
echo "[rcws] throttled=\$thr \$([ "\$thr" = 0x0 ] && echo OK || echo '!! UNDERVOLTAGE/THROTTLED')"
RC=0
trap ':' INT
./build/rcws_vision --config "\$CFG" --source camera --detector hailo $UART_ARGS \\
    --log "$DEST/logs/\$NAME.rlog" --clip-seconds $CLIP --preview 8080 --web-root "$RPI/web" || RC=\$?
trap - INT
echo "[rcws] rcws_vision exit=\$RC"
mv -f run.rclip "$DEST/logs/\$NAME.rclip" 2>/dev/null || true
if [ -f "$DEST/logs/\$NAME.rlog" ]; then
  python3 "$SRC/rcws_stage1/tools/rcws_log.py" "$DEST/logs/\$NAME.rlog" | tee "$DEST/logs/\$NAME.txt"
fi
EOS
chmod +x "$DEST/go.sh"

UNIT_DIR="$HOME/.config/systemd/user"
mkdir -p "$UNIT_DIR"
sed -e "s|@LAUNCHER@|$RPI/build/rcws_launcher|" -e "s|@GO@|$DEST/go.sh|" -e "s|@LOG@|$DEST/logs/launcher.txt|" \
  "$SRC/rcws_stage1/tools/pi/rcws-launcher.service" > "$UNIT_DIR/rcws-launcher.service"
systemctl --user daemon-reload
systemctl --user enable rcws-launcher.service >/dev/null
systemctl --user restart rcws-launcher.service
loginctl show-user "$USER" 2>/dev/null | grep -q "Linger=yes" || sudo loginctl enable-linger "$USER" \
  || warn "enable-linger failed: the launcher starts only after login."

# Pi screen: the same web pages in a browser window on the Pi desktop (launcher -> operator page).
# Started at desktop login (XDG autostart) and by hand: ~/rcws_live/screen.sh
cat > "$DEST/screen.sh" <<'EOS'
#!/usr/bin/env bash
# Opens the RCWS pages on the Pi's own monitor. Over SSH: DISPLAY=:0 ~/rcws_live/screen.sh
export DISPLAY=${DISPLAY:-:0}
[ -z "${WAYLAND_DISPLAY:-}" ] && [ -S "/run/user/$(id -u)/wayland-0" ] && export WAYLAND_DISPLAY=wayland-0
B=$(command -v chromium-browser || command -v chromium || command -v firefox || true)
[ -n "$B" ] || { echo "no browser (sudo apt install chromium-browser)"; exit 1; }
for i in $(seq 1 30); do curl -s -o /dev/null http://localhost:8000/ && break; sleep 1; done
URL=http://localhost:8000/
case "$B" in
  *firefox) exec "$B" --new-window "$URL" ;;
  # own profile: no first-run / restore prompts; --app: a plain window without tabs or address bar
  *) exec "$B" --user-data-dir="$HOME/.config/rcws-screen" --no-first-run --noerrdialogs \
          --disable-session-crashed-bubble --password-store=basic --start-maximized --app="$URL" ;;
esac
EOS
chmod +x "$DEST/screen.sh"
mkdir -p "$HOME/.config/autostart"
cat > "$HOME/.config/autostart/rcws-screen.desktop" <<EOS
[Desktop Entry]
Type=Application
Name=RCWS screen
Exec=$DEST/screen.sh
X-GNOME-Autostart-enabled=true
EOS
SCREEN_NOTE="auto from the next desktop login; now: DISPLAY=:0 ~/rcws_live/screen.sh"
if ! command -v chromium-browser >/dev/null && ! command -v chromium >/dev/null && ! command -v firefox >/dev/null; then
  SCREEN_NOTE="no browser (sudo apt install chromium-browser)"
fi

IP=$(hostname -I | awk '{print $1}')
say "Done ($COMMIT)"
cat <<EOF

  Launcher (start/stop): http://$IP:8000/   (starts at boot)
  Operator page:         http://$IP:8080/   (while tracking runs; several laptops/phones at once)
  Pi screen:             $SCREEN_NOTE
  Run by hand:           ~/rcws_live/go.sh  (Ctrl-C to stop)
  STM32 link:            ${UART:-none (see warning above)}

EOF
