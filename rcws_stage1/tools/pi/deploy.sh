#!/usr/bin/env bash
# RCWS Stage 1 - Raspberry Pi 5 deploy: packet v2 link to the STM32 + operator web page + launcher.
#
#   UART: /dev/ttyAMA0 (GPIO14/15, needs dtparam=uart0=on). Override: RCWS_UART=/dev/xxx bash deploy.sh
#   PC:  ssh -t pgy819@rcwspi.local "curl -fsSL https://raw.githubusercontent.com/pgy819/capstone_project/preview-stack/rcws_stage1/tools/pi/deploy.sh | bash -s preview-stack"
#   Pi:  bash deploy.sh [branch]        (default: preview-stack)
#
# Installs into ~/rcws_live (earlier deployments ~/rcws, ~/run3 are left alone):
#   ~/rcws_live/src         git checkout
#   ~/rcws_live/go.sh       one run: camera + Hailo + STM32 link + web page on :8080 (Ctrl-C saves log/clip)
#   ~/rcws_live/logs        rlog / rclip / summary per run
#   launcher                http://<pi>:8000/  start / stop without SSH (systemd user service)
set -euo pipefail

BRANCH=${1:-preview-stack}
REPO=https://github.com/pgy819/capstone_project.git
DEST="$HOME/rcws_live"
SRC="$DEST/src"
RPI="$SRC/rcws_stage1/rpi"

say() { printf '\n\033[1;34m[rcws]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[rcws] 주의:\033[0m %s\n' "$*"; }
die() { printf '\n\033[1;31m[rcws] 오류:\033[0m %s\n' "$*" >&2; exit 1; }

# ------------------------------------------------------------------ 1. code
say "1/6 코드 받기 ($BRANCH)"
mkdir -p "$DEST/logs"
if [ -d "$SRC/.git" ]; then
  git -C "$SRC" fetch --depth 1 origin "$BRANCH"
  git -C "$SRC" checkout -f --detach FETCH_HEAD
  git -C "$SRC" clean -fdq
else
  git clone --branch "$BRANCH" --depth 1 "$REPO" "$SRC" || die "GitHub에서 받지 못했습니다 (브랜치 $BRANCH 푸시 여부, Pi 인터넷 확인)."
fi
COMMIT=$(git -C "$SRC" rev-parse --short HEAD)
say "코드 버전: $COMMIT"

# ------------------------------------------------------------------ 2. model
say "2/6 검출 모델(HEF)"
mkdir -p "$RPI/models"
HEF="$RPI/models/yolov8s_h8.hef"
if [ ! -f "$HEF" ]; then
  for c in "$HOME/run3/src/rcws_stage1/rpi/models/yolov8s_h8.hef" "$HOME/rcws/rpi/models/yolov8s_h8.hef" \
           /usr/share/hailo-models/yolov8s_h8.hef; do
    if [ -f "$c" ]; then cp "$c" "$HEF"; say "HEF 복사: $c"; break; fi
  done
fi
[ -f "$HEF" ] || die "yolov8s_h8.hef가 없습니다. $HEF 로 복사한 뒤 다시 실행하세요."

# ------------------------------------------------------------------ 3. build
say "3/6 빌드 (몇 분)"
cd "$RPI"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release | grep -E "rcws:" || true
cmake --build build -j4
ldd build/rcws_vision | grep libhailort >/dev/null || die "HailoRT 없이 빌드됨: sudo apt install hailo-all, rm -rf $RPI/build 후 다시."
ldd build/rcws_vision | grep libcamera >/dev/null || die "libcamera 없이 빌드됨: sudo apt install libcamera-dev, rm -rf $RPI/build 후 다시."
sudo setcap cap_sys_nice+ep build/rcws_vision || warn "setcap 실패: 실시간 우선순위 없이 실행됩니다."

# ------------------------------------------------------------------ 4. tests
say "4/6 단위 시험"
./build/rcws_tests 2>&1 | tee "$DEST/tests.txt" | grep --line-buffered -E "^\[ FAIL|CHECK|tests," || true
if ! grep -q " 0 failed checks" "$DEST/tests.txt"; then
  if [ "${RCWS_ALLOW_TEST_FAIL:-0}" = 1 ]; then
    warn "시험 실패를 무시하고 계속합니다 (RCWS_ALLOW_TEST_FAIL=1). 실패 내용: $DEST/tests.txt"
  else
    die "시험 실패. $DEST/tests.txt 를 알려 주세요. (실패를 알고 계속하려면 RCWS_ALLOW_TEST_FAIL=1)"
  fi
fi

# ------------------------------------------------------------------ 5. UART to the STM32
say "5/6 STM32 UART 점검 (팀 배선: Pi GPIO14 TXD -> STM32 PC7, GPIO15 RXD <- PC6, GND 공통)"
# Pi 5: GPIO14/15 = PL011 UART0 = /dev/ttyAMA0, only with 'dtparam=uart0=on'.
#        /dev/serial0 defaults to /dev/ttyAMA10 = the 3-pin DEBUG connector, NOT the GPIO header.
UART="${RCWS_UART:-}"
if [ -z "$UART" ] && [ -e /dev/ttyAMA0 ]; then UART=/dev/ttyAMA0; fi
if [ -n "$UART" ]; then
  say "UART: $UART (GPIO14/15)"
else
  warn "/dev/ttyAMA0(GPIO14/15)이 없습니다. /dev/serial0은 $(readlink -f /dev/serial0 2>/dev/null || echo 없음) 로,"
  warn "Pi 5에서는 GPIO가 아니라 디버그 커넥터(ttyAMA10)입니다. 다음을 한 뒤 재부팅하고 deploy를 다시 실행하세요:"
  warn "  echo 'dtparam=uart0=on' | sudo tee -a /boot/firmware/config.txt && sudo reboot"
  warn "그때까지 go.sh는 STM32 없이(서보 구동 없이) 실행됩니다."
fi
if [ -n "$UART" ] && grep -Eq "console=(ttyAMA0|$(basename "$UART"))" /proc/cmdline; then
  warn "직렬 콘솔이 $UART 를 쓰고 있습니다(부팅 메시지가 STM32로 감). /boot/firmware/cmdline.txt 에서 console=ttyAMA0,... 를 지우고 재부팅하세요."
fi
id -nG | grep -qw dialout || { warn "사용자가 dialout 그룹이 아닙니다: sudo usermod -aG dialout $USER 후 재로그인."; }

# ------------------------------------------------------------------ 6. run script + launcher
say "6/6 실행 스크립트와 실행기(:8000)"
MEM_GB=$(awk '/MemTotal/ {printf "%d", $2/1048576}' /proc/meminfo)
CLIP=$([ "$MEM_GB" -ge 6 ] && echo 3 || echo 2)
UART_ARGS=""
[ -n "$UART" ] && UART_ARGS="--uart $UART --uart-baud 115200"
cat > "$DEST/go.sh" <<EOS
#!/usr/bin/env bash
# one tracking run (from the launcher page or by hand). Ctrl-C / web "종료" saves log and clip.
cd "$RPI"
CFG=\${1:-config/rcws.ini}
NAME=live_\$(date +%m%d_%H%M%S)
echo "[rcws] code $COMMIT, config \$CFG, log $DEST/logs/\$NAME.rlog"
thr=\$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
echo "[rcws] throttled=\$thr \$([ "\$thr" = 0x0 ] && echo OK || echo '!! 저전압/스로틀 이력')"
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
  || warn "linger 설정 실패: 로그인해야 실행기가 켜집니다."

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
SCREEN_NOTE="모니터: 다음 데스크톱 로그인부터 자동, 지금 띄우기: DISPLAY=:0 ~/rcws_live/screen.sh"
if ! command -v chromium-browser >/dev/null && ! command -v chromium >/dev/null && ! command -v firefox >/dev/null; then
  SCREEN_NOTE="모니터: 브라우저 없음 (sudo apt install chromium-browser)"
fi

IP=$(hostname -I | awk '{print $1}')
say "완료 ($COMMIT)"
cat <<EOF

  실행기(시작/종료):  http://$IP:8000/     (부팅하면 자동으로 켜짐)
  조작 화면:          http://$IP:8080/     (추적 실행 중일 때, 노트북·휴대폰 여러 대 동시 가능)
  Pi 화면:            $SCREEN_NOTE
  직접 실행:          ~/rcws_live/go.sh    (Ctrl-C로 종료)
  STM32 링크:         ${UART:-없음 (위 주의 참조)}

EOF
