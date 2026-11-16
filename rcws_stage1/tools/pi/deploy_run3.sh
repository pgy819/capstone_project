#!/usr/bin/env bash
# RCWS Stage 1 - run 3 배포 스크립트 (Raspberry Pi 5에서 실행)
#   코드: github.com/pgy819/capstone_project 태그 stage1-v3.1-run2 (공개 저장소, 로그인 불필요)
#   설치 위치: ~/run3   (이전 배포본 ~/rcws 는 건드리지 않음)
#
#   bash deploy_run3.sh          # 받기 + 빌드 + 시험 + 실행 스크립트 생성
#   ~/run3_preview_stack.patch 가 있으면 자동 적용 (태그 코드에 preview-stack 커밋을 얹을 때)
#   ~/run3/go.sh                 # run 3 실행 (Ctrl-C로 종료하면 로그·클립 저장)
set -euo pipefail

TAG=stage1-v3.1-run2
REPO=https://github.com/pgy819/capstone_project.git
DEST="$HOME/run3"
SRC="$DEST/src"
RPI="$SRC/rcws_stage1/rpi"

say() { printf '\n\033[1;34m[run3]\033[0m %s\n' "$*"; }
die() { printf '\n\033[1;31m[run3] 오류:\033[0m %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- 1. 코드 받기
say "1/5 코드 받기 ($TAG)"
mkdir -p "$DEST/logs"
if [ -d "$SRC/.git" ]; then
  git -C "$SRC" fetch --depth 1 origin "refs/tags/$TAG:refs/tags/$TAG" --force
  git -C "$SRC" checkout -f --detach "$TAG"
  git -C "$SRC" clean -fdq   # 이전 패치가 만든 새 파일 제거 (build/ 는 .gitignore라 유지)
else
  git clone --branch "$TAG" --depth 1 "$REPO" "$SRC" \
    || die "GitHub에서 받지 못했습니다. Pi 인터넷 연결을 확인하거나, PC에서 zip으로 올리는 방법을 쓰세요."
fi
PATCH="$HOME/run3_preview_stack.patch"
VIEW=plain
TESTS=19
if [ -f "$PATCH" ]; then
  git -C "$SRC" apply --check "$PATCH" || die "패치가 코드와 맞지 않습니다: $PATCH"
  git -C "$SRC" apply "$PATCH"
  VIEW=stack
  TESTS=21
  say "패치 적용: $(basename "$PATCH") (미리보기 stack 보기)"
fi
COMMIT=$(git -C "$SRC" rev-parse --short HEAD)
if [ "$VIEW" = stack ]; then COMMIT="$COMMIT+preview-stack"; fi
say "코드 버전: $(git -C "$SRC" describe --tags --always) ($COMMIT)"

# ---------------------------------------------------------------- 2. 검출 모델(HEF)
say "2/5 검출 모델 찾기"
mkdir -p "$RPI/models"
HEF_DST="$RPI/models/yolov8s_h8.hef"
if [ ! -f "$HEF_DST" ]; then
  for c in "$HOME/rcws/rpi/models/yolov8s_h8.hef" \
           /usr/share/hailo-models/yolov8s_h8.hef \
           $(find "$HOME" -maxdepth 5 -name 'yolov8s*h8*.hef' -not -path "$DEST/*" 2>/dev/null); do
    if [ -f "$c" ]; then cp "$c" "$HEF_DST"; say "HEF 복사: $c"; break; fi
  done
fi
[ -f "$HEF_DST" ] || die "yolov8s_h8.hef를 찾지 못했습니다. run 2 때 쓰던 HEF를 $HEF_DST 로 복사한 뒤 다시 실행하세요."

# ---------------------------------------------------------------- 3. 빌드
say "3/5 빌드 (몇 분 걸립니다)"
cd "$RPI"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release | grep -E "rcws:" || true
cmake --build build -j4
# libcamera / Hailo가 빠진 빌드는 run 3에 쓸 수 없음 (개발 PC용 합성 빌드가 됨)
ldd build/rcws_vision | grep libhailort >/dev/null \
  || die "HailoRT 없이 빌드되었습니다. 'sudo apt install hailo-all' 후 rm -rf $RPI/build 하고 다시 실행하세요."
ldd build/rcws_vision | grep libcamera >/dev/null \
  || die "libcamera 없이 빌드되었습니다. 'sudo apt install libcamera-dev' 후 rm -rf $RPI/build 하고 다시 실행하세요."

# ---------------------------------------------------------------- 4. 시험
say "4/5 단위 시험 ($TESTS tests, 0 failed checks 이어야 함)"
# 진행 상황을 한 줄씩 보여 주고(Pi에서 몇 분 걸림), 전체 출력은 파일로 남김
./build/rcws_tests 2>&1 | tee "$DEST/tests.txt" | grep --line-buffered -E "^\[ RUN|^\[ FAIL|CHECK|tests,"  \
  || die "시험 실패. $DEST/tests.txt 내용과 함께 알려 주세요."
sudo setcap cap_sys_nice+ep build/rcws_vision || say "setcap 실패: 실시간 우선순위 없이 실행됩니다."

# ---------------------------------------------------------------- 5. 실행 스크립트
say "5/5 실행 스크립트 생성: $DEST/go.sh"
MEM_GB=$(awk '/MemTotal/ {printf "%d", $2/1048576}' /proc/meminfo)
CLIP=$([ "$MEM_GB" -ge 6 ] && echo 3 || echo 2)   # 클립 1초 ≈ 280 MB
cat > "$DEST/go.sh" <<EOF
#!/usr/bin/env bash
# run 3 실행. 인자로 설정 파일을 줄 수 있음: ~/run3/go.sh config/rcws_lin.ini
set -e
cd "$RPI"
CFG=\${1:-config/rcws.ini}
STAMP=\$(date +%m%d_%H%M%S)
NAME=run3_\$(basename "\$CFG" .ini)_\$STAMP
echo "[run3] code $TAG ($COMMIT), config \$CFG, log $DEST/logs/\$NAME.rlog"
thr() {  # 0x0 이 아니면 경고: 0x1/0x10000 저전압, 0x4/0x40000 스로틀 (앞자리 = 부팅 후 이력)
  local t; t=\$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
  echo "[run3] throttled=\$t \$([ "\$t" = 0x0 ] && echo OK || echo '!! 저전압/스로틀 이력: 이 실행의 지연값은 P2 판정에 쓰지 말 것')"
}
thr
RC=0
trap ':' INT   # Ctrl-C는 rcws_vision만 끝내고, 아래 정리·요약은 계속 실행
./build/rcws_vision --config "\$CFG" --source camera --detector hailo \\
    --log "$DEST/logs/\$NAME.rlog" --clip-seconds $CLIP --preview 8080 --preview-view $VIEW || RC=\$?
trap - INT
echo "[run3] rcws_vision exit=\$RC  (0 정상, 130 Ctrl-C, 139 세그폴트, 137 강제 종료, 1/2 오류 메시지 참조)"
mv -f run.rclip "$DEST/logs/\$NAME.rclip" 2>/dev/null || true
thr
if [ -f "$DEST/logs/\$NAME.rlog" ]; then
  python3 "$SRC/rcws_stage1/tools/rcws_log.py" "$DEST/logs/\$NAME.rlog" | tee "$DEST/logs/\$NAME.txt"
  echo "[run3] 저장: $DEST/logs/\$NAME.{rlog,rclip,txt}"
else
  echo "[run3] 로그가 없습니다: rcws_vision이 시작 단계에서 끝났습니다(위 exit 값 참조)."
fi
EOF
chmod +x "$DEST/go.sh"

IP=$(hostname -I | awk '{print $1}')
say "준비 완료. RAM ${MEM_GB} GB → 클립 ${CLIP}초"
cat <<EOF

  실행:      ~/run3/go.sh
  미리보기:  PC 브라우저에서 http://$IP:8080/
  종료:      Ctrl-C (마지막 ${CLIP}초 원본 프레임과 로그가 ~/run3/logs 에 저장됨)

  run 3 조건(docs/field_runs): 1~2 m 거리, 손에 든 폰, 30~60초
EOF
