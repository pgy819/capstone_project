# RCWS Stage 1 — Step 1: Pi 5 객체 인식·추적 런타임

설계 기준: 상세 설계 기획서 V2.5 §3(Flow 1), §5–7(시간·역투영·추정), §12(개발 순서). Step 1은 **짐벌 고정** 상태에서 카메라와 Pi 5만으로 타겟을 검출·추적하고, 추적 파라미터를 로그로 확인하는 단계다. STM32 관련 코드는 Step 2 이후에 붙는다.

```
rcws/
  rpi/                       C++17 (CMake)
    src/common/              타입, 설정(INI), 시계, SPSC 큐, 스레드 배치
    src/camera/              libcamera 소스 (main 1536x864 + lores 640x360, AE off, 수동 초점)
    src/detect/              비동기 검출기 인터페이스, HailoRT 검출기, 전처리/좌표 역사상
    src/perception/          ZNCC 서브픽셀 추적, ROI/타일, 트랙 상태기계 + SPRT, 게인 제어
    src/estimation/          카메라 모델(composeLOS·시차), IMM(CV+CA), 역거리 필터
    src/record/              바이너리 로그(.rlog), RAM 클립 기록·재생(.rclip)
    src/sim/                 합성 장면 + 오라클 검출기 (하드웨어 없이 개발·시험)
    src/app/main.cpp         rcws_vision 실행 파일
    tests/                   단위·통합 시험 14개
    config/rcws.ini          런타임 설정 ([measure] 표시는 보정값으로 교체)
  tools/                     Python 분석 도구
    rcws_log.py              로그 리더 + 요약
    track_plot.py            상태·각도·오차·ZNCC 품질·처리시간 그래프
    calibrate_camera.py      체커보드 내부 파라미터 보정
```

## 1. 빌드 (Raspberry Pi 5, Raspberry Pi OS Bookworm 64-bit)

```bash
sudo apt install -y cmake g++ pkg-config libopencv-dev libcamera-dev hailo-all python3-opencv python3-matplotlib
cd rcws/rpi
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/rcws_tests                       # 14 tests, 0 failed checks 이어야 함
sudo setcap cap_sys_nice+ep build/rcws_vision   # SCHED_FIFO 허용 (Core 1, 우선순위 70)
```

CMake 출력에서 `libcamera source ON`, `HailoRT detector ON`을 확인한다. HailoRT는 **4.x 계열(4.20 이상)**이어야 한다. Hailo-8은 5.x 계열의 대상이 아니다. 개발 PC(x86)에서도 libcamera·HailoRT 없이 빌드되며, 이때는 합성 장면으로 전체 파이프라인을 돌릴 수 있다.

**검출 모델.** 맞춤 2-클래스 HEF(§3.4, §3.11)를 학습하기 전까지는 Hailo Model Zoo의 COCO 사전학습 YOLOv8s HEF(Hailo-8용, NMS 포함)를 `models/yolov8s_h8.hef`로 둔다. COCO 클래스 0 = person, 67 = cell phone이며 `config/rcws.ini`의 `class_person`, `class_phone`에 이미 맞춰져 있다. 입력 크기는 HEF에서 자동으로 읽는다(Model Zoo 기본 640×640).

## 2. 실행

```bash
# 실기: 카메라 + Hailo-8, 로그 기록, 마지막 3초 원본 프레임을 RAM에 보관했다가 종료 시 저장
./build/rcws_vision --config config/rcws.ini --source camera --detector hailo \
                    --log run.rlog --clip-seconds 3

# 하드웨어 없이: 합성 장면 + 오라클 검출기 (참값 TRUTH가 로그에 함께 기록됨)
./build/rcws_vision --config config/rcws.ini --source synthetic --detector oracle --frames 1200 --log syn.rlog

# 재생: 기록한 클립과 그때의 검출 결과로 같은 파이프라인을 다시 실행 (A/B 비교)
./build/rcws_vision --config config/rcws.ini --source replay:run.rclip --detector replay:run.rlog --log replay.rlog
```

콘솔에는 주기적으로 상태, 처리율, 처리시간(평균/최대), 프레임 누락, 추정 시선각·각속도·거리가 출력된다. Ctrl-C로 끝내면 클립을 저장하고 통계를 출력한다.

**클립 메모리.** 1536×864 YUV420 + lores = 약 2.3 MB/프레임이므로 3초(360프레임) ≈ 840 MB를 미리 잡는다. Pi 5 4 GB 모델에서는 2초 이하를 권장한다.

## 3. 로그 분석 (Python)

```bash
cd rcws/tools
python3 rcws_log.py run.rlog          # 프레임 간격·누락, 처리시간 p50/p99, 상태 점유율·전이, 검출 지연, 측정 채택률
python3 track_plot.py run.rlog        # run.png: 상태/각도/오차/ZNCC/처리시간 그래프
```

로그에 참값(TRUTH)이 있으면(합성 장면, 이후 모니터 시험대) 추정 오차 RMS·p95[mrad]와 측정 오차[px]가 출력된다. `rcws_log.read_log()`는 numpy 구조체 배열을 반환하므로 노트북에서 바로 분석할 수 있다.

## 4. Step 1 보정 절차 (기획서 §12.3의 1–4번)

| # | 항목 | 방법 | 반영 위치 |
|---|---|---|---|
| 1 | 초점 | 1.2 m 표적에서 `LensPosition`을 0.5–1.2 범위로 스윕하며 선명도(라플라시안 분산) 최대값 선택 | `camera.lens_position` |
| 2 | 내부 파라미터 | 아래 명령으로 체커보드 30장 이상(화면 모서리 포함) 촬영 후 `calibrate_camera.py` | `[intrinsics]` |
| 3 | 노광 시각 부호 σ | LED를 알려진 시각에 점멸시키고 행별 점등 시각을 비교 | `camera.timestamp_sign` |
| 4 | 행 시간 | `v4l2-ctl -d /dev/v4l-subdevN --list-ctrls`의 `pixel_rate`, `horizontal_blanking`으로 계산 후 3번 시험으로 교차 확인 | `camera.line_time_ns` |

```bash
# 2번용 촬영: 런타임과 같은 센서 모드·초점
rpicam-still --mode 1536:864:10 --width 1536 --height 864 --autofocus-mode manual \
             --lens-position 0.81 --timelapse 1500 -t 60000 -o calib_%03d.jpg
python3 tools/calibrate_camera.py "calib_*.jpg" --cols 9 --rows 6 --square 0.025
```

합격 기준은 재투영 RMS ≤ 0.3 px이다. 가장자리 오차가 크면 `--model rational`로 비교한다.

## 5. 구현 상태

시험 결과는 개발 PC(x86)의 합성 장면 기준이다. **Pi 5 실기 빌드·실행은 아직 하지 않았다.**

| 기능 | 기획서 | 상태 |
|---|---|---|
| main + lores 이중 스트림, AE off, 수동 초점, 120 fps 모드 강제 | §3.2–3.3 | 코드 (libcamera 0.2 헤더로 빌드 확인) |
| SensorTimestamp BOOTTIME→MONOTONIC 변환, 행 단위 t_cap | §5.1 | 코드 |
| HailoRT 비동기 검출, NMS by-class 파싱, 시작 시 출력 형식 검증 | §3.4 | 코드 (HailoRT 4.23 헤더로 구문 확인) |
| 스케일 정규화 ROI, 3×3 타일, 사람→손 영역 탐색 | §3.5, §3.13.1 | 코드·시험 |
| ZNCC 서브픽셀 추적, 드리프트 방지, 앵커 편향 추정 | §3.6 | 코드·시험 (변위 RMS 0.09 px) |
| 트랙 상태기계 5상태 + SPRT 획득 | §3.8, §3.13.2 | 코드·시험 |
| composeLOS(틸트 합성·보어사이트·시차), IMM(CV+CA), 역거리 | §6–7 | 코드·시험 |
| ROI 기준 게인 제어, 상태별 노광 프리셋 | §3.2, §3.13.5 | 코드 |
| 신뢰도 기반 검출 스케줄링 | §3.13.10 | 코드 |
| 바이너리 로그, RAM 클립, 결정론적 재생 | §12.4 | 코드·시험 (재생 상태 불일치 0) |
| ROI 시간 평균, ROI 국소 왜곡 보정, 배경 위상상관 | §3.13.4/6/9 | **미구현** (설정 자리만 있음) |
| 사람 대리 추적(휴대폰이 화소 한계일 때) | §3.13.1 | **미구현** (사람 단서로 탐색 위치만 좁힘) |
| Stop-and-Stare, 짐벌 속도 게이팅 | §3.13.3 | Step 3 (짐벌 구동 후) |
| 맞춤 2-클래스 HEF 학습 파이프라인 | §3.11 | **미포함** (데이터 수집부터 필요) |

합성 장면 기준 결과(약 14°/s 사인 궤적, 오라클 검출기 박스 잡음 σ = 1.5 px): 획득 0.04 s, TRACK 중 추정 오차 RMS 약 0.3 mrad, 0.35 s 가림 후 즉시 복귀, 유사 물체(디코이)에 잠기지 않음. 실제 카메라에서는 블러·조명·검출기 특성이 달라지므로 P2–P4 실측으로 다시 확인해야 한다.
