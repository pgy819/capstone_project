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
./build/rcws_tests                       # 35 tests, 0 failed checks 이어야 함 (v4: 시나리오·트랙 보드·게이트 14개 추가)
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

**실시간 미리보기.** `--preview 8080`이면 PC 브라우저에서 `http://<pi>:8080/`, `--window`면 Pi 데스크톱 팝업으로 본다. `--preview-view stack`을 더하면 세 화면을 가로로 붙여 보낸다(1281×360).

| 화면 | 내용 | 보는 법 |
|---|---|---|
| 왼쪽 | 원본(lores) + 검출 박스, 추적 박스, 마지막 검출 영역 | 기존 화면과 같음 |
| 가운데 | **검출기가 실제로 받은 입력**(640×640, 확대·축소·회색 패딩 포함)과 그 결과 박스, 추적 박스 | 제목: 요청 종류, 프레임 번호, 전처리 경로(`lores plane` 등), 원본 크롭 크기, 배율. 근거리 대형 ROI가 너무 줄었는지, 레터박스·경로 A/B가 의도대로인지 확인 |
| 오른쪽 | **NCC** 템플릿, 탐색창(일치 위치 사각형), ZNCC 상관 지도(파랑 −1 … 빨강 +1, 흰 십자 최댓값, 자홍 X 두 번째 봉우리) | 제목: rho, 2nd, 배율, OK/FAIL. 지도가 넓게 빨가면 질감 부족, 봉우리가 둘이면 유사 무늬. 2프레임 넘게 갱신이 없으면 흐리게 표시 |

비전 스레드는 버퍼 참조만 넘기고(템플릿만 복사, 최대 96×96), 그리기·인코딩은 미리보기 스레드(Core 0)가 한다. `plain`(기본)이면 아무것도 보관하지 않아 v3.1과 같다.

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

## 6. v3 변경점 — YOLO 검출 성능 튜닝

v2(이 폴더의 직전 버전, `../v2`)와 같은 구조에 아래만 추가했다. 런타임 동작 변경은 모두 설정 플래그로 끌 수 있다.

| 항목 | 위치 | 기본값 | v2로 되돌리기 |
|---|---|---|---|
| ROI 전처리: YUV 평면에서 먼저 축소 후 RGB 변환 (크롭 > 모델 입력일 때만) | `detect/detector.cpp` `make_request_i420`, `perception.cpp` | `detector.preproc_i420 = true` | `false` |
| 1–2배 축소에 bilinear 사용 (INTER_AREA 비정수 축소가 병목) | 같은 함수 | `detector.linear_mild_downscale = false` | 기본이 v2와 같음 |
| 모델 입력보다 넓은 ROI는 lores에서 크롭 (run 2 수정) | `perception.cpp` `submit_main_crop` | `detector.lores_large_roi = true` | `false`: lores는 해상도 손실이 없을 때만, 나머지는 메인 + 위 전처리 |
| 맞춤 2-클래스 모델 데이터셋 추출 (ROI 크롭 분포 + NCC 보강 + 캘리브레이션 셋) | `tools/model/export_dataset.py` | 오프라인 | — |
| YOLOv8 미세조정 + ONNX 출력 | `tools/model/train_yolo.py` | 오프라인 (GPU PC) | — |
| 도메인 캘리브레이션 HEF 컴파일 | `tools/model/hailo/compile_hef.sh` | 오프라인 (Hailo SW Suite) | — |
| 맞춤 HEF용 설정 | `config/rcws_custom2cls.ini` | — | `rcws.ini` 사용 |
| 검출기 A/B 지표 (검출률·중심 오차·정지 지터·처리시간, 나빠지면 WORSE 표시) | `tools/det_report.py` | — | — |

개발 PC(x86) 측정, 864×864 ROI → 640×640: v2 3.37 ms, v3 2.44 ms(×1.38), v3+bilinear 0.68 ms(×4.97). v2 대비 픽셀 평균차 0.07, 표적 중심 역사상 차이 ≤ 0.04 px. **Pi 5 실측은 아직 없다.**

### A/B 절차 (같은 프레임으로 비교)
```bash
# 1) 실기 녹화 (클립 + 로그)
./build/rcws_vision --config config/rcws.ini --source camera --detector hailo --log base.rlog --clip-seconds 2
# 2) 같은 클립에 대해 설정만 바꿔 NPU로 재검출 (--detector replay 는 기록된 박스를 재주입하므로 검출기 비교 불가)
./build/rcws_vision --config config/rcws.ini        --source replay:run.rclip --detector hailo --log a.rlog
./build/rcws_vision --config config/rcws_lin.ini    --source replay:run.rclip --detector hailo --log b.rlog
python3 tools/det_report.py a.rlog b.rlog
```

### 맞춤 모델 파이프라인 — 보류 (사용하지 않음)
> 개발 제약(기획서 V2.6 부록 B.11): 모델 학습·HEF 재컴파일과 추가 구매가 필요한 선택은 제외한다. 검출기는 Model Zoo COCO HEF로 고정하고, `tools/model/`과 `config/rcws_custom2cls.ini`는 보존만 한다. 검출 개선은 설정 A/B(임계값, `roi.k_s`, 검출 주기, 전처리 플래그)와 Model Zoo 사전 컴파일 HEF 교체로 한정한다.

```bash
python3 tools/model/export_dataset.py --run run1.rclip:run1.rlog --run run2.rclip:run2.rlog \
        -o ds --ncc-fill --roi-crops 3 --calib 1024 --blur 0.2      # Pi 또는 PC
python3 tools/model/train_yolo.py ds/data.yaml --model yolov8s.pt     # GPU PC
tools/model/hailo/compile_hef.sh runs/rcws/rcws_2cls/weights/best.onnx ds/calib   # Hailo SW Suite
cp rcws_2cls_h8.hef rpi/models/ && ./build/rcws_vision --config config/rcws_custom2cls.ini ...
python3 tools/tests/test_tools.py     # 도구 자체 시험 (numpy + opencv-python만 필요)
```
세션 분할: 녹화 1개 = 촬영 세션 1개. `--run`을 둘 이상 주면 마지막(또는 `--val-run`) 세션이 통째로 검증에 간다. 유사 물체만 찍은 녹화는 `--neg-run decoys.rclip`으로 라벨 없이 넣는다. 설계 근거: 상세설계기획서 V2.6 부록 B.6.

라벨 품질: 모니터 시험대에서 TRUTH를 기록한 녹화가 가장 정확하다. 검출기 박스만으로 만든 의사 라벨은 COCO 모델의 편향을 물려받으므로 `--det-thr`를 높게 두고, `--ncc-fill`로 YOLO가 놓친 프레임(어려운 양성 샘플)을 보강한다.

## 7. v4 — 추적 시나리오 대응 (A~J)

조건은 그대로다: 모델 학습·HEF 재컴파일 없음, 추가 구매 없음, 기존 구조(단일 표적 상태 머신 + NCC + IMM) 유지. 그 위에 **다중 트랙 보드**, **화질 게이트**, **조준 출력 게이트**를 얹고, STM32에는 **동작 제한·안전 감시기**를 넣었다. 모두 설정으로 끌 수 있다(`mot.enabled`, `quality.enabled`, `track.size_jump = 0`).

```
검출 배치 ─▶ 트랙 보드(mot.cpp) ─▶ 각 검출에 ID ─▶ Perception(기존 단일 표적 루프)
               │  칼만(CV) + 헝가리안, 2단계 할당      │  잠금 ID 외 확정 트랙은 측정에서 제외
               │  색 히스토그램 재식별               │  크기 급변 거부, 재고정 차단
               │  상태: TENT/CONF/OVERLAP/OCCL/      ▼
               │        EXIT/AMBIG/LOST             조준 출력 게이트(aim.cpp) ─▶ FOLLOW / PREDICT / HOLD + 이유
               └ 우선순위 점수 + 히스테리시스           ▲
화질 감시(quality.cpp): DARK / GLARE / BLUR ──────────┘
```

| 상황 | 반영 | 위치 |
|---|---|---|
| A 다중 표적 | 휴대폰 검출마다 트랙 ID. 우선순위 = 0.35 화면 중심 + 0.20 크기 + 0.10 접근(박스 크기 증가율) + 0.20 검출 신뢰도 + 0.15 추적 시간. 지정 표적은 도전자가 15 % 이상 높은 상태가 0.3 s 이어지고 현재 표적을 1 s 이상 유지했을 때만 바뀐다. 추적 중에도 30프레임마다 전체 화면을 한 번 검출해 다른 표적을 놓치지 않는다 | `perception/mot.cpp` `designate`, `perception.cpp` `schedule` |
| B 중첩 | 예측 박스가 겹치면 OVERLAP. 두 휴대폰이 한 박스로 검출되면 크기 급변(√면적비 > 2)으로 거부. 분리된 뒤 색으로 구별이 안 되면 AMBIGUOUS 1 s → 조준 출력 HOLD | `mot.cpp` `check_separation`, `perception.cpp` |
| C 가림 | 짧은 가림(≤ 0.5 s)은 OCCLUDED로 예측 유지, 길면 LOST로 3 s 보관. 다시 보이면 크기·위치·**색 히스토그램**(U×V 8×8 + 휘도 8, 바타차리야 거리)으로 같은 ID로 재식별. 박스 크기와 관계없이 1600개 표본만 읽는다 | `mot.cpp` `box_histogram`, 재식별 블록 |
| D 화면 밖 | 가장자리 4 % 띠에서 바깥으로 움직이면 EXITING, 사라지면 LOST(나간 방향 기록). Perception은 REACQ 동안 **다른 표적으로 바꾸지 않고** HOLD + 나간 방향 힌트를 낸다(짐벌 제한 탐색용) | `mot.cpp`, `aim.cpp`, `perception.cpp` `other_target` |
| E 카메라 움직임 | 트랙 보드 예측 전에 짐벌 각 변화로 생긴 화면 이동(−fx·Δψ, +fy·Δθ)을 뺀다. 지금은 짐벌 각이 상수라 0, 엔코더 회신(U3, U5)이 붙으면 그대로 동작 | `perception.cpp` `handle_batch` |
| F 오탐 | 기존 SPRT·게이트에 더해: 크기 급변 거부, 미확정(TENTATIVE) 트랙은 COAST/REACQ에서 표적을 되살리지 못함, 게이트 밖 강검출로 재고정할 때 확정된 트랙이어야 함 | `perception.cpp` |
| G 조명 | 4프레임마다 lores 평균 휘도·포화 비율, 표적 선명도(라플라시안)/기준선. DARK·GLARE·BLUR는 3회 연속이면 켜지고 6회 연속 정상이면 꺼진다 → 조준 출력 HOLD | `perception/quality.cpp` |
| H 거리·크기 | 기존 역거리 필터·ROI 재검출 유지 + 크기 급변 거부 | — |
| I 조준 제어 | Pi: TRACK → FOLLOW, COAST 0.15 s까지 PREDICT, 그 뒤 HOLD(마지막 속도로 계속 돌지 않음). STM32: 데드밴드 3 px, 속도 300°/s, 가속 3000°/s², 정지 감속 6000°/s², 목표값을 EEPROM 소프트 제한(626~3470)에 묶고 한계에서 속도 0(와인드업 방지, K8 수정) | `aim.cpp`, `stm32_fw/src/servo_guard.c`, `main.c` |
| J 안전 | STM32 감시기: 패킷 0.5 s 끊김 → HOLD, 10 s → 토크 해제, 다시 오면 1 s 저속 복귀(RAMP). **Nucleo 파란 버튼(B1) = 긴급 정지**(토크 해제, RESET까지 유지). LD2: 켜짐 RUN, 1 Hz HOLD, 2.5 Hz 통신 끊김, 5 Hz RAMP, 2 s마다 짧게 TORQUE_OFF, 10 Hz ESTOP | `servo_guard.c`, `main.c` |

발사·교전 판단은 이 작품의 범위가 아니어서 넣지 않았다. J는 기구와 주변 사람을 지키는 정지 동작만 다룬다.

### 시나리오 실행 (시연 영상용)
```bash
./build/rcws_vision --source synthetic --detector oracle --scenario cross --preview 8080      # 브라우저로 보기
./build/rcws_vision --source synthetic --detector oracle --scenario exit --record rec_exit       # 30 fps 프레임 저장
ffmpeg -framerate 30 -i rec_exit/f_%05d.jpg -pix_fmt yuv420p exit.mp4
python3 tools/rcws_log.py run.rlog       # 조준 출력 점유율·HOLD 이유, 트랙 상태 전이, 화질 플래그 요약
```
시나리오: `sine parallel cross merge pillar longocc exit dark clutter`. 화면에는 트랙 ID·상태·우선순위, 잠금(자홍 이중 박스), 지정 표적(`*`), 나간 방향 화살표, 셋째 줄에 조준 출력과 HOLD 이유·화질 플래그가 나온다.

### 시험 결과 (개발 PC, 합성 장면)
`./build/rcws_tests` 35개(기존 21 + 시나리오·보드·게이트 14) 모두 통과. STM32 감시기는 `stm32_fw/host_test/test_guard.c`(호스트 C 시험).

시드 10개 반복(`RCWS_SWEEP=10 ./build/rcws_tests`, 장면 배경·검출 잡음·누락·오검출이 매번 다름):

| 시나리오 | 내용 | 표적이 다른 휴대폰으로 넘어간 실행 | 표적 트랙 ID가 바뀐 실행 |
|---|---|---|---|
| parallel | 두 휴대폰 나란히 이동 | 0 / 10 | 0 / 10 |
| cross | 색이 다른 두 휴대폰 교차 | 0 / 10 | 0 / 10 |
| merge | 같은 모양 두 휴대폰 완전 중첩 후 분리 | 0 / 10 | 0 / 10 (분리 때 AMBIGUOUS → HOLD 1 s) |
| pillar | 기둥 뒤 짧은 가림 + 다른 휴대폰 | 0 / 10 | 0 / 10 |
| longocc | 1.5 s 가림 → 색으로 재식별 | 0 / 10 | 0 / 10 |
| exit | 오른쪽으로 나갔다 1.2 s 뒤 복귀, 다른 휴대폰은 화면에 남음 | 0 / 10 | 0 / 10 |
| dark | 1 s 소등 | 0 / 10 | 1 / 10 (어두운 동안 놓쳐 새 ID, 표적은 같음) |
| clutter | 유사 물체 + 오검출 8 %/요청 | 0 / 10 | 0 / 10 |

한계:
- 같은 색·같은 모양의 휴대폰이 완전히 겹쳤다 갈라지면(merge) 어느 쪽이 원래 표적인지 영상만으로는 보장할 수 없다. 합성 장면에서는 두 휴대폰이 등속으로 지나가서 칼만 예측이 대부분 구별해 내지만, 겹친 채 멈추거나 방향을 바꾸면 틀릴 수 있다. 그래서 분리 직후 AMBIGUOUS로 표시하고 1초 동안 조준 출력을 멈추는 것이 이 기능의 목표다.
- 채점 방법: 두 휴대폰 중심이 박스 크기보다 가까운 프레임은 어느 쪽을 따라가는지 기하로 판정할 수 없어 집계에서 뺐다.
- 합성 장면은 실제 카메라의 블러·조명·검출기 특성과 다르다. Pi 5 현장 실행(run 3)으로 다시 확인해야 한다. 특히 COCO 'cell phone' 점수가 낮은 실제 검출기에서 트랙 확정(3회)과 재식별 문턱(색 거리 0.35)을 조정해야 할 수 있다.
