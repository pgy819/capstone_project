# 5축 시선 안정화 시스템 — Stage 1 (RCWS)

스마트폰 같은 표적을 카메라로 인식해 실시간으로 추적·조준하는 2축(Pan/Tilt) 카메라 짐벌 프로토타입입니다. 이후 하단에 BNO085 IMU 기반 3축 평형 플랫폼을 더해 5축 시선 안정화 시스템으로 확장합니다.

| 구성 | 사양 | 역할 |
| :--- | :--- | :--- |
| 상위 제어기 | Raspberry Pi 5 + Hailo-8 NPU, Camera Module 3 Wide | 표적 검출(YOLOv8s), ZNCC 추적, IMM 상태 추정, 시선각 산출 |
| 하위 제어기 | STM32 NUCLEO-F401RE (Cortex-M4, FPU) | DMA 링버퍼 수신, TIM2 500 Hz 제어 루프 |
| 구동기 | Feetech STS3215 × 2 | Pan ID `1`, Tilt ID `2` (1 Mbps 직렬 버스, 4096 스텝) |
| 기구 | 3D 프린팅 2축 Pan-Tilt 짐벌 | 6806 베어링 지지, 유격 없는 출력물 커플러 |

## 폴더 구성

```text
capstone_project/
├── docs/STAGE1_SPEC.md        Stage 1 수학 모델·프로토콜 명세 (v1 기준)
├── rcws_stage1/
│   ├── README.md              비전 런타임 빌드·실행·보정 절차
│   ├── GEMINI.md              AI 에이전트 규칙, 하드웨어 계약
│   ├── rpi/                   [Raspberry Pi 5] C++20 비전 런타임 (CMake)
│   ├── tools/                 Python 로그 분석·보정·검출 A/B 도구
│   └── stm32_fw/              [STM32F401RE] 500 Hz 모터 제어 펌웨어 (PlatformIO)
└── mechanical/                상단부 2축 짐벌 기구 (CadQuery → CATIA STEP / 3D 프린팅 STL)
```

## 개발 기록

버전마다 무엇을 왜 바꿨는지는 각 커밋 본문에 적혀 있습니다. 태그로 버전 사이를 비교할 수 있습니다(예: `stage1-v2-cpp...stage1-v3-det-tuning`).

| 태그 | 날짜 | 내용 | 바꾼 이유 |
| :--- | :--- | :--- | :--- |
| `stage1-v1-python` | 10/12 | Python 비전 + 2D 등속도 칼만 + 8B UART + STM32 PD | Stage 1 명세 1차 구현 |
| `stage1-v2-cpp` | 10/27 | C++ 비전 런타임으로 교체, 현장 run 1 수정 포함 | Python GIL·GC 지터가 120 fps 예산을 넘음, Ego-motion·광각 왜곡·급기동 대응 |
| (브랜치 `field-run2-fixes`) | 10/28 | 현장 run 2 수정: NCC 캐시, lores ROI, q 상향, 강검출 재고정 | 실효 60 fps, NCC 성공 2%, 검출 47% 게이트 탈락 |
| `mech-v0.4` | 10/30–11/2 | 짐벌 기구 v0.1 → v0.4 | 서보 축 부하 해소, 설계공간 탐색, 3D 프린팅·구매 최소화 |
| `stage1-v3-det-tuning` | 11/4 | YOLO 전처리 고속화, 검출기 A/B 도구 | 근거리 대형 ROI 전처리가 프레임 예산을 잡아먹음 |
| `stage1-v3.1-run2` | 11/6 | v3에 run 2 수정 병합, 대형 ROI 경로 선택 설정 추가 | run 3를 두 개선이 모두 들어간 상태로 하기 위해 |

> v3는 run 2 수정이 들어가기 전의 v2를 바탕으로 만들어졌고, 11/6에 `field-run2-fixes` 브랜치를 병합해 둘을 합쳤습니다.

**개발 제약** (기획서 V2.6 부록 B.11)
- 모델 학습·HEF 재컴파일은 하지 않습니다. 검출기는 Hailo Model Zoo COCO HEF로 고정합니다.
- 추가 구매가 필요한 선택은 하지 않습니다. 보유품과 절차로 대체합니다.

## 현재 상태

- **비전(Step 1):** C++ 런타임이 Pi 5에서 동작합니다(카메라 120 fps 드롭 0, Hailo 지연 p50 9.7 ms). 현재 main에는 run 2 수정과 v3 전처리가 모두 들어 있으며, 근거리 손 추적 성능은 Pi에서 run 3로 재검증해야 합니다.
- **STM32(Step 2):** 펌웨어는 아직 v1 구현(8B 단방향 패킷, PD 제어)입니다. 양방향 패킷과 보간·외란 관측 제어로 갱신할 예정입니다.
- **기구:** v0.4 설계 완료, 출력 검증 전. 자세한 내용은 [mechanical/README.md](mechanical/README.md)에 있습니다.

## 배선 (v1 펌웨어 기준)

* **RPi 5 ↔ STM32 (USART1, 115,200 bps, 8N1)**
  * RPi `GPIO 14 (Pin 8, TX)` → STM32 `PA10 (USART1_RX)` (`DMA2_Stream2` Circular)
  * RPi `GPIO 15 (Pin 10, RX)` → STM32 `PA9 (USART1_TX)`
  * RPi `GND (Pin 6)` → STM32 `GND` **(공통 접지 필수)**
* **STM32 ↔ Feetech 서보 버스 (USART6, 1,000,000 bps)**
  * STM32 `PC6 (USART6_TX)` → FE-URT-1 시리얼 버스 보드 (`DMA2_Stream6` Normal)
  * STM32 `PC7 (USART6_RX)` → FE-URT-1 시리얼 버스 보드
* 서보 구동 범위 제한 (v1): Pan `1024 ~ 3072` (±90°), Tilt `1365 ~ 2730` (±60°)
