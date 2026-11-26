# 서보 식별(sysid) 실측 절차

STS3215 Pan·Tilt의 실제 응답(고유진동수·감쇠·지연)을 재서 `matlab/servo_sysid.m`으로 모델을 만든다.
결과는 U7(각도 기반 제어 법칙) 게인 설계와 논문 C0~C3 시뮬레이션의 출발점이다.

## 0. 준비

- 짐벌 조립 상태 그대로(회전부 하중이 실제와 같아야 함). 서보 12 V 전원, Nucleo ST-LINK USB를 PC에.
- **레이저는 끄거나 사람 쪽을 향하지 않게** 둔다. 측정 중 Pan ±35°, Tilt ±35°까지 움직인다.
- 손은 Nucleo **B1(파란 버튼)** 근처에. B1 = 비상정지(토크 끔, RESET까지 유지).

## 1. 정면 보정 (처음 한 번)

sysid 펌웨어는 **정면 보정이 안 된 서보를 움직이지 않는다**(전원 투입 때 위치를 읽고, 이동 범위
Pan 626~3470 / Tilt 1536~2560 밖이면 토크를 끄고 `# LOCKED`).

1. `pio run -e nucleo_f401re -t upload`
2. Pi에서 RCWS 실행 → 웹 화면 → **E-STOP** (서보 힘 빠짐)
3. 짐벌을 손으로 정면(Pan 정면, Tilt 수평)에 맞춤 → **Set current pose as home**
4. 상태에 HOME_SET 표시 확인. 이후 정면 = 2048 (서보가 오프셋을 스스로 저장)

## 2. sysid 펌웨어 올리기

```
cd rcws_stage1/stm32_fw
pio run -e sysid -t upload
pio device monitor -b 921600      # 부팅 메시지 확인 후 Ctrl+C로 닫기 (포트를 비워야 캡처 가능)
```

정상: `# servo read: Pan(ID2) 2048 근처, Tilt(ID1) 2048 근처` → 천천히 정면으로 이동 → `# RCWS sysid mode ...`
`# LOCKED`가 나오면 1번(정면 보정)부터, `NO ANSWER`면 서보 전원·버스 배선 확인.

## 3. 캡처 (축마다 3회, 약 1분)

COM 번호는 장치 관리자에서 확인. 저장 위치는 `docs/field_runs/sysid1/`.

```
cd rcws_stage1
mkdir ..\docs\field_runs\sysid1
python tools\sysid_capture.py --port COM5 --axis pan  --profile chirp --out ..\docs\field_runs\sysid1\pan_chirp.csv
python tools\sysid_capture.py --port COM5 --axis pan  --profile prbs  --out ..\docs\field_runs\sysid1\pan_prbs.csv
python tools\sysid_capture.py --port COM5 --axis pan  --profile step  --out ..\docs\field_runs\sysid1\pan_step.csv
python tools\sysid_capture.py --port COM5 --axis tilt --profile chirp --out ..\docs\field_runs\sysid1\tilt_chirp.csv
python tools\sysid_capture.py --port COM5 --axis tilt --profile prbs  --out ..\docs\field_runs\sysid1\tilt_prbs.csv
python tools\sysid_capture.py --port COM5 --axis tilt --profile step  --out ..\docs\field_runs\sysid1\tilt_step.csv
```

| 프로파일 | 시간 | 진폭 | 용도 |
|---|---|---|---|
| chirp | 30 s | ±5.3° (0.2→12 Hz) | 모델 추정 |
| prbs | 21 s | ±5.3° (20 ms 비트) | 검증 |
| step | 12 s | ±2.2 / 4.4 / 8.8 / 17.6 / 35° | 큰 신호(속도 포화) 확인 |

매 회 끝의 요약 줄을 본다:

- `fresh` 약 99 % 이상, `rx_err` 0~수 개 → 정상
- `rx_err`가 대부분 → 서보 응답을 못 받음 (아래 문제 해결)
- `** ABORTED **` → 그 회차는 버리고 다시 (부하 80 %가 0.1 s 넘게 지속 = 걸림·기구 충돌, 또는 B1)

(선택) 서보 자체 한계: 같은 명령에 `--max-speed`를 붙여 `pan_chirp_max.csv`처럼 따로 저장.

## 4. MATLAB

```matlab
cd rcws_stage1/matlab
AXIS = 'pan';  DATA_DIR = '../../docs/field_runs/sysid1'; servo_sysid
AXIS = 'tilt'; DATA_DIR = '../../docs/field_runs/sysid1'; servo_sysid
```

출력되는 고유진동수·감쇠비·지연·적합도(%)와 그림을 다음 작업(U7 게인 설계)에 쓴다.
CSV 6개를 커밋해 두면 Claude가 같은 분석을 다시 돌려 볼 수 있다.

## 문제 해결

| 증상 | 원인 후보 |
|---|---|
| `# LOCKED: outside the travel window` | 정면 보정 안 됨 → 1번 |
| `NO ANSWER` | 서보 전원, FE-URT-1 배선(PA9 TX / PA10 RX), ID(Pan 2 / Tilt 1) |
| `rx_err` 대부분 | 서보 응답 지연(레지스터 7 Return Delay) 또는 어댑터 방향 전환. 첫 회 결과를 보내 주면 확인 |
| `ABORTED` 반복 | 케이블·FPC 당김, 기구 걸림. 손으로 같은 범위를 돌려 보고 걸리는 곳 확인 |
| 캡처가 아무 출력 없이 60 s 대기 | 포트를 다른 프로그램(시리얼 모니터)이 잡고 있음 |
