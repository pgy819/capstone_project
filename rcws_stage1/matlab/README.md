# 서보 시스템 식별과 제어 루프 설계 (MATLAB)

STS3215에 정해진 입력(계단, PRBS, 처프)을 주고 실제 위치를 500 Hz로 기록한 뒤, MATLAB으로 "목표 위치 → 실제 위치" 모델을 뽑습니다. 그 모델로 Pan/Tilt 추적 루프의 게인과, 비전 지연이 추적 오차에 미치는 영향을 계산합니다. 게인을 실기에서 감으로 돌리기 전에 근거 있는 시작값을 얻고, 지연 보상 논문(A안)의 시뮬레이션 절로도 씁니다.

```text
stm32_fw (env: sysid) --921600 bps VCP--> tools/sysid_capture.py --> <axis>_<profile>.csv
                                                                        |
                         matlab/servo_sysid.m  (모델 식별)  <-----------+
                         matlab/loop_design.m  (게인, 지연 vs 오차)
```

## 0. 하드웨어 없이 먼저 확인

모델을 알고 있는 합성 데이터로 MATLAB 스크립트가 제대로 도는지 먼저 봅니다.

```bash
cd rcws_stage1/tools
python sysid_synth.py --out-dir synth
```

MATLAB에서 `rcws_stage1/matlab` 폴더로 가서 실행합니다.

```matlab
DATA_DIR = '../tools/synth'; AXIS = 'pan'; servo_sysid
loop_design
clear DATA_DIR
```

결과가 **고유진동수 약 6 Hz, 감쇠비 약 0.8, 지연 약 4~5 ms**로 나오면 정상입니다. 합성 모델의 지연은 4 ms이고, 버스 응답과 샘플 정렬 때문에 1 ms 안팎이 더해집니다.

## 1. 펌웨어 올리기

```bash
cd rcws_stage1/stm32_fw
pio run -e sysid -t upload
```

배선은 추적 펌웨어와 같습니다(USART6 서보 버스, PA4 방향 핀). 라즈베리파이는 연결하지 않아도 됩니다. 부팅하면 두 서보에 토크를 켜고 천천히 중앙(2048)으로 갑니다.

**안전**
- 짐벌에 조립된 상태라면 Tilt는 ±45°(±512 counts) 안에서만 움직이도록 잘라 둡니다. 가장 큰 계단이 ±35°입니다.
- 케이블이 감기지 않았는지, 스토퍼에 닿지 않는지 처음 한 번은 손으로 확인합니다.
- 멈추려면 캡처 중 Ctrl-C 후 `X`를 보내거나 보드 리셋을 누릅니다.

## 2. 데이터 받기

ST-LINK 가상 COM 포트 번호를 장치 관리자에서 확인합니다(예: `COM5`). `pip install pyserial`이 필요합니다.

```bash
cd rcws_stage1/tools
python sysid_capture.py --port COM5 --axis pan  --profile step     # 12.5 s
python sysid_capture.py --port COM5 --axis pan  --profile prbs     # 21 s
python sysid_capture.py --port COM5 --axis pan  --profile chirp    # 30.5 s
python sysid_capture.py --port COM5 --axis tilt --profile step
python sysid_capture.py --port COM5 --axis tilt --profile prbs
python sysid_capture.py --port COM5 --axis tilt --profile chirp
```

각 실행이 끝나면 `pan_prbs.csv` 같은 파일이 생기고 요약이 출력됩니다. **missing replies가 1 %를 넘거나 seq gaps가 0이 아니면** 다시 받으세요.

| 입력 | 내용 | 용도 |
|---|---|---|
| step | ±2.2 / 4.4 / 8.8 / 17.6 / 35° 계단, 각 0.6 s | 큰 움직임에서 속도 제한이 걸리는지 확인 |
| prbs | ±5.3°, 20 ms 비트, 511비트 × 2 | 검증용 |
| chirp | ±5.3°, 0.2 → 12 Hz 로그 스윕 | 모델 추정용 |

기본 목표 속도는 추적 펌웨어와 같은 값(`STS_DEFAULT_SPEED`)이라, 실제로 쓰는 조건의 서보를 재게 됩니다. 서보 자체 최대 성능도 보고 싶으면 `--max-speed`를 붙입니다(파일 이름에 `_max`가 붙음).

## 3. MATLAB

```matlab
cd rcws_stage1/matlab
DATA_DIR = '../tools';          % CSV가 있는 폴더
AXIS = 'pan';  servo_sysid      % -> servo_model_pan.mat
AXIS = 'tilt'; servo_sysid      % -> servo_model_tilt.mat
AXIS = 'pan';  loop_design      % 스크립트 위쪽의 Tv_meas, f_vision을 실측값으로
```

**servo_sysid.m 출력**
- 후보 모델 네 개(1차·2차·영점 포함·과감쇠 표준형, 모두 지연 포함)의 PRBS 검증 적합도와 지연. 적합도가 2 % 안쪽이면 단순한 모델을 고릅니다.
- 고른 모델의 DC 이득, 지연, −3 dB 대역폭, 극점
- 계단 크기별 상승 시간, 오버슈트, 최고 속도와 선형 모델 예측의 비교. 큰 계단에서만 상승 시간이 늘어나면 속도 제한 구간입니다.

**loop_design.m 출력**
- **Part A:** 비전 지연별로 위상 여유 60°가 되는 PI 게인과 교차 주파수, 그리고 v1 펌웨어 상수(`KP_PAN`, `KD_PAN`)로 바꾼 값. v1의 증분식 `goal += KP*e + KD*de`는 사실상 PI 제어기라 이렇게 바로 대응됩니다.
- **Part B:** 같은 서보 모델로 오차 피드백(C0)과 절대 시선각 + 등속도 예측(C2)의 RMS 추적 오차를 지연별로 비교합니다. 표적은 경사(34°/s, run 2 손 속도 중앙값), 사인, 대역 제한 잡음, 그리고 `docs/field_runs/run2/run2_los.csv`(run 2의 실제 손 움직임)입니다.

## 4. 결과 정리

`servo_model_<axis>.mat`, 출력된 표, 그림 두 장을 `docs/field_runs/` 옆에 `docs/sysid/`로 남겨 두면 됩니다. 논문 A안에서는 Part B의 지연 vs 오차 곡선이 "시뮬레이션 예측"이 되고, 이후 실측 C0~C3 결과와 같은 축에 겹쳐 그립니다.

## 문제 해결

| 증상 | 원인 후보 |
|---|---|
| missing replies 100 % | 서보 응답 수준(레지스터 8)이 0, PA4 방향 제어 또는 FE-URT-1 배선, ID 불일치 |
| 응답은 오는데 위치가 안 변함 | 토크 꺼짐(레지스터 40), 각도 제한(9, 11)이 0이라 모터 모드 |
| overrun이 많음 | 버스 프레임이 2 ms 안에 안 끝남: 반환 지연(레지스터 7)이 큼 |
| CSV가 비어 있음 | 포트 번호, 921600 bps 설정, 다른 프로그램이 포트를 점유 |
