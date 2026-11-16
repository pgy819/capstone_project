# run 3 — 배포 기록 (2026-11-16, 측정 전 중단)

측정 데이터 없음. 배포와 첫 실행 시도에서 확인한 사항만 기록한다.

## 배포
- 코드: 태그 `stage1-v3.1-run2` (e71ec8c) + `preview-stack` 커밋(패치로 적용)
- 도구: `rcws_stage1/tools/pi/deploy_run3.sh` — GitHub에서 태그를 받아 `~/run3`에 빌드, 시험, 실행 스크립트 `~/run3/go.sh` 생성
- Pi 환경: hailort 4.23.0, hailo-all 5.1.1, libcamera-dev 0.7.2, HailoRT CMake 설정 `/usr/lib/cmake/HailoRT`
- 빌드: libhailort.so.4.23.0, libcamera.so.0.7 연결 확인. 단위 시험 21/21 통과

## 배포 스크립트 결함과 수정
1. **HailoRT 미연결 오판.** `set -o pipefail` 아래 `ldd … | grep -q libhailort`가, grep이 첫 일치에서 먼저 끝나 ldd가 SIGPIPE로 종료되면 실패로 판정됨(타이밍 의존). 출력을 끝까지 읽는 `grep … >/dev/null`로 수정. 재현 시험으로 확인.
2. **시험 단계가 멈춘 것처럼 보임.** 시험을 두 번 돌리고 마지막 한 줄만 출력해 Pi에서 몇 분간 출력이 없었음. 한 번만 돌리며 테스트 이름을 진행 표시로 출력, 전체는 `~/run3/tests.txt`.
3. **종료 원인이 보이지 않음.** `go.sh`가 `rcws_vision`의 종료 코드를 `|| true`로 숨김. 종료 코드 표시, 로그가 없으면 요약 생략·안내로 수정.

## 첫 실행 시도 (22:51)
- `rcws_vision`이 메시지 없이 즉시 종료, 로그 미생성. 종료 코드 미기록(결함 3). 원인 미확인 — 다음 시도에서 수정된 `go.sh`의 exit 값으로 판정.
- `vcgencmd get_throttled = 0x50000`: 부팅 후 **저전압 발생 이력(bit 16) + 스로틀 이력(bit 18)**. 재부팅 직후 몇 분 안에 기록됨.
  - P2 기준(30분 연속 운용 중 스로틀 0회)에 직접 걸리므로, 저전압이 해소되기 전 측정한 지연값은 판정·논문 데이터로 쓰지 않는다.
  - 조치(구매 없음): 보유 어댑터 중 5 V 5 A급 사용, 짧고 굵은 케이블, Pi 전원을 다른 부하와 공유하지 않기. 실행 전후 `go.sh`가 throttled 값을 기록하고 0x0이 아니면 경고한다.

## 다음 시도 전 확인
1. 부팅 후 `vcgencmd get_throttled` = `0x0`
2. `~/run3/go.sh` 실행 → `rcws_vision exit=` 값과 미리보기(원본 | 검출기 입력 | NCC) 확인
