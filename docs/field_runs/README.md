# 현장 실행 기록

실행마다 폴더 하나에 요약(`summary.md`), 그래프(`runN.png`), 로그 원본(`runN.rlog`)을 둡니다. `.rlog`는 1분에 약 1 MB라 저장소에 함께 넣습니다. 원본 프레임 클립(`.rclip`, 3초에 약 840 MB)은 넣지 않고, 보관 위치만 요약에 적습니다.

| 실행 | 날짜 | 코드 | 결과 한 줄 | 다음 조치 |
|---|---|---|---|---|
| [run 1](run1/summary.md) | 10/27 | 첫 C++ 배포본 (태그 없음, `stage1-v2-cpp` 직전) | 96 px 템플릿이 폰 화면 가운데만 잡아 NCC 1.4% 성공, 관측 없는 외삽으로 고각 −401°까지 발산 | run 1 수정 → `stage1-v2-cpp` |
| [run 2](run2/summary.md) | 10/28 | `stage1-v2-cpp` | 발산은 해소, 그러나 실효 60 fps, NCC 2.1% 성공, 검출 47% 게이트 탈락 | run 2 수정 → `field-run2-fixes` → `stage1-v3.1-run2` |
| run 3 | 예정 | `stage1-v3.1-run2` | 1~2 m 거리, run 2 수정 실기 검증 | — |

## 새 실행을 추가하는 방법

```bash
cd rcws_stage1/tools
python3 rcws_log.py runN.rlog          # 요약 수치
python3 track_plot.py runN.rlog        # runN.png
```

`docs/field_runs/runN/`에 `runN.rlog`, `runN.png`, `summary.md`(아래 항목)를 넣고 커밋합니다.

- 날짜, 코드 태그 또는 커밋, 사용한 설정 파일
- 거리, 조명, 표적, 손으로 움직였는지
- 핵심 수치: 실효 fps, 처리 시간 p50/p99, 상태 점유율, NCC 성공률, 검출 수용률
- 관찰과 원인, 다음 조치
