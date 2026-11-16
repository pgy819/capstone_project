# 상단부 2축 짐벌 v0.8.5 — CATIA 수정용 · 3D 프린팅용 묶음

```text
01_CATIA/
  gimbal_v085_assembly.step   전체 조립 (BASE / PAN / TILT 하위 조립, 부품 이름·색 포함)
  parts/      P01~P11, C01~C03  출력 부품 하나당 STEP 하나 (조립 좌표 그대로)
  reference/  B01~B10           구매 부품 형상 (서보, 혼, 카메라, Pi, 나사) — 간섭 확인용
  parameters.csv               형상을 결정하는 주요 치수 (원본 파이썬 변수 이름과 값)
02_PRINT/
  P01_top_deck.stl …          출력 방향으로 이미 돌려 둔 STL (바닥 z = 0, 판 가운데)
  print_list.csv              출력 순서·수량·방향·설정·크기·예상 무게
03_DOCS/
  PRINT_GUIDE.md              출력·조립 가이드 (쿠폰 고르는 법, 케이블 넣는 법, 조립 순서)
  BOM.csv                     구매 부품 목록
  checks_v05.json             간섭·스윕·화각 검수 결과
  package_check.json          이 묶음의 모든 파일 재검사 결과 (STEP 유효성, STL 닫힘·베드 크기)
```

## 좌표와 단위

- 단위 mm. 원점은 Pan 회전축이 상판 윗면과 만나는 점입니다.
- X는 정면(시선 방향), Y는 왼쪽, Z는 위쪽입니다. Pan 0°, Tilt 0° 자세입니다.
- `parts/`의 STEP은 모두 조립 좌표에 있습니다. 어느 부품을 열어도 조립 위치에 놓여 있으므로, 여러 부품을 같이 열면 서로 맞물린 상태로 보입니다.

## CATIA에서 열고 수정하기

1. **조립 전체:** File → Open → `gimbal_v085_assembly.step`. CATProduct가 만들어지고, 그 아래 BASE, PAN, TILT(PAN 아래) 제품과 부품별 CATPart가 생깁니다. 바로 File → Save All As로 CATProduct/CATPart로 저장해 두세요.
2. **회전부 움직여 보기:** PAN은 Z축, TILT는 Y축(높이 79 mm) 둘레로 돌리면 됩니다. Assembly Design에서 Manipulation(회전, 축 지정)으로 돌리고, Analyze → Clash로 간섭을 확인할 수 있습니다.
3. **부품 수정:** STEP으로 들어온 부품은 히스토리가 없는 솔리드 하나(PartBody)입니다. Part Design에서 그 위에 피처를 더하는 방식으로 고칩니다.
   - 구멍·홈 추가: 면 위에 스케치 → Pocket / Hole.
   - 살 붙이기: 스케치 → Pad.
   - 면 밀기·당기기(두께, 길이 변경): Part Design의 Thickness 또는 Generative Shape Design에서 면을 오프셋 후 Split.
   - 다른 부품에 맞춰 수정할 때는 Product 안에서 부품을 더블클릭해 그 자리에서(in context) 편집하면 상대 부품을 보면서 그릴 수 있습니다.
4. **크게 바꿀 때:** 원본은 파라메트릭 파이썬 소스(`mechanical/gimbal_v03.py`)라서, `parameters.csv`의 치수(예: 공차, 너트 홈 크기, 판 두께)를 바꾸는 수정은 소스에서 값을 바꾸고 `python make_package.py`로 묶음 전체를 다시 만드는 편이 빠르고 정확합니다. CATIA 수정과 소스 수정을 섞으면 다음 재생성 때 CATIA 수정이 사라지니, 한쪽만 기준으로 정하세요.

## CATIA에서 고친 부품을 출력용으로 내보내기

- 수정한 CATPart를 열고 File → Save As → 형식 **stl**. Tools → Options → General → Compatibility → STL에서 Chordal tolerance를 0.01~0.02 mm 정도로 두면 곡면(혼 턱, 기둥)이 매끄럽습니다.
- CATIA에서 내보낸 STL은 조립 좌표 그대로라 슬라이서에서 출력 방향으로 돌려야 합니다. 방향은 `02_PRINT/print_list.csv`의 "출력 방향" 칸을 따르세요.

## 3D 프린팅 (Cubicon Style NEO-A22C, Cubicreator)

1. `02_PRINT/print_list.csv`의 순서대로 출력합니다. **쿠폰이 먼저**입니다: C01(M3 안내 구멍), C02(혼 지름·두께), C03 A~F(카메라 고정 방식, 고르는 것만). 쿠폰으로 값을 정한 다음 큰 부품을 뽑습니다(값을 바꿔야 하면 소스 수정 후 재생성).
2. STL을 Cubicreator에 불러오면 이미 출력 방향, 바닥 z = 0 상태입니다. **회전하지 말고** 판 가운데 배치만 합니다. 설정이 같은 부품끼리는 한 판에 같이 올려도 됩니다.
3. 하판(P02)은 기둥과 한 몸이라 하판 바닥을 판에 두고 그대로 출력합니다(리트랙션 켬).
4. 서포트는 턴테이블(P08)만 "빌드 플레이트에서만"으로 켜고 나머지는 끕니다. 나사 구멍 천장은 짧은 브리지로 출력됩니다. 브림은 서보쪽 팔(P09)에만 켭니다.
5. 출력 후 조립은 `03_DOCS/PRINT_GUIDE.md`의 순서를 따릅니다(너트는 상판-기둥 4개뿐).

## 실측 전 가정값

혼 지름·두께(혼 게이지로 결정), 서보 축 면 아래 턱 폭, Pi 스택 높이는 실측 전 가정값입니다. 혼 나사 원은 긴 구멍이라 몰라도 되고, 카메라 장착 구멍은 기본 방식(E)에서는 쓰지 않습니다. 실물과 다르면 `parameters.csv`의 해당 값을 소스에서 고쳐 다시 만드세요.
