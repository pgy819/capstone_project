"""Build the hand-off package of the current gimbal geometry (v0.8.2):

  gimbal_v051_package/
    README_먼저읽기.md
    01_CATIA/  assembly STEP (BASE / PAN / TILT sub-assemblies, named + coloured parts),
               one STEP per printed part (assembly coordinates), bought-part reference STEPs,
               parameters.csv (the driving dimensions of the parametric source)
    02_PRINT/  one STL per part already turned to its print orientation, print_list.csv (settings,
               order, quantity), coupons first
    03_DOCS/   BOM.csv, PRINT_GUIDE.md, checks_v05.json

Every exported file is read back and verified (STEP: valid solids, same volume; STL: watertight,
fits the bed of a Cubicon Style NEO-A22C). Usage: python make_package.py  -> gimbal_v051_package(.zip)
"""
import csv
import json
import os
import shutil
import tempfile

import cadquery as cq
import trimesh

import gimbal_print as gp
import gimbal_v03 as g

HERE = os.path.dirname(os.path.abspath(__file__))
VER = "v082"
PKG = os.path.join(HERE, f"gimbal_{VER}_package")
BED = (220.0, 220.0, 200.0)          # Cubicon Style NEO-A22C build volume [mm]

# part number, source name, Korean name, group, qty, print orientation, settings, order
PRINTED = [
    ("C01", "check_coupon", "확인 쿠폰 (M3 안내 구멍 지름)", "-", 1, "그대로", "PLA 100 %, 층 0.2", 1),
    ("C02", "horn_gauge", "혼 게이지 (지름 20~27 턱 + 두께 3.0~6.0 홈)", "-", 1, "그대로", "PLA 100 %, 층 0.2", 1),
] + [(f"C03{v}", f"cam_coupon_{v}", f"카메라 고정 샘플 {v}: {g.CAM_VARIANTS[v]}", "-", 0, "가운데 판 뒷면이 바닥", "PLA 100 %, 층 0.12~0.16", 1)
     for v in "ABCDEF"] + [
    ("P01", "top_deck", "상판 (아래 ㄱ자 혀 4개)", "BASE", 1, "윗면이 바닥(뒤집어서), 혀가 위로", "충전 25 %, 벽 3", 2),
    ("P02", "lower_deck", "하판 + 기둥 일체형", "BASE", 1, "하판 바닥이 바닥, 기둥 세워서", "충전 25 %, 벽 4, 리트랙션 켬", 2),
    ("P05", "pan_cage", "Pan 서보 케이지 (귀 4개: 뒤 2, 옆 2)", "BASE", 1, "바닥이 바닥", "충전 40 %, 벽 3", 3),
    ("P08", "turntable", "턴테이블 (높이 14 mm, 케이블 틈, 가운데 구멍, 밑면 너트 홈)", "PAN", 1, "윗면이 바닥(뒤집어서), 혼 고리가 위로", "충전 60 %, 벽 4, 서포트: 빌드 플레이트에서만(가운데 구멍 속 기둥 하나)", 3),
    ("P09", "arm_servo", "서보쪽 팔 (서보 채널·받침판)", "PAN", 1, "발이 바닥(세워서), 브림", "충전 60 %, 벽 4, 받침판 아래 25 mm 브리지", 4),
    ("P10", "arm_idle", "부싱쪽 팔 (위에서 끼우는 축 자리)", "PAN", 1, "바깥면이 바닥(눕혀서)", "충전 60 %, 벽 4", 4),
    ("P11", "tilt_cradle", f"Tilt 크래들 (H형, 카메라 고정 {g.CAM_MOUNT})", "TILT", 1, "위쪽 끝이 바닥(뒤집어서)", "충전 60 %, 벽 4", 4),
]
if g.EYE:
    PRINTED.append(("P12", "eye_plate", "눈 장식판", "TILT", 1, "뒷면이 바닥", "충전 100 %, 층 0.12 권장", 4))
BOUGHT = [
    ("B01", "pan_servo", "Feetech STS3215 (Pan, ID 1)", "BASE"),
    ("B02", "tilt_servo", "Feetech STS3215 (Tilt, ID 2)", "PAN"),
    ("B04", "pan_horn", "Pan 혼 (서보 동봉)", "PAN"),
    ("B05", "tilt_horn", "Tilt 혼 (서보 동봉)", "TILT"),
    ("B06", "camera", "Camera Module 3 Wide", "TILT"),
    ("B07", "pi5_stack", "Raspberry Pi 5 + 쿨러 + AI HAT+", "BASE"),
    ("B08", "fasteners_base", "M3 x 10/8/16 + 너트, M2.5 x 8 (고정부)", "BASE"),
    ("B09", "fasteners_pan", "M3 x 10 (Pan 회전부)", "PAN"),
    ("B10", "fasteners_tilt", "M2 x 5 (카메라 아래 멈춤, 선택)", "TILT"),
]
PARAMS = [  # (name, description) -> value read from the generator
    ("DECK_T", "상판·하판 두께"), ("BASE_GAP", "상판 밑면 ~ 하판 윗면"), ("PILLAR_R", "기둥 반지름"),
    ("H_TILT", "Tilt축 높이 (상판 윗면 기준)"), ("DISC_R", "턴테이블 반지름"), ("DISC_T", "턴테이블 두께"),
    ("HORN_D", "혼 지름 (혼 게이지로 결정)"), ("HORN_T", "혼 두께 (혼 게이지로 결정)"), ("HORN_CLR", "혼 턱 반지름 여유"),
    ("HORN_RIB", "혼 턱 돌기 높이 (net 0.2 눌림)"), ("AX_HOLE_R", "턴테이블 가운데 구멍 반지름"), ("AX_RANGE", "케이블 틈 설계 범위 ± [deg]"), ("AX_SLOT", "케이블 틈 높이"),
    ("AX_RISE_X", "케이블이 올라오는 자리 (축에서 뒤로)"), ("DISC_Z0", "턴테이블 밑면 높이"), ("JOINT_H", "상판-기둥 겹침 높이"), ("JOINT_GAP", "혀-기둥 틈"),
    ("SV_H", "서보 높이 (뒷면~축 면, 실측)"), ("SV_BOSS_H", "톱니 돌출 (실측 2~3)"),
    ("PILOT_R", "M3 안내 구멍 반지름"), ("EAR_CLAMP", "케이지 귀 체결 두께"),
    ("HUB_R", "턴테이블 가운데 기둥 반지름"), ("HORN_SCREW_R", "혼 나사 원 반지름"), ("ARM_T_SERVO", "Tilt 서보쪽 팔 판 두께"), ("LOWER_T", "하판 두께"), ("PI_STANDOFF", "Pi 받침 돌출"), ("CAM_BOSS", "카메라 받침 돌출"), ("CH_SHELF", "Tilt 서보 받침판 두께"), ("HORN_HOLE_R", "팔 판 혼 U홈 반지름"),
    
    ("TILT_PIN_D", "Tilt 출력 핀 지름"), ("PAN_SOFT_LIMIT", "Pan 소프트 리밋 [deg]"),
]


def stl_check(path):
    m = trimesh.load(path)
    size = m.bounds[1] - m.bounds[0]
    return m, bool(m.is_watertight), [round(float(v), 1) for v in size], bool(all(size <= BED)), bool(abs(m.bounds[0][2]) < 1e-3)


def main():
    gp.configure()
    parts, keepouts, routes = g.build(export=False)
    yoke, a, b, c = g.turntable_yoke()
    tt, arm_s, arm_i = gp.split_yoke(yoke, a, b, c)
    shapes = {n: v[0] for n, v in parts.items()}
    shapes.update(turntable=tt, arm_servo=arm_s, arm_idle=arm_i, check_coupon=gp.check_coupon(), horn_gauge=gp.horn_gauge(),
                  **{f"cam_coupon_{v}": gp.cam_coupon(v) for v in "ABCDEF"})
    orient = dict(gp.ORIENT)
    colors = {n: v[1] for n, v in parts.items()}
    colors.update(turntable=(0.20, 0.45, 0.85), arm_servo=(0.25, 0.50, 0.90), arm_idle=(0.25, 0.50, 0.90),
                  pillar=(0.55, 0.57, 0.60))

    if os.path.exists(PKG):
        shutil.rmtree(PKG)
    d_cat, d_prt, d_doc = (os.path.join(PKG, s) for s in ("01_CATIA", "02_PRINT", "03_DOCS"))
    for d in (os.path.join(d_cat, "parts"), os.path.join(d_cat, "reference"), d_prt, d_doc):
        os.makedirs(d)
    report = []

    # ---- CATIA: per-part STEP (assembly coordinates) + hierarchical assembly
    groups = {k: cq.Assembly(name=k) for k in ("BASE", "PAN", "TILT")}
    for pn, src, ko, grp, qty, *_ in PRINTED:
        sh = shapes[src]
        f = os.path.join(d_cat, "parts", f"{pn}_{src}.step")
        cq.exporters.export(sh, f)
        back = cq.importers.importStep(f)
        ok = back.val().isValid() and abs(back.val().Volume() - sh.val().Volume()) < 1e-3 * sh.val().Volume() + 1e-3
        report.append({"file": f"01_CATIA/parts/{pn}_{src}.step", "valid": bool(ok), "volume_cm3": round(sh.val().Volume() / 1000, 2)})
        if grp in groups and qty:
            groups[grp].add(sh, name=f"{pn}_{src}", color=cq.Color(*colors.get(src, (0.7, 0.7, 0.7))))
    for pn, src, ko, grp in BOUGHT:
        f = os.path.join(d_cat, "reference", f"{pn}_{src}.step")
        cq.exporters.export(shapes[src], f)
        groups[grp].add(shapes[src], name=f"{pn}_{src}", color=cq.Color(*colors[src]))
    groups["PAN"].add(groups["TILT"], name="TILT")
    root = cq.Assembly(name=f"gimbal_{VER}")
    root.add(groups["BASE"], name="BASE")
    root.add(groups["PAN"], name="PAN")
    fa = os.path.join(d_cat, f"gimbal_{VER}_assembly.step")
    root.export(fa)
    back = cq.importers.importStep(fa)
    report.append({"file": f"01_CATIA/gimbal_{VER}_assembly.step", "valid": all(s.isValid() for s in back.vals()),
                   "solids": len(back.solids().vals())})
    with open(os.path.join(d_cat, "parameters.csv"), "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.writer(fh)
        w.writerow(["변수 (gimbal_v03.py)", "값 [mm]", "설명"])
        for k, desc in PARAMS:
            w.writerow([k, round(getattr(g, k), 3), desc])

    # ---- PRINT: oriented STL per part + print list
    tmp = tempfile.mkdtemp()
    with open(os.path.join(d_prt, "print_list.csv"), "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.writer(fh)
        w.writerow(["순서", "부품번호", "파일", "부품", "수량", "출력 방향(파일에 이미 적용)", "설정", "크기 x×y×z [mm]", "예상 g (PLA)"])
        for pn, src, ko, grp, qty, ori, sett, order in sorted(PRINTED, key=lambda r: (r[7], r[0])):
            s = shapes[src]
            for axis, ang in orient[src]:
                s = s.rotate((0, 0, 0), axis, ang)
            p = os.path.join(tmp, src + ".stl")
            cq.exporters.export(s, p, tolerance=0.02, angularTolerance=0.05)
            m = trimesh.load(p)
            lo, hi = m.bounds
            m.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
            name = f"{pn}_{src}.stl"
            m.export(os.path.join(d_prt, name))
            _, wt, size, fits, on_bed = stl_check(os.path.join(d_prt, name))
            fill = 1.0 if "100" in sett else 0.6 if "60" in sett else 0.45 if "40" in sett else 0.35
            grams = round(shapes[src].val().Volume() / 1000 * 1.24 * fill, 1)
            w.writerow([order, pn, name, ko, qty if qty else "선택", ori, sett, " × ".join(map(str, size)), grams])
            report.append({"file": f"02_PRINT/{name}", "watertight": wt, "size_mm": size, "fits_bed": fits, "on_plate": on_bed})

    # ---- DOCS
    shutil.copy(os.path.join(HERE, "docs", "PRINT_GUIDE.md"), d_doc)
    shutil.copy(os.path.join(HERE, "checks_v05.json"), d_doc)
    with open(os.path.join(d_doc, "BOM.csv"), "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.writer(fh)
        w.writerow(["품목", "규격", "수량", "쓰임"])
        for row in [
            ("서보", "Feetech STS3215", 2, "Pan ID 1, Tilt ID 2 (혼·동봉 나사 포함)"),
            ("나사", "M3 × 10", 4, "케이지 4 (안내 구멍에 바로)"),
            ("나사 + 너트", "M3 × 12 + M3 너트", 4, "팔 → 턴테이블 관통, 너트는 밑면 육각 홈"),
            ("나사 + 너트", "M3 × 16 + M3 너트", 4, "상판 혀 ↔ 기둥 (가로)"),
            ("케이블 타이", "폭 4.8 이하, 길이 200 이상", 1, "Tilt 서보 들림 방지"),
            ("나사 (Pi)", "M2.5 × 8", 4, "Pi 위에서 5 mm 받침 안내 구멍에 바로"),
            ("나사 (카메라)", "M2 × 5", 2, "방식 E: 기판 아래 끝 멈춤 (선택). A~C 방식이면 4개"),
            ("혼 나사", "혼 구멍 규격 (M2~M3), 길이: Pan = 3 + 혼 두께(머리는 홈에 묻힘), Tilt = 2.5 + 혼 두께", 6, "턴테이블→Pan 혼 4, 크래들→Tilt 혼 2 (모두 방사형 긴 구멍)"),
            ("카메라 케이블", "Pi 5용 FPC 22핀↔15핀 200 mm 이상", 1, f"경로 약 {round(g.length(routes['camera FPC']))} mm (Tilt 여유 포함)"),
            ("서보 케이블", "5264-3P 900 mm", 1, "경로 약 360 mm"),
        ]:
            w.writerow(row)
    with open(os.path.join(d_doc, "package_check.json"), "w") as fh:
        json.dump(report, fh, indent=1, ensure_ascii=False)
    shutil.copy(os.path.join(HERE, "docs", "PACKAGE_README.md"), os.path.join(PKG, "README_먼저읽기.md"))

    bad = [r for r in report if not all(v for k, v in r.items() if isinstance(v, bool))]
    zipf = shutil.make_archive(PKG, "zip", HERE, os.path.basename(PKG))
    print(f"{len(report)} files checked, problems: {bad if bad else 'none'}")
    print(zipf, os.path.getsize(zipf) // 1024, "KB")


if __name__ == "__main__":
    main()
