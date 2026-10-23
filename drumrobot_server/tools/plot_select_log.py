#!/usr/bin/env python3
"""타격점 선정 로그(log/log_MMDD_HHMM_select.csv) 분석.

사용: python3 drumrobot_server/tools/plot_select_log.py <select.csv> [--t0 S] [--t1 S]
  - 선정 표(stdout): 새 비행마다 선택 후보, 점수(허리 폭), 최대 후보 대비 손해, 유지 규칙, 타격 순간 계획 허리 vs 가능 구간
  - 그래프 창(연주마다 하나): R/L 비행·고정 타임라인 + 허리 계획/가능 구간. 툴바 돋보기로 확대, 화살표로 이동
한 파일에 연주가 여러 번이면 song_t 가 0 으로 돌아가는 곳에서 나눈다.
"""
import argparse
import sys

import matplotlib.pyplot as plt
import pandas as pd

NAMES = {0: "bass", 1: "snare", 2: "floor", 3: "mid", 4: "top", 5: "closed hh",
         6: "ride", 7: "R crash", 8: "L crash", 9: "open hh"}
MODE_COLOR = {"keep": "tab:green", "single": "tab:blue", "pair": "tab:purple",
              "fallback": "tab:red", "pair_fallback": "tab:red", "": "tab:gray"}


def floats(s):
    return [float(v) for v in s.split("|")] if isinstance(s, str) and s else []


def selections(play):
    """새 비행(state=new) 행 → 선정 표. 타격 순간 허리는 t1 == 타격 시각인 창에서 찾는다."""
    by_t1 = play.set_index(play["t1"].round(4))
    rows = []
    for _, r in play.iterrows():
        for arm in ("R", "L"):
            if r[f"{arm}_state"] != "new":
                continue
            widths = floats(r[f"{arm}_width_deg"])
            ok = floats(r[f"{arm}_order_ok"])
            cand = int(r[f"{arm}_cand"])
            allowed = [w for w, o in zip(widths, ok) if o or not r[f"{arm}_hand_order"]]
            best = max(allowed) if allowed else float("nan")
            chosen = widths[cand] if 0 <= cand < len(widths) else float("nan")
            t_hit = round(r[f"{arm}_end_time"], 4)
            w = by_t1.loc[[t_hit]].iloc[0] if t_hit in by_t1.index else None
            other = "L" if arm == "R" else "R"
            rows.append({
                "t_sel": r["song_t"], "t_hit": t_hit, "arm": arm,
                "from": NAMES.get(int(r[f"{arm}_start_inst"]), "?"), "to": NAMES.get(int(r[f"{arm}_end_inst"]), "?"),
                "cand": f'{cand}/{int(r[f"{arm}_n_cand"])}', "mode": r[f"{arm}_mode"] or "-",
                "keep_idx": int(r[f"{arm}_keep_idx"]), "width": chosen, "best": best, "loss": best - chosen,
                "other": r[f"{other}_state"] + ":" + NAMES.get(int(r[f"{other}_end_inst"]), "?"),
                "waist_hit": None if w is None else w["waist_t1_deg"],
                "min_hit": None if w is None else w["waist_min_t1_deg"],
                "max_hit": None if w is None else w["waist_max_t1_deg"],
                "widths": " ".join(f"{v:.1f}" for v in widths),
            })
    df = pd.DataFrame(rows)
    if not df.empty:
        df["margin"] = pd.concat([df["waist_hit"] - df["min_hit"], df["max_hit"] - df["waist_hit"]], axis=1).min(axis=1)
    return df


def plot(play, sel, title):
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True, num=title,
                             gridspec_kw={"height_ratios": [1, 1, 1.2]})
    for ax, arm in zip(axes[:2], ("R", "L")):
        for _, r in play.iterrows():
            st, s_i, e_i = r[f"{arm}_state"], r[f"{arm}_start_inst"], r[f"{arm}_end_inst"]
            if st == "hold":    # 고정
                ax.plot([r["song_t"], r["t1"]], [s_i, s_i], color="0.7", lw=3, solid_capstyle="butt")
            elif st == "new":   # 비행 시작 → 타격
                c = MODE_COLOR.get(r[f"{arm}_mode"], "tab:gray")
                ax.annotate("", xy=(r[f"{arm}_end_time"], e_i), xytext=(r[f"{arm}_start_time"], s_i),
                            arrowprops=dict(arrowstyle="->", color=c, lw=1.5))
                widths = floats(r[f"{arm}_width_deg"])
                cand = int(r[f"{arm}_cand"])
                label = f'c{cand}' + (f' {widths[cand]:.0f}°' if 0 <= cand < len(widths) else "")
                ax.text(r[f"{arm}_end_time"], e_i + 0.25, label, fontsize=7, color=c, ha="center")
            if r[f"{arm}_note_now"]:
                ax.plot(r["song_t"], s_i, "k|", ms=10)   # rds[0] 타격 (s_i = 오픈 하이햇 반영 악기)
        ax.set_yticks(range(10))
        ax.set_yticklabels([NAMES[i] for i in range(10)])
        ax.set_ylabel(f"{arm} arm")
        ax.grid(axis="y", alpha=0.3)
    handles = [plt.Line2D([], [], color=c, lw=2, label=m) for m, c in MODE_COLOR.items() if m]
    handles.append(plt.Line2D([], [], color="0.7", lw=3, label="hold"))
    axes[0].legend(handles=handles, loc="upper right", fontsize=7, ncol=len(handles))

    ax = axes[2]
    ax.fill_between(play["t1"], play["waist_min_t1_deg"], play["waist_max_t1_deg"], step=None, alpha=0.2, label="feasible (actual pos @ t1)")
    ax.plot(play["t1"], play["waist_t1_deg"], "k.-", lw=1, ms=3, label="planned waist @ t1")
    if not sel.empty:
        tight = sel[sel["margin"] < 5]
        ax.plot(tight["t_hit"], tight["waist_hit"], "rx", ms=8, label="hit, margin < 5°")
    ax.set_ylabel("waist [deg]")
    ax.set_xlabel("song time [s]")
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(alpha=0.3)
    fig.tight_layout()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--t0", type=float, default=None)
    ap.add_argument("--t1", type=float, default=None)
    a = ap.parse_args()

    df = pd.read_csv(a.csv, keep_default_na=False)
    if df.empty:
        sys.exit("빈 로그")
    play_id = (df["song_t"].diff() < 0).cumsum()   # song_t 가 되돌아가면 새 연주
    pd.set_option("display.width", 250)
    pd.set_option("display.max_rows", None)
    pd.set_option("display.max_colwidth", 60)
    for k, play in df.groupby(play_id):
        if a.t0 is not None: play = play[play["t1"] >= a.t0]
        if a.t1 is not None: play = play[play["song_t"] <= a.t1]
        if play.empty:
            continue
        sel = selections(play)
        print(f"\n===== play {k}: {play['song_t'].min():.1f} ~ {play['t1'].max():.1f} s, 창 {len(play)}, 선정 {len(sel)} =====")
        if not sel.empty:
            print(sel.round(1).to_string(index=False))
            print(f"\n모드별: {sel['mode'].value_counts().to_dict()}")
            print(f"유지 규칙 손해 > 0: {(sel['loss'] > 0).sum()}건, 최대 {sel['loss'].max():.1f}°")
            print(f"최소 선정 폭 {sel['width'].min():.1f}° (t={sel.loc[sel['width'].idxmin(), 't_hit']}), "
                  f"타격 순간 허리 여유 최소 {sel['margin'].min():.1f}° (t={sel.loc[sel['margin'].idxmin(), 't_hit']})")
        plot(play, sel, f"play {k}")
    plt.show()


if __name__ == "__main__":
    main()
