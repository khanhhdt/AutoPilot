"""Run Phase 5 scenarios and write per-case and aggregate metrics."""

import argparse
import csv
import json
from datetime import datetime
from pathlib import Path

from scripts.scenarios import run_trials


CASES = ("clear-road", "static", "cut-in", "pedestrian", "avoid", "avoid-left")
FIELDS = ("name", "status", "distance_m", "collisions", "collisions_per_km", "lane_invasions",
          "lane_invasions_per_km", "min_ttc_s", "fps_wall")


def rates(distance_m: float, collisions: int, invasions: int) -> dict:
    return {
        "collisions_per_km": collisions * 1000 / distance_m if distance_m > 0 else None,
        "lane_invasions_per_km": invasions * 1000 / distance_m if distance_m > 0 else None,
    }


def write_report(root: Path, summary: dict, planned: tuple) -> str:
    trials = {trial["name"]: trial for trial in summary["trials"]}
    rows = []
    totals = dict(distance_m=0., collisions=0, lane_invasions=0, frames=0, elapsed_s=0., ttc_samples=0)
    for name, _ in planned:
        trial = trials.get(name, {})
        metrics = trial.get("metrics")
        row = {"name": name, "status": "PASS" if trial.get("passed") else "FAIL" if trial else "SKIP"}
        if metrics:
            row.update(distance_m=metrics["distance_m"], collisions=trial["collisions"],
                       lane_invasions=metrics["lane_invasions"], min_ttc_s=metrics["min_ttc_s"],
                       fps_wall=metrics["avg_fps_wall"])
            row.update(rates(row["distance_m"], row["collisions"], row["lane_invasions"]))
            for key in totals:
                totals[key] += row["collisions"] if key == "collisions" else metrics[key]
        rows.append(row)
    totals.update(rates(totals["distance_m"], totals["collisions"], totals["lane_invasions"]))
    totals["min_ttc_s"] = min((row["min_ttc_s"] for row in rows if row.get("min_ttc_s") is not None), default=None)
    totals["fps_wall"] = totals["frames"] / totals["elapsed_s"] if totals["elapsed_s"] else None
    totals["success_rate"] = sum(row["status"] == "PASS" for row in rows) / len(rows)

    (root / "metrics.json").write_text(json.dumps({
        "schema": "phase5-v1", "verdict": summary["verdict"], "host": summary["host"], "port": summary["port"],
        "cases": rows, "aggregate": totals,
    }, indent=2, allow_nan=False) + "\n")
    with (root / "metrics.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    def render(row):
        values = (row.get(field) for field in FIELDS)
        return "| " + " | ".join("N/A" if value is None else f"{value:.2f}" if isinstance(value, float)
                                  else str(value) for value in values) + " |"

    table = [
        "# Kết quả kiểm chứng CARLA",
        "",
        f"**{summary['verdict']}** · Tỷ lệ đạt **{totals['success_rate']:.1%}**",
        "",
        "| Kịch bản | Kết quả | Quãng đường (m) | Va chạm | Va chạm/km | Chạm vạch | Chạm vạch/km | TTC min (s) | FPS thực |",
        "|" + "---|" * len(FIELDS),
        *(render(row) for row in rows),
        render(dict(totals, name="Tổng", status=summary["verdict"])),
        "",
        "TTC dọc dùng khoảng cách và vận tốc khép từ tracker. "
        "N/A nghĩa là thiếu mẫu hợp lệ hoặc xe không tiến gần vật cản. "
        "Chạm vạch là sự kiện thô từ CARLA, gồm cả thao tác chuyển làn chủ động.",
        "",
    ]
    report = "\n".join(table)
    (root / "report.md").write_text(report, encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=3000)
    parser.add_argument("--out", type=Path, default=Path("output/phase5"))
    parser.add_argument("--case", choices=(*CASES, "curve-road", "avoid-rear"),
                        help="run a single case for a quick verification")
    parser.add_argument("--weather", choices=("clear", "rain"), default="clear")
    parser.add_argument("--lane-input", choices=("semantic", "rgb"), default="semantic")
    parser.add_argument("--save-video", action="store_true")
    parser.add_argument("--video-hd", nargs="?", const=1024, type=int, choices=(1024, 1920),
                        help="save RGB and overlay only at this native RGB width (default 1024)")
    args = parser.parse_args()
    root = args.out / datetime.now().strftime("run_%Y%m%d_%H%M%S_%f")
    planned = tuple((name, 0) for name in (CASES if args.case is None else (args.case,)))
    try:
        summary = run_trials(planned, root, args.host, args.port, args.save_video or args.video_hd is not None,
                             video_hd=args.video_hd, weather=args.weather, lane_input=args.lane_input)
    except RuntimeError as exc:
        parser.error(str(exc))
    print(write_report(root, summary, planned))
    return 0 if summary["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
