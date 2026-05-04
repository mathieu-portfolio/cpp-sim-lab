#!/usr/bin/env python3
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def mode_label(row: dict[str, str]) -> str:
    backend = row.get("backend", "")
    threading = row.get("threading", "")
    return f"{backend}:{threading}" if backend else threading


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: plot.py <results.csv>")
        return 1

    csv_path = Path(sys.argv[1])
    out_path = csv_path.with_name("avg_frame_ms.png")

    series: dict[str, list[tuple[float, float]]] = {}
    x_key = None

    with csv_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if x_key is None:
                x_key = "entity_count" if "entity_count" in row else "grid_size"
            label = mode_label(row)
            x = float(row.get("entity_count", "0"))
            y = float(row["avg_frame_ms"])
            series.setdefault(label, []).append((x, y))

    if not series:
        print("No data rows found in CSV")
        return 1

    plt.figure(figsize=(10, 6))
    for label, points in sorted(series.items()):
        points.sort(key=lambda p: p[0])
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        plt.plot(xs, ys, marker="o", label=label)

    plt.xlabel("Entity Count")
    plt.ylabel("Average Frame Time (ms)")
    plt.title("Execution Matrix Benchmark")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"Saved plot: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
