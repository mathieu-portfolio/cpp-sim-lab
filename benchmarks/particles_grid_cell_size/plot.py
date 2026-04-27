import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


def read_rows(path: Path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def main():
    if len(sys.argv) != 2:
        print("Usage: python plot.py results/results.csv")
        raise SystemExit(1)

    input_path = Path(sys.argv[1])
    rows = read_rows(input_path)

    by_particle_count = defaultdict(list)

    for row in rows:
        by_particle_count[int(row["particles"])].append({
            "cell_size": float(row["cell_size"]),
            "grid_ms": float(row["grid_ms"]),
            "checks": int(row["checks"]),
            "resolved": int(row["resolved"]),
        })

    output_dir = input_path.parent

    plt.figure()

    for particle_count, points in sorted(by_particle_count.items()):
        points.sort(key=lambda p: p["cell_size"])

        cell_sizes = [p["cell_size"] for p in points]
        grid_ms = [p["grid_ms"] for p in points]

        plt.plot(
            cell_sizes,
            grid_ms,
            marker="o",
            label=f"{particle_count} particles"
        )

    plt.xlabel("Cell size")
    plt.ylabel("Milliseconds per step")
    plt.title("Spatial grid cell size vs performance")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "grid_cell_size_time.png", dpi=160)

    plt.figure()

    for particle_count, points in sorted(by_particle_count.items()):
        points.sort(key=lambda p: p["cell_size"])

        cell_sizes = [p["cell_size"] for p in points]
        checks = [p["checks"] for p in points]

        plt.plot(
            cell_sizes,
            checks,
            marker="o",
            label=f"{particle_count} particles"
        )

    plt.xlabel("Cell size")
    plt.ylabel("Collision checks per step")
    plt.title("Spatial grid cell size vs collision checks")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "grid_cell_size_checks.png", dpi=160)

    print(f"Wrote plots to {output_dir}")


if __name__ == "__main__":
    main()