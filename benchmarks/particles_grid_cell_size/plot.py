import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


def read_rows(path: Path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def group_by_particle_count(rows):
    by_particle_count = defaultdict(list)

    for row in rows:
        by_particle_count[int(row["particles"])].append({
            "cell_size": float(row["cell_size"]),
            "total_ms": float(row["total_ms"]),
            "build_ms": float(row["build_ms"]),
            "query_collision_ms": float(row["query_collision_ms"]),
            "checks": int(row["checks"]),
            "resolved": int(row["resolved"]),
        })

    return by_particle_count


def sorted_series(points, key):
    points = sorted(points, key=lambda p: p["cell_size"])
    return (
        [p["cell_size"] for p in points],
        [p[key] for p in points],
    )


def main():
    if len(sys.argv) != 2:
        print("Usage: python plot.py results/results.csv")
        raise SystemExit(1)

    input_path = Path(sys.argv[1])
    rows = read_rows(input_path)
    by_particle_count = group_by_particle_count(rows)

    output_dir = input_path.parent

    plt.figure()
    for particle_count, points in sorted(by_particle_count.items()):
        cell_sizes, total_ms = sorted_series(points, "total_ms")
        plt.plot(cell_sizes, total_ms, marker="o", label=f"{particle_count} particles")

    plt.xlabel("Cell size")
    plt.ylabel("Milliseconds per step")
    plt.title("Spatial grid cell size vs total time")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "grid_cell_size_total_time.png", dpi=160)

    plt.figure()
    for particle_count, points in sorted(by_particle_count.items()):
        cell_sizes, build_ms = sorted_series(points, "build_ms")
        plt.plot(cell_sizes, build_ms, marker="o", label=f"{particle_count} particles")

    plt.xlabel("Cell size")
    plt.ylabel("Milliseconds per step")
    plt.title("Spatial grid build time")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "grid_cell_size_build_time.png", dpi=160)

    plt.figure()
    for particle_count, points in sorted(by_particle_count.items()):
        cell_sizes, query_ms = sorted_series(points, "query_collision_ms")
        plt.plot(cell_sizes, query_ms, marker="o", label=f"{particle_count} particles")

    plt.xlabel("Cell size")
    plt.ylabel("Milliseconds per step")
    plt.title("Spatial grid query + collision time")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "grid_cell_size_query_collision_time.png", dpi=160)

    plt.figure()
    for particle_count, points in sorted(by_particle_count.items()):
        cell_sizes, checks = sorted_series(points, "checks")
        plt.plot(cell_sizes, checks, marker="o", label=f"{particle_count} particles")

    plt.xlabel("Cell size")
    plt.ylabel("Collision checks per step")
    plt.title("Spatial grid cell size vs collision checks")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "grid_cell_size_checks.png", dpi=160)

    print(f"Wrote plots to {output_dir}")


if __name__ == "__main__":
    main()