import csv
import sys
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

    particle_counts = [int(row["particles"]) for row in rows]
    naive_ms = [float(row["naive_ms"]) for row in rows]
    grid_ms = [float(row["grid_ms"]) for row in rows]
    speedup = [float(row["speedup"]) for row in rows]
    naive_checks = [int(row["naive_checks"]) for row in rows]
    grid_checks = [int(row["grid_checks"]) for row in rows]

    output_dir = input_path.parent

    plt.figure()
    plt.plot(particle_counts, naive_ms, marker="o", label="Naive")
    plt.plot(particle_counts, grid_ms, marker="o", label="Spatial grid")
    plt.xlabel("Particles")
    plt.ylabel("Milliseconds per step")
    plt.title("Collision broad phase time")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "collision_time.png", dpi=160)

    plt.figure()
    plt.plot(particle_counts, naive_checks, marker="o", label="Naive")
    plt.plot(particle_counts, grid_checks, marker="o", label="Spatial grid")
    plt.xlabel("Particles")
    plt.ylabel("Collision checks per step")
    plt.title("Collision checks")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "collision_checks.png", dpi=160)

    plt.figure()
    plt.plot(particle_counts, speedup, marker="o")
    plt.xlabel("Particles")
    plt.ylabel("Speedup")
    plt.title("Spatial grid speedup over naive")
    plt.grid(True)
    plt.savefig(output_dir / "collision_speedup.png", dpi=160)

    print(f"Wrote plots to {output_dir}")


if __name__ == "__main__":
    main()
