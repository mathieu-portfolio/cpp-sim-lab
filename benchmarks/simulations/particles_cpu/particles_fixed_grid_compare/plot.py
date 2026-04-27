import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main():
    if len(sys.argv) != 2:
        print("Usage: python plot.py results.csv")
        return

    input_path = Path(sys.argv[1])
    rows = read_rows(input_path)

    particles = [int(r["particles"]) for r in rows]

    unordered_total = [float(r["unordered_total_ms"]) for r in rows]
    fixed_total = [float(r["fixed_total_ms"]) for r in rows]

    unordered_build = [float(r["unordered_build_ms"]) for r in rows]
    fixed_build = [float(r["fixed_build_ms"]) for r in rows]

    unordered_query = [float(r["unordered_query_collision_ms"]) for r in rows]
    fixed_query = [float(r["fixed_query_collision_ms"]) for r in rows]

    speedup = [float(r["speedup"]) for r in rows]

    output_dir = input_path.parent

    # --- Total time ---
    plt.figure()
    plt.plot(particles, unordered_total, marker="o", label="Unordered grid")
    plt.plot(particles, fixed_total, marker="o", label="Fixed grid")
    plt.xlabel("Particles")
    plt.ylabel("ms per step")
    plt.title("Total time comparison")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "total_time.png", dpi=160)

    # --- Build time ---
    plt.figure()
    plt.plot(particles, unordered_build, marker="o", label="Unordered grid")
    plt.plot(particles, fixed_build, marker="o", label="Fixed grid")
    plt.xlabel("Particles")
    plt.ylabel("ms per step")
    plt.title("Build time comparison")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "build_time.png", dpi=160)

    # --- Query time ---
    plt.figure()
    plt.plot(particles, unordered_query, marker="o", label="Unordered grid")
    plt.plot(particles, fixed_query, marker="o", label="Fixed grid")
    plt.xlabel("Particles")
    plt.ylabel("ms per step")
    plt.title("Query + collision time comparison")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "query_time.png", dpi=160)

    # --- Speedup ---
    plt.figure()
    plt.plot(particles, speedup, marker="o")
    plt.xlabel("Particles")
    plt.ylabel("Speedup (unordered / fixed)")
    plt.title("Fixed grid speedup over unordered_map")
    plt.grid(True)
    plt.savefig(output_dir / "speedup.png", dpi=160)

    print(f"Wrote plots to {output_dir}")


if __name__ == "__main__":
    main()