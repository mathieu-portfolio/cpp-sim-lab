import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def read_results(path):
    boid_counts = []
    avg_frame_ms = []
    neighbor_checks = []

    with open(path, newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            boid_counts.append(int(row["boid_count"]))
            avg_frame_ms.append(float(row["avg_frame_ms"]))
            neighbor_checks.append(int(row["neighbor_checks"]))

    return boid_counts, avg_frame_ms, neighbor_checks


def save_plot(x, y, xlabel, ylabel, title, output_path):
    plt.figure()
    plt.plot(x, y, marker="o")
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()


def main():
    if len(sys.argv) != 2:
        print("Usage: python plot.py <results.csv>")
        sys.exit(1)

    results_path = Path(sys.argv[1])
    output_dir = results_path.parent

    boid_counts, avg_frame_ms, neighbor_checks = read_results(results_path)

    save_plot(
        boid_counts,
        avg_frame_ms,
        "Boid count",
        "Average frame time (ms)",
        "Naive boids average frame time",
        output_dir / "avg_frame_ms.png",
    )

    save_plot(
        boid_counts,
        neighbor_checks,
        "Boid count",
        "Neighbor checks",
        "Naive boids neighbor checks",
        output_dir / "neighbor_checks.png",
    )

    print(f"Saved plots to {output_dir}")


if __name__ == "__main__":
    main()