import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def read_results(path):
    obstacle_counts = []
    avg_frame_ms = []
    obstacle_checks = []
    neighbor_checks = []
    neighbor_candidates = []

    with open(path, newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            obstacle_counts.append(int(row["obstacles"]))
            avg_frame_ms.append(float(row["avg_frame_ms"]))
            obstacle_checks.append(int(row["obstacle_checks"]))
            neighbor_checks.append(int(row["neighbor_checks"]))
            neighbor_candidates.append(int(row["neighbor_candidates"]))

    return (
        obstacle_counts,
        avg_frame_ms,
        obstacle_checks,
        neighbor_checks,
        neighbor_candidates,
    )


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
    output_dir.mkdir(parents=True, exist_ok=True)

    (
        obstacle_counts,
        avg_frame_ms,
        obstacle_checks,
        neighbor_checks,
        neighbor_candidates,
    ) = read_results(results_path)

    save_plot(
        obstacle_counts,
        avg_frame_ms,
        "Obstacle count",
        "Average frame time (ms)",
        "Agents CPU: Obstacle Scaling Frame Time",
        output_dir / "avg_frame_ms.png",
    )

    save_plot(
        obstacle_counts,
        obstacle_checks,
        "Obstacle count",
        "Obstacle checks",
        "Agents CPU: Obstacle Checks",
        output_dir / "obstacle_checks.png",
    )

    save_plot(
        obstacle_counts,
        neighbor_checks,
        "Obstacle count",
        "Neighbor checks",
        "Agents CPU: Neighbor Checks",
        output_dir / "neighbor_checks.png",
    )

    save_plot(
        obstacle_counts,
        neighbor_candidates,
        "Obstacle count",
        "Neighbor candidates",
        "Agents CPU: Neighbor Candidates",
        output_dir / "neighbor_candidates.png",
    )

    print(f"Saved plots to {output_dir}")


if __name__ == "__main__":
    main()