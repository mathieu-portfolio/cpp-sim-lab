import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def read_results(path):
    agent_counts = []
    naive_ms = []
    grid_ms = []
    speedup = []
    naive_neighbor_checks = []
    grid_neighbor_checks = []
    naive_obstacle_checks = []
    grid_obstacle_checks = []

    with open(path, newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            agent_counts.append(int(row["agent_count"]))
            naive_ms.append(float(row["naive_avg_frame_ms"]))
            grid_ms.append(float(row["grid_avg_frame_ms"]))
            speedup.append(float(row["speedup"]))
            naive_neighbor_checks.append(int(row["naive_neighbor_checks"]))
            grid_neighbor_checks.append(int(row["grid_neighbor_checks"]))
            naive_obstacle_checks.append(int(row["naive_obstacle_checks"]))
            grid_obstacle_checks.append(int(row["grid_obstacle_checks"]))

    return (
        agent_counts,
        naive_ms,
        grid_ms,
        speedup,
        naive_neighbor_checks,
        grid_neighbor_checks,
        naive_obstacle_checks,
        grid_obstacle_checks,
    )


def save_plot(x, ys, labels, xlabel, ylabel, title, output_path):
    plt.figure()

    for y, label in zip(ys, labels):
        plt.plot(x, y, marker="o", label=label)

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.legend()
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
        agent_counts,
        naive_ms,
        grid_ms,
        speedup,
        naive_neighbor_checks,
        grid_neighbor_checks,
        naive_obstacle_checks,
        grid_obstacle_checks,
    ) = read_results(results_path)

    save_plot(
        agent_counts,
        [naive_ms, grid_ms],
        ["Naive", "Spatial Grid"],
        "Agent count",
        "Average frame time (ms)",
        "Agents CPU: Frame Time",
        output_dir / "frame_time.png",
    )

    save_plot(
        agent_counts,
        [speedup],
        ["Speedup"],
        "Agent count",
        "Speedup (naive / grid)",
        "Agents CPU: Spatial Grid Speedup",
        output_dir / "speedup.png",
    )

    save_plot(
        agent_counts,
        [naive_neighbor_checks, grid_neighbor_checks],
        ["Naive checks", "Grid checks"],
        "Agent count",
        "Neighbor checks",
        "Agents CPU: Neighbor Checks",
        output_dir / "neighbor_checks.png",
    )

    save_plot(
        agent_counts,
        [naive_obstacle_checks, grid_obstacle_checks],
        ["Naive checks", "Grid checks"],
        "Agent count",
        "Obstacle checks",
        "Agents CPU: Obstacle Checks",
        output_dir / "obstacle_checks.png",
    )

    print(f"Saved plots to {output_dir}")


if __name__ == "__main__":
    main()