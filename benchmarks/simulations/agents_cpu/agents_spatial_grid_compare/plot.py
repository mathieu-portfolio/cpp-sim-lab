import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3] / "common"))

from Plotting import column, read_rows, require_results_path, save_line_plot


def main():
    results_path, output_dir = require_results_path()
    rows = read_rows(results_path)

    agent_counts = column(rows, "agent_count", int)
    naive_ms = column(rows, "naive_avg_frame_ms", float)
    grid_ms = column(rows, "grid_avg_frame_ms", float)
    speedup = column(rows, "speedup", float)
    naive_neighbor_checks = column(rows, "naive_neighbor_checks", int)
    grid_neighbor_checks = column(rows, "grid_neighbor_checks", int)
    naive_obstacle_checks = column(rows, "naive_obstacle_checks", int)
    grid_obstacle_checks = column(rows, "grid_obstacle_checks", int)

    save_line_plot(
        agent_counts,
        [("Naive", naive_ms), ("Spatial Grid", grid_ms)],
        "Agent count",
        "Average frame time (ms)",
        "Agents CPU: Frame Time",
        output_dir / "frame_time.png",
    )

    save_line_plot(
        agent_counts,
        [("Speedup", speedup)],
        "Agent count",
        "Speedup (naive / grid)",
        "Agents CPU: Spatial Grid Speedup",
        output_dir / "speedup.png",
    )

    save_line_plot(
        agent_counts,
        [("Naive checks", naive_neighbor_checks), ("Grid checks", grid_neighbor_checks)],
        "Agent count",
        "Neighbor checks",
        "Agents CPU: Neighbor Checks",
        output_dir / "neighbor_checks.png",
    )

    save_line_plot(
        agent_counts,
        [("Naive checks", naive_obstacle_checks), ("Grid checks", grid_obstacle_checks)],
        "Agent count",
        "Obstacle checks",
        "Agents CPU: Obstacle Checks",
        output_dir / "obstacle_checks.png",
    )

    print(f"Saved plots to {output_dir}")


if __name__ == "__main__":
    main()
