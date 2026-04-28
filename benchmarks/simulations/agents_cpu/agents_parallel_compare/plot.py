import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3] / "common"))

from Plotting import column, read_rows, require_results_path, save_line_plot


def main():
    results_path, output_dir = require_results_path()
    rows = read_rows(results_path)

    agent_counts = column(rows, "agent_count", int)

    single_ms = column(rows, "single_avg_frame_ms", float)
    parallel_ms = column(rows, "parallel_avg_frame_ms", float)
    speedup = column(rows, "speedup", float)

    single_neighbor_checks = column(rows, "single_neighbor_checks", int)
    parallel_neighbor_checks = column(rows, "parallel_neighbor_checks", int)

    single_neighbor_candidates = column(rows, "single_neighbor_candidates", int)
    parallel_neighbor_candidates = column(rows, "parallel_neighbor_candidates", int)

    single_obstacle_checks = column(rows, "single_obstacle_checks", int)
    parallel_obstacle_checks = column(rows, "parallel_obstacle_checks", int)

    single_obstacle_candidates = column(rows, "single_obstacle_candidates", int)
    parallel_obstacle_candidates = column(rows, "parallel_obstacle_candidates", int)

    save_line_plot(
        agent_counts,
        [("Single-thread", single_ms), ("Parallel", parallel_ms)],
        "Agent count",
        "Average frame time (ms)",
        "Agents CPU: Parallel Update Frame Time",
        output_dir / "frame_time.png",
    )

    save_line_plot(
        agent_counts,
        [("Speedup", speedup)],
        "Agent count",
        "Speedup (single-thread / parallel)",
        "Agents CPU: Parallel Update Speedup",
        output_dir / "speedup.png",
    )

    save_line_plot(
        agent_counts,
        [
            ("Single-thread checks", single_neighbor_checks),
            ("Parallel checks", parallel_neighbor_checks),
        ],
        "Agent count",
        "Neighbor checks",
        "Agents CPU: Neighbor Checks",
        output_dir / "neighbor_checks.png",
    )

    save_line_plot(
        agent_counts,
        [
            ("Single-thread candidates", single_neighbor_candidates),
            ("Parallel candidates", parallel_neighbor_candidates),
        ],
        "Agent count",
        "Neighbor candidates",
        "Agents CPU: Neighbor Candidates",
        output_dir / "neighbor_candidates.png",
    )

    save_line_plot(
        agent_counts,
        [
            ("Single-thread checks", single_obstacle_checks),
            ("Parallel checks", parallel_obstacle_checks),
        ],
        "Agent count",
        "Obstacle checks",
        "Agents CPU: Obstacle Checks",
        output_dir / "obstacle_checks.png",
    )

    save_line_plot(
        agent_counts,
        [
            ("Single-thread candidates", single_obstacle_candidates),
            ("Parallel candidates", parallel_obstacle_candidates),
        ],
        "Agent count",
        "Obstacle candidates",
        "Agents CPU: Obstacle Candidates",
        output_dir / "obstacle_candidates.png",
    )

    print(f"Saved plots to {output_dir}")


if __name__ == "__main__":
    main()