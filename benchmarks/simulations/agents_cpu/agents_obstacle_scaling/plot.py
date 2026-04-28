import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3] / "common"))

from Plotting import column, read_rows, require_results_path, save_line_plot


def main():
    results_path, output_dir = require_results_path()
    rows = read_rows(results_path)

    obstacle_counts = column(rows, "obstacles", int)
    avg_frame_ms = column(rows, "avg_frame_ms", float)
    obstacle_checks = column(rows, "obstacle_checks", int)
    neighbor_checks = column(rows, "neighbor_checks", int)
    neighbor_candidates = column(rows, "neighbor_candidates", int)

    save_line_plot(
        obstacle_counts,
        [(avg_frame_ms,)],
        "Obstacle count",
        "Average frame time (ms)",
        "Agents CPU: Obstacle Scaling Frame Time",
        output_dir / "avg_frame_ms.png",
    )

    save_line_plot(
        obstacle_counts,
        [(obstacle_checks,)],
        "Obstacle count",
        "Obstacle checks",
        "Agents CPU: Obstacle Checks",
        output_dir / "obstacle_checks.png",
    )

    save_line_plot(
        obstacle_counts,
        [(neighbor_checks,)],
        "Obstacle count",
        "Neighbor checks",
        "Agents CPU: Neighbor Checks",
        output_dir / "neighbor_checks.png",
    )

    save_line_plot(
        obstacle_counts,
        [(neighbor_candidates,)],
        "Obstacle count",
        "Neighbor candidates",
        "Agents CPU: Neighbor Candidates",
        output_dir / "neighbor_candidates.png",
    )

    print(f"Saved plots to {output_dir}")


if __name__ == "__main__":
    main()
