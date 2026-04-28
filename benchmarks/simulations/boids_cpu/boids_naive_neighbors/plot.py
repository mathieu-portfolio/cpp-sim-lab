import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3] / "common"))

from Plotting import column, read_rows, require_results_path, save_line_plot


def main():
    results_path, output_dir = require_results_path()
    rows = read_rows(results_path)

    boid_counts = column(rows, "boid_count", int)
    avg_frame_ms = column(rows, "avg_frame_ms", float)
    neighbor_checks = column(rows, "neighbor_checks", int)

    save_line_plot(
        boid_counts,
        [(avg_frame_ms,)],
        "Boid count",
        "Average frame time (ms)",
        "Naive boids average frame time",
        output_dir / "avg_frame_ms.png",
    )

    save_line_plot(
        boid_counts,
        [(neighbor_checks,)],
        "Boid count",
        "Neighbor checks",
        "Naive boids neighbor checks",
        output_dir / "neighbor_checks.png",
    )

    print(f"Saved plots to {output_dir}")


if __name__ == "__main__":
    main()
