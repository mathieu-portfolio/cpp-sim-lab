import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3] / "common"))

from Plotting import column, read_rows, require_results_path, save_line_plot


def main():
    results_path, output_dir = require_results_path()
    rows = read_rows(results_path)

    boid_counts = column(rows, "boid_count", int)
    naive_ms = column(rows, "naive_avg_frame_ms", float)
    grid_ms = column(rows, "grid_avg_frame_ms", float)
    speedup = column(rows, "speedup", float)
    naive_checks = column(rows, "naive_neighbor_checks", int)
    grid_checks = column(rows, "grid_neighbor_checks", int)

    save_line_plot(
        boid_counts,
        [("Naive", naive_ms), ("Spatial Grid", grid_ms)],
        "Boid count",
        "Average frame time (ms)",
        "Boids CPU: Frame Time",
        output_dir / "frame_time.png",
    )

    save_line_plot(
        boid_counts,
        [("Speedup", speedup)],
        "Boid count",
        "Speedup (naive / grid)",
        "Spatial Grid Speedup",
        output_dir / "speedup.png",
    )

    save_line_plot(
        boid_counts,
        [("Naive checks", naive_checks), ("Grid checks", grid_checks)],
        "Boid count",
        "Neighbor checks",
        "Neighbor Checks Comparison",
        output_dir / "neighbor_checks.png",
    )

    print(f"Saved plots to {output_dir}")


if __name__ == "__main__":
    main()
