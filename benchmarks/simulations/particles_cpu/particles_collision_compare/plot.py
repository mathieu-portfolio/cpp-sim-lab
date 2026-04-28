import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3] / "common"))

from Plotting import column, read_rows, require_results_path, save_line_plot


def main():
    input_path, output_dir = require_results_path()
    rows = read_rows(input_path)

    particle_counts = column(rows, "particles", int)
    naive_ms = column(rows, "naive_ms", float)
    grid_ms = column(rows, "grid_ms", float)
    speedup = column(rows, "speedup", float)
    naive_checks = column(rows, "naive_checks", int)
    grid_checks = column(rows, "grid_checks", int)

    save_line_plot(
        particle_counts,
        [("Naive", naive_ms), ("Spatial grid", grid_ms)],
        "Particles",
        "Milliseconds per step",
        "Collision broad phase time",
        output_dir / "collision_time.png",
    )

    save_line_plot(
        particle_counts,
        [("Naive", naive_checks), ("Spatial grid", grid_checks)],
        "Particles",
        "Collision checks per step",
        "Collision checks",
        output_dir / "collision_checks.png",
    )

    save_line_plot(
        particle_counts,
        [(speedup,)],
        "Particles",
        "Speedup",
        "Spatial grid speedup over naive",
        output_dir / "collision_speedup.png",
    )

    print(f"Wrote plots to {output_dir}")


if __name__ == "__main__":
    main()
