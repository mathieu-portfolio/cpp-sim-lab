import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3] / "common"))

from Plotting import column, read_rows, require_results_path, save_line_plot


def main():
    input_path, output_dir = require_results_path("Usage: python plot.py results.csv")
    rows = read_rows(input_path)

    particles = column(rows, "particles", int)
    unordered_total = column(rows, "unordered_total_ms", float)
    fixed_total = column(rows, "fixed_total_ms", float)
    unordered_build = column(rows, "unordered_build_ms", float)
    fixed_build = column(rows, "fixed_build_ms", float)
    unordered_query = column(rows, "unordered_query_collision_ms", float)
    fixed_query = column(rows, "fixed_query_collision_ms", float)
    speedup = column(rows, "speedup", float)

    save_line_plot(
        particles,
        [("Unordered grid", unordered_total), ("Fixed grid", fixed_total)],
        "Particles",
        "ms per step",
        "Total time comparison",
        output_dir / "total_time.png",
    )

    save_line_plot(
        particles,
        [("Unordered grid", unordered_build), ("Fixed grid", fixed_build)],
        "Particles",
        "ms per step",
        "Build time comparison",
        output_dir / "build_time.png",
    )

    save_line_plot(
        particles,
        [("Unordered grid", unordered_query), ("Fixed grid", fixed_query)],
        "Particles",
        "ms per step",
        "Query + collision time comparison",
        output_dir / "query_time.png",
    )

    save_line_plot(
        particles,
        [(speedup,)],
        "Particles",
        "Speedup (unordered / fixed)",
        "Fixed grid speedup over unordered_map",
        output_dir / "speedup.png",
    )

    print(f"Wrote plots to {output_dir}")


if __name__ == "__main__":
    main()
