import sys
from collections import defaultdict
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3] / "common"))

from Plotting import read_rows, require_results_path, save_line_plot


def group_by_particle_count(rows):
    by_particle_count = defaultdict(list)

    for row in rows:
        by_particle_count[int(row["particles"])].append({
            "cell_size": float(row["cell_size"]),
            "total_ms": float(row["total_ms"]),
            "build_ms": float(row["build_ms"]),
            "query_collision_ms": float(row["query_collision_ms"]),
            "checks": int(row["checks"]),
            "resolved": int(row["resolved"]),
        })

    return by_particle_count


def sorted_series(points, key):
    points = sorted(points, key=lambda p: p["cell_size"])
    return (
        [p["cell_size"] for p in points],
        [p[key] for p in points],
    )


def save_grouped_plot(by_particle_count, key, ylabel, title, output_path):
    first_particle_count = next(iter(sorted(by_particle_count)))
    cell_sizes, _ = sorted_series(by_particle_count[first_particle_count], key)

    series = []
    for particle_count, points in sorted(by_particle_count.items()):
        _, values = sorted_series(points, key)
        series.append((f"{particle_count} particles", values))

    save_line_plot(
        cell_sizes,
        series,
        "Cell size",
        ylabel,
        title,
        output_path,
    )


def main():
    input_path, output_dir = require_results_path()
    rows = read_rows(input_path)
    by_particle_count = group_by_particle_count(rows)

    save_grouped_plot(
        by_particle_count,
        "total_ms",
        "Milliseconds per step",
        "Spatial grid cell size vs total time",
        output_dir / "grid_cell_size_total_time.png",
    )

    save_grouped_plot(
        by_particle_count,
        "build_ms",
        "Milliseconds per step",
        "Spatial grid build time",
        output_dir / "grid_cell_size_build_time.png",
    )

    save_grouped_plot(
        by_particle_count,
        "query_collision_ms",
        "Milliseconds per step",
        "Spatial grid query + collision time",
        output_dir / "grid_cell_size_query_collision_time.png",
    )

    save_grouped_plot(
        by_particle_count,
        "checks",
        "Collision checks per step",
        "Spatial grid cell size vs collision checks",
        output_dir / "grid_cell_size_checks.png",
    )

    print(f"Wrote plots to {output_dir}")


if __name__ == "__main__":
    main()
