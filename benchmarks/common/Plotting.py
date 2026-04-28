import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def require_results_path(usage="Usage: python plot.py results/results.csv"):
    if len(sys.argv) != 2:
        print(usage)
        raise SystemExit(1)

    results_path = Path(sys.argv[1])
    output_dir = results_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    return results_path, output_dir


def read_rows(path):
    with Path(path).open(newline="") as file:
        return list(csv.DictReader(file))


def column(rows, name, convert=float):
    return [convert(row[name]) for row in rows]


def save_line_plot(
    x,
    series,
    xlabel,
    ylabel,
    title,
    output_path,
    *,
    dpi=160,
):
    plt.figure()

    for item in series:
        if len(item) == 2:
            label, y = item
            plt.plot(x, y, marker="o", label=label)
        else:
            y = item[0]
            plt.plot(x, y, marker="o")

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)

    if any(len(item) == 2 for item in series):
        plt.legend()

    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi)
    plt.close()
