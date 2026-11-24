"""Command-line entry point for the Fly_in simulator."""

import argparse
from pathlib import Path

from fly_in import parse_map, simulate


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a Fly_in drone simulation")
    parser.add_argument("map", type=Path, help="path to a Fly_in map")
    arguments = parser.parse_args()

    try:
        map_data = parse_map(arguments.map.read_text(encoding="utf-8"))
        for turn in simulate(map_data):
            print(turn)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
