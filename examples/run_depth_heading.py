"""Convenience script to launch the demo simulation for 120 seconds."""

import os
import sys
import subprocess
import pathlib


def main() -> None:
    here = pathlib.Path(__file__).resolve().parent.parent
    os.chdir(here)
    subprocess.run([sys.executable, "-m", "subsim.run_simulation", "--duration", "120"], check=False)


if __name__ == "__main__":
    main()
