import logging
import subprocess
from collections.abc import Sequence
from pathlib import Path


def run_command(command: Sequence[str | Path], cwd: Path | None = None) -> None:
    """Prints the given command and then runs it with ``subprocess.run``"""
    # Convert any paths to str
    command_str = [str(c) for c in command]

    logging.warning("Running " + " ".join(command_str))
    subprocess.run(command_str, check=True, cwd=cwd)
