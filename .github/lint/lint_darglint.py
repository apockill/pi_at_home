from pathlib import Path

from .run_command import run_command


def lint_darglint(fix: bool) -> None:
    # Configure darglint in here because it doesn't support pyproject.toml, and we're
    # hoping to get rid of it in favor of ruff once it supports it
    cmd: list[str | Path] = [
        "darglint",
        "--strictness",
        "short",
        "--docstring-style",
        "sphinx",
        "--ignore-regex",
        "^test.*",
        Path(),
    ]
    if not fix:
        run_command(cmd)
