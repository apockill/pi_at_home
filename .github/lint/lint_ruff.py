from pathlib import Path

from .run_command import run_command


def lint_ruff_check(fix: bool) -> None:
    mode = "--fix" if fix else "--no-fix"
    cmd: list[str | Path] = ["ruff", "check", mode, Path()]
    run_command(cmd)


def lint_ruff_format(fix: bool) -> None:
    optional = [] if fix else ["--check"]
    cmd: list[str | Path] = ["ruff", "format", *optional, Path()]
    run_command(cmd)
