from shutil import which

from .errors import CommandError
from .paths import cpp_files
from .run_command import run_command


def lint_clang_format(fix: bool) -> None:
    if which("clang-format") is None:
        raise CommandError("clang-format is not installed")

    additional_args = ["-i"] if fix else ["--dry-run", "-Werror"]

    for source in cpp_files():
        run_command(["clang-format", "-style=file", *additional_args, str(source)])
