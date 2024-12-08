from .paths import bash_files
from .run_command import run_command


def lint_shellcheck(fix: bool) -> None:
    if fix:
        return  # Shellcheck does not have a fix mode

    for path in bash_files():
        run_command(["shellcheck", "--external-sources", str(path)])
