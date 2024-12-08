import logging
import sys
from argparse import ArgumentParser
from subprocess import SubprocessError

from .errors import CommandError
from .lint_clang_format import lint_clang_format
from .lint_darglint import lint_darglint
from .lint_mypy import lint_mypy
from .lint_ruff import lint_ruff_check, lint_ruff_format
from .lint_shellcheck import lint_shellcheck

PYTHON_LANGUAGE = "python"
CPP_LANGUAGE = "cpp"
BASH_LANGUAGE = "bash"
ALL_LANGUAGES = "all"


LINTERS = {
    PYTHON_LANGUAGE: [
        # Run linters from fastest to slowest
        lint_ruff_format,
        lint_ruff_check,
        lint_darglint,
        lint_mypy,
    ],
    CPP_LANGUAGE: [
        lint_clang_format,
    ],
    BASH_LANGUAGE: [
        lint_shellcheck,
    ],
}


FIX_MODE = "fix"
CHECK_MODE = "check"
ALL_MODE = "all"


def entrypoint() -> None:
    parser = ArgumentParser(description="Runs a suite of code linting tools")

    parser.add_argument(
        "--mode",
        choices=[FIX_MODE, CHECK_MODE, ALL_MODE],
        default=ALL_MODE,
        help=f"The '{FIX_MODE}' mode allows linters to auto-fix problems. Linters "
        f"without this feature will not be run. The '{CHECK_MODE}' mode runs all "
        f"linters without changing code, and exits with an error code if any fail."
        f"The '{ALL_MODE}' mode runs '{FIX_MODE}', then '{CHECK_MODE}'. This is the"
        f"default.",
    )

    parser.add_argument(
        "--languages",
        nargs="+",
        choices=[ALL_LANGUAGES, *LINTERS.keys()],
        default=[PYTHON_LANGUAGE],
        help=f"Specifies which languages should be linted. To run linting for all "
        f"available languages, specify '{ALL_MODE}'.",
    )

    args = parser.parse_args()

    linters_to_run = []
    if ALL_LANGUAGES in args.languages:
        for linters in LINTERS.values():
            linters_to_run += linters
    else:
        for language in args.languages:
            linters_to_run += LINTERS[language]

    for linter in linters_to_run:
        if args.mode in [FIX_MODE, ALL_MODE]:
            linter(True)
        if args.mode in [CHECK_MODE, ALL_MODE]:
            linter(False)


def main() -> None:
    try:
        entrypoint()
    except SubprocessError:
        # The subprocess will log error information
        sys.exit(1)
    except CommandError as ex:
        logging.error(ex.message)
        sys.exit(1)


if __name__ == "__main__":
    main()
