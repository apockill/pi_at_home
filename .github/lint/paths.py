from itertools import chain
from pathlib import Path


def required_path(path_str: str) -> Path:
    path = Path(path_str)
    if not path.is_dir():
        raise NotADirectoryError(f"Path {path} is not a directory")
    return path


ROS_PATH = required_path("pkgs")
LINT_PATH = required_path(".github/lint")
FIRMWARE_PATH = Path()
LAUNCH_PATH = required_path("launch-profiles")


def python_paths() -> list[Path]:
    """Get all top-level directories containing lintable Python code"""
    return [ROS_PATH, LINT_PATH, LAUNCH_PATH]


def python_files() -> list[Path]:
    """Get all lintable Python files"""
    output = []
    for path in python_paths():
        output += list(path.rglob("*.py"))
    return output


def cpp_paths() -> list[Path]:
    """Get all top-level directories containing lintable C++ code"""

    # Directories under firmware/ that are not written by us
    blacklist = [".vs", "libClearCore", "LwIP", "Tools"]

    # Discover project locations
    paths: list[Path] = [
        dir_
        for dir_ in FIRMWARE_PATH.iterdir()
        if dir_.is_dir() and dir_.name not in blacklist
    ]

    return paths


# File extensions for C++ code
_CPP_EXTENSIONS = ["*.h", "*.cpp", "*.cc"]


def _is_external_source(source: Path) -> bool:
    """Checks if the source resides in a directory indicating that it's a vendored
    dependency and not written by us.

    :param source: The source to check
    :return: True if an external source
    """
    return any(parent.name in ["external", "submodules"] for parent in source.parents)


def cpp_files() -> list[Path]:
    """Get all lintable C++ files"""
    output = []

    for path in cpp_paths():
        sources = chain(*[path.rglob(e) for e in _CPP_EXTENSIONS])
        output += [s for s in sources if not _is_external_source(s)]

    return output


def ros_packages() -> list[Path]:
    """Get all directories that are ROS packages"""
    return [p for p in ROS_PATH.iterdir() if (p / "pyproject.toml").is_file()]


# Directories that contain bash scripts somewhere in them
_DIRS_WITH_BASH_SCRIPTS = [Path("docker"), Path("docs")]


def bash_files() -> list[Path]:
    """
    :return: All bash scripts in the project
    """
    output: list[Path] = []

    for folder in _DIRS_WITH_BASH_SCRIPTS:
        output += [f for f in folder.rglob("*") if _is_bash_script(f)]

    return output


def _is_bash_script(path: Path) -> bool:
    """Checks if the given file is a bash script by inspecting its shebang"""
    shebang = b"#!/usr/bin/env bash"

    if not path.is_file():
        return False

    with path.open("rb") as f:
        return f.read(len(shebang)) == shebang
