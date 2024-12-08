import os

from .paths import LAUNCH_PATH, LINT_PATH, ROS_PATH, ros_packages
from .run_command import run_command


def lint_mypy(fix: bool) -> None:
    if not fix:
        # Create MYPYPATH, which allows mypy to cross-check type hints in monorepo
        # packages
        mypy_path = ""
        for path in ROS_PATH.iterdir():
            mypy_path += f":{path}"
        os.environ["MYPYPATH"] = mypy_path

        # Run MyPy on each ROS package and other Python locations
        target_paths = ros_packages() + [LAUNCH_PATH, LINT_PATH]
        for path in target_paths:
            run_command(
                [
                    "mypy",
                    # Turns off the cache to avoid "Should never get here in
                    # normal mode, got NoneType: instead of TypeInfo" errors
                    "--cache-dir",
                    os.devnull,
                    "--show-error-codes",
                    path,
                ]
            )
