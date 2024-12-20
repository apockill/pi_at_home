from pathlib import Path


def get_next_numbered_dir(contains_numbered_dirs: Path, prefix: str) -> Path:
    """Get the next directory within path assuming there exist directories with names
    like "dir_001"."""
    contains_numbered_dirs.mkdir(parents=True, exist_ok=True)
    existing_numbered_dirs = [
        int(p.name.split("_")[-1])
        for p in contains_numbered_dirs.iterdir()
        if p.is_dir()
    ]
    next_number = max(existing_numbered_dirs, default=0) + 1
    return contains_numbered_dirs / f"{prefix}_{next_number:03d}"
