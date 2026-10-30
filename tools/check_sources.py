"""Report source revisions and reject a checkout that differs from its manifest."""

from pathlib import Path
import subprocess
import sys

import yaml


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    manifest = yaml.safe_load((root / "sources.repos").read_text())
    mismatches = []
    for name, source in manifest["repositories"].items():
        directory = root / "src" / name
        head = subprocess.check_output(
            ["git", "-C", str(directory), "rev-parse", "HEAD"], text=True
        ).strip()
        expected = source["version"]
        dirty = subprocess.check_output(
            ["git", "-C", str(directory), "status", "--porcelain"], text=True
        ).strip()
        print(f"{name}: {head}{' (local changes)' if dirty else ''}")
        if head != expected or dirty:
            mismatches.append(name)
    if mismatches:
        print(
            "Sources differ from the pinned release: " + ", ".join(mismatches),
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
