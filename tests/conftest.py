import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
for pkg in ("ag_perception", "ag_execution", "ag_agent"):
    path = ROOT / "src" / pkg
    if path.exists():
        sys.path.insert(0, str(path))

