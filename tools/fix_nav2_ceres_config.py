#!/usr/bin/env python3
"""Pin nav2_constrained_smoother to Debian/Ubuntu packaged Ceres dependencies."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

def package_config(package, basename):
    result = subprocess.run(["dpkg-query", "-L", package], check=True,
                            text=True, capture_output=True)
    matches = [Path(p) for p in result.stdout.splitlines()
               if Path(p).name == basename and Path(p).is_file()]
    if len(matches) != 1:
        raise RuntimeError(f"{package}: expected one {basename}, found {matches}")
    return str(matches[0].parent)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    if not (workspace / "src").is_dir():
        parser.error("Run from a ROS workspace containing src/, or pass --workspace.")
    packages = {
        "Ceres_DIR": ("libceres-dev", "CeresConfig.cmake"),
        "glog_DIR": ("libgoogle-glog-dev", "glog-config.cmake"),
        "gflags_DIR": ("libgflags-dev", "gflags-config.cmake"),
    }
    settings = {key: package_config(*value) for key, value in packages.items()}
    path = workspace / "colcon.meta"
    # JSON is valid colcon metadata. Preserve existing unrelated settings.
    data = json.loads(path.read_text()) if path.exists() else {}
    entry = data.setdefault("names", {}).setdefault("nav2_constrained_smoother", {})
    current = entry.get("cmake-args", [])
    if not isinstance(current, list) or not all(isinstance(x, str) for x in current):
        raise ValueError("Existing cmake-args must be a list of strings.")
    prefixes = tuple("-D" + key for key in settings)
    filtered = [arg for arg in current if not any(
        arg.startswith(prefix + "=") or arg.startswith(prefix + ":")
        for prefix in prefixes)]
    entry["cmake-args"] = filtered + [f"-D{key}={value}" for key, value in settings.items()]
    rendered = json.dumps(data, indent=2, ensure_ascii=False) + "\n"
    if not path.exists() or path.read_text() != rendered:
        if path.exists():
            with tempfile.NamedTemporaryFile(prefix="colcon.meta.backup.", dir=workspace,
                                             delete=False) as backup:
                backup_path = backup.name
            shutil.copy2(path, backup_path)
            print("Backup:", backup_path)
        with tempfile.NamedTemporaryFile(mode="w", prefix=".colcon.meta.", dir=workspace,
                                         delete=False) as output:
            output.write(rendered)
            temp_path = Path(output.name)
        temp_path.replace(path)
    print("Configured:", path)
    for key, value in settings.items():
        print(f"  {key}={value}")
    print("Build: colcon build --symlink-install --packages-select nav2_constrained_smoother --cmake-clean-cache")

if __name__ == "__main__":
    main()
