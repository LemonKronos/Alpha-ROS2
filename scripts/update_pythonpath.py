
from pathlib import Path
import json

workspace = Path(__file__).resolve().parents[1]
install = workspace / "install"

# Discover isolated colcon Python installation directories.
paths = sorted(
    str(path.resolve())
    for path in install.glob("*/lib/python*/site-packages")
    if path.is_dir()
)

# Support colcon --merge-install too.
paths.extend(
    str(path.resolve())
    for path in install.glob("lib/python*/site-packages")
    if path.is_dir()
)

paths.append("/opt/ros/jazzy/lib/python3.12/site-packages")
paths = list(dict.fromkeys(paths))

# 1. Update runtime Python paths (.pth).
import sysconfig

site_packages = Path(sysconfig.get_paths()["purelib"])
pth_file = site_packages / "ros2_colcon_paths.pth"
pth_file.write_text("\n".join(paths) + "\n")

# 2. Update Pylance paths (pyrightconfig.json).
config_file = workspace / "pyrightconfig.json"

config = (
    json.loads(config_file.read_text())
    if config_file.exists()
    else {}
)

config["extraPaths"] = paths

config_file.write_text(
    json.dumps(config, indent=4) + "\n"
)

print(f"Generated: {pth_file}")
print(f"Generated: {config_file}")
print(f"Registered {len(paths)} Python paths.")
