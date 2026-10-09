"""Copy every built firmware image into <project>/ota/ for OTA upload.

ota/seeker_bot_<version>.bin   versioned copy (PROJECT_VER in CMakeLists.txt)
ota/seeker_bot_latest.bin      always the last build
"""

import re
import shutil
from pathlib import Path

Import("env")

root = Path(env.subst("$PROJECT_DIR"))


def _version():
    text = (root / "CMakeLists.txt").read_text(encoding="utf-8")
    match = re.search(r'set\(PROJECT_VER\s+"([^"]+)"\)', text)
    return match.group(1) if match else "dev"


def export_ota(source, target, env):
    image = Path(str(target[0]))
    out = root / "ota"
    out.mkdir(exist_ok=True)
    versioned = out / f"seeker_bot_{_version()}.bin"
    shutil.copyfile(image, versioned)
    shutil.copyfile(image, out / "seeker_bot_latest.bin")
    print(f"OTA-Firmware: {versioned} ({image.stat().st_size} Bytes)")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", export_ota)
