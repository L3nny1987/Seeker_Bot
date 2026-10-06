"""Refresh the dashboard before PlatformIO scans C header dependencies.

PlatformIO may reuse its CMake file API without executing CMake again when
only HTML changes; CMAKE_CONFIGURE_DEPENDS alone is insufficient there.
"""

import runpy
from pathlib import Path

Import("env")

root = Path(env.subst("$PROJECT_DIR"))
build = Path(env.subst("$BUILD_DIR"))
generator = runpy.run_path(str(root / "tools" / "embed_dashboard.py"))
generator["generate"](
    root / "src" / "web" / "index.html",
    build / "esp-idf" / "src" / "generated" / "dashboard_asset.h",
)