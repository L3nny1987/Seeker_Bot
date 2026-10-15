"""Run the hardware-free C and JavaScript regression suite on Windows/Linux/macOS."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CASES = {
    "navigation_test": "src/navigation.c",
    "navigation_escape_test": "src/navigation.c",
    "navigation_feedback_test": "src/navigation.c",
    "motion_settings_test": None,
    "pose_feedback_test": None,
    "map_transport_test": None,
    "ld20_scan_test": "components/ld20_lidar/ld20_scan_builder.c",
    "slam_test": "src/slam.c",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CC", "gcc"))
    args = parser.parse_args()
    compiler = shutil.which(args.compiler)
    node = shutil.which("node")
    if not compiler or not node:
        parser.error("Install a host C compiler (gcc) and Node.js; both must be on PATH.")
    env = os.environ.copy()
    env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
    env["CC"] = compiler
    out = ROOT / ".pio/host-tests"
    out.mkdir(parents=True, exist_ok=True)

    def run(command):
        subprocess.run([str(x) for x in command], cwd=ROOT, env=env, check=True)

    # This fixture must match the current HTML even on a fresh checkout.
    run([sys.executable, "tools/embed_dashboard.py", "src/web/index.html",
         ".pio/build/seeker_bot/esp-idf/src/generated/dashboard_asset.h"])
    for name, source in CASES.items():
        exe = out / (name + (".exe" if os.name == "nt" else ""))
        command = [compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                   "-I", "src", "-I", "components/ld20_lidar/include", f"test/{name}.c"]
        if source:
            command.append(source)
        run(command + ["-lm", "-o", exe])
        run([exe])
    tests = sorted((ROOT / "test").glob("*_test.js"))
    for test in tests:
        run([node, test])
    print(f"PASS: {len(CASES)} C programs and {len(tests)} JavaScript test programs.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        sys.exit(exc.returncode)
