"""Check the source-export boundary for local secrets, private paths and broken links.

This is a focused release check, not a full security audit. It never prints secret values.
"""
import re
from pathlib import Path
import sys
from urllib.parse import unquote
from public_files import ROOT, public_files


def check(root: Path = ROOT):
    secret_file = root / "src/credentials.h"
    secrets = []
    if secret_file.exists():
        secrets = re.findall(r'^\s*#define\s+_WIFI_(?:SSID|PASS)\s+"([^"\n]+)"',
                             secret_file.read_text(encoding="utf-8"), re.M)
        secrets = [x for x in secrets if x not in {"YOUR_WIFI_SSID", "YOUR_WIFI_PASSWORD"}]
    errors = []
    paths = public_files(root)
    for path in paths:
        if path.suffix.lower() == ".png":
            continue
        data = path.read_bytes()
        rel = path.relative_to(root).as_posix()
        if any(secret.encode("utf-8") in data for secret in secrets):
            errors.append(f"{rel}: local Wi-Fi secret detected (value withheld)")
        text = data.decode("utf-8")
        if re.search(r'[A-Za-z]:[\\/]+Users[\\/]+(?!PUBLIC|USER|<)[^\\/\s]+', text, re.I):
            errors.append(f"{rel}: private absolute Windows user path")
        if re.search(r'^\s*#define\s+_WIFI_(?:SSID|PASS)\s+"', text, re.M) and rel != "src/credentials.example.h":
            errors.append(f"{rel}: hard-coded credential macro outside template")
        if path.suffix.lower() == ".md":
            for target in re.findall(r'!?\[[^\]]*\]\(([^\s)]+)(?:\s+"[^"]*")?\)', text):
                if target.startswith(("http://", "https://", "mailto:", "#")):
                    continue
                target = unquote(target.split("#", 1)[0].strip("<>"))
                if target and not (path.parent / target).is_file():
                    errors.append(f"{rel}: broken local link: {target}")
    return paths, errors


def main():
    paths, errors = check()
    for error in errors:
        print("FAIL:", error)
    if errors:
        return 1
    print(f"PASS: {len(paths)} exportable files; no checked secret/path/link violations.")
    print("License, historical secrets and real-world validation require a separate review.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
