"""Export a checked public source ZIP without Git history, firmware or local credentials."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile
from check_publication import check
from public_files import ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Destination .zip (use exports/ locally)")
    args = parser.parse_args()
    paths, errors = check()
    if errors:
        for error in errors:
            print("FAIL:", error)
        raise SystemExit("Export stopped: fix publication checks first.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    manifest = {}
    with zipfile.ZipFile(args.output, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for path in paths:
            rel = path.relative_to(ROOT).as_posix()
            data = path.read_bytes()
            archive.writestr("Seeker_Bot/" + rel, data)
            manifest[rel] = hashlib.sha256(data).hexdigest()
        archive.writestr("Seeker_Bot/SOURCE_MANIFEST.json", json.dumps(manifest, indent=2) + "\n")
    print(f"Exported {len(paths)} files to {args.output.name} (no firmware or Git history).")


if __name__ == "__main__":
    main()
