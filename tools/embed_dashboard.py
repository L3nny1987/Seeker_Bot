"""Generate a deterministic, flash-resident gzip asset during CMake configure."""

import gzip
import sys
from pathlib import Path


def generate(source: Path, target: Path) -> None:
    original = source.read_bytes()
    compressed = gzip.compress(original, compresslevel=9, mtime=0)
    if gzip.decompress(compressed) != original:
        raise RuntimeError("Dashboard gzip round-trip failed")
    rows = [
        "    " + ",".join(f"0x{byte:02x}" for byte in compressed[offset:offset + 16])
        for offset in range(0, len(compressed), 16)
    ]
    header = (
        "/* Generated from src/web/index.html; do not edit. */\n"
        "#pragma once\n"
        "static const unsigned char dashboard_gzip[] = {\n"
        + ",\n".join(rows)
        + "\n};\n"
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.exists() or target.read_text(encoding="utf-8") != header:
        target.write_text(header, encoding="utf-8")
    print(f"Dashboard: {len(original)} bytes -> {len(compressed)} gzip bytes")


if __name__ == "__main__":
    generate(Path(sys.argv[1]), Path(sys.argv[2]))