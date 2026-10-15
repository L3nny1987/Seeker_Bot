"""Explicit source-export boundary; local firmware, data and secrets stay excluded."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DIRECTORIES = {".github", ".vscode", "components", "docs", "include", "lib", "src", "test", "tools"}
ROOT_FILES = {"README.md", "LICENSE", "LICENSE.md", "CHANGELOG.md", "CONTRIBUTING.md",
              "SECURITY.md", "THIRD_PARTY_NOTICES.md", ".gitignore", ".gitattributes",
              ".editorconfig", "CMakeLists.txt", "platformio.ini", "sdkconfig.defaults",
              "dependencies.lock", "partitions_ota.csv"}
EXTENSIONS = {".c", ".h", ".cpp", ".hpp", ".py", ".ps1", ".js", ".html", ".css",
              ".md", ".json", ".yml", ".yaml", ".svg", ".png", ".txt", ".csv"}
EXCLUDED_PARTS = {"__pycache__", "node_modules", ".venv", "ipch"}


def public_files(root: Path = ROOT):
    result = []
    for path in root.rglob("*"):
        rel = path.relative_to(root)
        if not path.is_file() or path.is_symlink():
            continue
        if len(rel.parts) == 1:
            allowed = rel.name in ROOT_FILES
        else:
            allowed = (rel.parts[0] in DIRECTORIES and not EXCLUDED_PARTS.intersection(rel.parts)
                       and (path.suffix.lower() in EXTENSIONS or path.name in {"README", "CMakeLists.txt"}))
        if not allowed or path.name == "credentials.h":
            continue
        if rel.parts[0] == ".vscode" and path.name not in {"settings.json", "tasks.json", "extensions.json"}:
            continue
        result.append(path)
    return sorted(result)
