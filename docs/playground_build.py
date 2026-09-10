"""Bundle this checkout for the static, browser-only playground."""

import hashlib
import io
import json
import re
import zipfile
from pathlib import Path

PYODIDE_VERSION = "314.0.6"


def build_bundle(project_root: Path, destination: Path) -> dict:
    """Snapshot package files (not a JS port or a floating PyPI release)."""
    version = re.search(
        r'^version\s*=\s*"([^"]+)"',
        (project_root / "pyproject.toml").read_text(),
        re.MULTILINE,
    ).group(1)
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in sorted((project_root / "irsim").rglob("*")):
            if path.is_file() and path.suffix in {".py", ".yaml", ".png"}:
                # Fixed timestamps make the archive hash independent of build time.
                info = zipfile.ZipInfo(path.relative_to(project_root).as_posix())
                info.compress_type = zipfile.ZIP_DEFLATED
                archive.writestr(info, path.read_bytes())
        archive.writestr(
            zipfile.ZipInfo(f"ir_sim-{version}.dist-info/METADATA"),
            f"Metadata-Version: 2.1\nName: ir-sim\nVersion: {version}\n",
        )
        archive.writestr(
            zipfile.ZipInfo("LICENSE"), (project_root / "LICENSE").read_bytes()
        )
    content = buffer.getvalue()
    manifest = {
        "version": version,
        "sha256": hashlib.sha256(content).hexdigest(),
        "archive": "irsim-source.zip",
        "pyodide": PYODIDE_VERSION,
        "packages": [
            "numpy",
            "scipy",
            "matplotlib",
            "shapely",
            "pyyaml",
            "imageio",
            "micropip",
        ],
        "requirements": ["loguru==0.7.3"],
    }
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "irsim-source.zip").write_bytes(content)
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def on_build_finished(app, exception):
    if exception is None and app.builder.format == "html":
        build_bundle(
            Path(__file__).resolve().parents[1],
            Path(app.outdir) / "_static" / "playground",
        )
