#!/usr/bin/env python3
"""Fetch only the pinned official K1 URDF, its referenced meshes, and license."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET

REVISION = "3c2dfa99e09beddf092e0d6521dbbcec7e7903ed"
BASE = f"https://raw.githubusercontent.com/BoosterRobotics/booster_assets/{REVISION}/"
URDF_SHA = "834e17faf4681e5fcae116130a9da6d81b696c7f8278b95f281cec0551ca6124"


def main():
    folder = Path(__file__).resolve().parents[1] / "assets/booster_k1"
    folder.mkdir(parents=True, exist_ok=True)
    manifest = {"repository": "https://github.com/BoosterRobotics/booster_assets", "revision": REVISION, "license": "BSD-3-Clause", "files": {}}
    def fetch(relative, source):
        data = urllib.request.urlopen(BASE + source, timeout=60).read()
        sha = hashlib.sha256(data).hexdigest()
        if relative == "K1_22dof.urdf" and sha != URDF_SHA:
            raise RuntimeError("official URDF hash mismatch")
        destination = folder / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists() and destination.read_bytes() != data:
            raise RuntimeError(f"refusing to replace different asset: {destination}")
        destination.write_bytes(data)
        manifest["files"][relative] = {"sha256": sha, "source": BASE + source}
        return data
    urdf = fetch("K1_22dof.urdf", "robots/K1/K1_22dof.urdf")
    for mesh in sorted({m.get("filename") for m in ET.fromstring(urdf).iter("mesh")}):
        if not mesh.startswith("meshes/") or ".." in mesh:
            raise ValueError("unexpected mesh path")
        fetch(mesh, "robots/K1/" + mesh)
    fetch("LICENSE", "LICENSE")
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"directory": str(folder), "files": len(manifest["files"]), "revision": REVISION}))


if __name__ == "__main__":
    main()
