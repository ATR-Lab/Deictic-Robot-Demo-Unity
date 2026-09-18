#!/usr/bin/env python3
"""Restore vendor assets from their pinned official revision (Python stdlib only)."""
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET

REVISION = "3c2dfa99e09beddf092e0d6521dbbcec7e7903ed"
BASE = f"https://raw.githubusercontent.com/BoosterRobotics/booster_assets/{REVISION}/"
DESTINATION = Path(__file__).resolve().parents[1] / "models/K1"


def fetch(remote, local):
    local.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(BASE+remote, timeout=60) as response:
        local.write_bytes(response.read())


def main():
    fetch("LICENSE", DESTINATION / "LICENSE")
    fetch("robots/K1/K1_22dof.urdf", DESTINATION / "K1_22dof.urdf")
    root = ET.parse(DESTINATION / "K1_22dof.urdf").getroot()
    meshes = sorted({mesh.attrib["filename"] for mesh in root.iter("mesh")})
    for mesh in meshes:
        fetch("robots/K1/"+mesh, DESTINATION / mesh)
    print(f"Restored URDF and {len(meshes)} meshes at {REVISION}")


if __name__ == "__main__":
    main()
