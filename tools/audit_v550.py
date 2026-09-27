#!/usr/bin/env python3
"""Read-only package and dependency inventory for the V550 workspace."""
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
for path in sorted((root / 'src').rglob('package.xml')):
    tree = ET.parse(path).getroot()
    deps = sorted({e.text for e in tree if 'depend' in e.tag and e.text})
    print(f"{tree.findtext('name')}: {path.parent.relative_to(root)} :: {', '.join(deps)}")
