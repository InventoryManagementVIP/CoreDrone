#!/usr/bin/env python3
"""pre-commit hook: validate ROS package.xml files (xmllint-style, but works offline and on any OS).

* schema: REP 149 package format 3 (tools/lint/package_format3.xsd, vendored)
* the <name> matches the package directory (colcon and rosdep get confused otherwise)
* an <export><build_type> is declared
"""

from __future__ import annotations

import sys
from pathlib import Path

from lxml import etree

SCHEMA = Path(__file__).resolve().with_name("package_format3.xsd")


def check(path: Path, schema: etree.XMLSchema) -> list[str]:
    try:
        doc = etree.parse(str(path))
    except etree.XMLSyntaxError as exc:
        return [f"{path}: not well-formed XML: {exc}"]
    errors = [f"{path}:{e.line}: {e.message}" for e in schema.error_log] if not schema.validate(doc) else []
    root = doc.getroot()
    name = root.findtext("name")
    if name and name != path.parent.name:
        errors.append(f"{path}: <name>{name}</name> does not match its directory '{path.parent.name}'")
    if root.find("export/build_type") is None:
        errors.append(f"{path}: missing <export><build_type>")
    return errors


def main(argv: list[str]) -> int:
    schema = etree.XMLSchema(etree.parse(str(SCHEMA)))
    errors = [e for p in argv for e in check(Path(p), schema)]
    for e in errors:
        print(e)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
