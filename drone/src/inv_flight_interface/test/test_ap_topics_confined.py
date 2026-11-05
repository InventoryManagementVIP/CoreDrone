"""Only inv_flight_interface may name ArduPilot's /ap/* interfaces (docs/adr/0003-ardupilot-dds.md).

If this fails, route the new need through inv_flight_interface (add a topic/action there)
instead of talking to the flight controller directly.
"""

import pathlib
import re

SRC = pathlib.Path(__file__).resolve().parents[2]  # drone/src
OWNER = "inv_flight_interface"
PATTERN = re.compile(r"""["'/]ap/[a-z]""")
SUFFIXES = {".py", ".yaml", ".yml", ".xml", ".cpp", ".hpp", ".h", ".json"}


def test_no_ap_interfaces_outside_flight_interface():
    offenders = []
    for path in SRC.rglob("*"):
        if not path.is_file() or path.suffix not in SUFFIXES:
            continue
        rel = path.relative_to(SRC)
        if rel.parts[0] == OWNER or any(p in ("build", "install", "log") for p in rel.parts):
            continue
        for lineno, line in enumerate(path.read_text(errors="ignore").splitlines(), start=1):
            if PATTERN.search(line):
                offenders.append(f"{rel}:{lineno}: {line.strip()}")
    assert not offenders, "ArduPilot /ap/* used outside inv_flight_interface:\n" + "\n".join(offenders)
