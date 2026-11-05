"""rclpy raises ValueError ("Logger severity cannot be changed between calls") when one source line
logs at two severities, which kills the node the first time the severity changes. It took down
the localization adapter in SITL; the same pattern was in the flight interface and safety monitor.
"""

import pathlib
import re

SRC = pathlib.Path(__file__).resolve().parents[2]
PATTERN = re.compile(r"=\s*[\w.()]*get_logger\(\)\.\w+\s+if\b|\bif\b.*\belse\s+[\w.()]*get_logger\(\)\.\w+\s*$", re.M)


def test_no_call_site_picks_its_severity_at_runtime():
    offenders = [
        f"{path.relative_to(SRC)}: {match.group(0)}"
        for path in SRC.rglob("*.py")
        if path != pathlib.Path(__file__).resolve()
        for match in PATTERN.finditer(path.read_text(encoding="utf-8"))
    ]
    assert not offenders, offenders
