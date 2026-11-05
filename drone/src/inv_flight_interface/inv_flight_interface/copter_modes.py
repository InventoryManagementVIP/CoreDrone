"""ArduCopter flight-mode numbers (ArduCopter/mode.h, Mode::Number, at Copter-4.7.2)."""

COPTER_MODES: dict[str, int] = {
    "STABILIZE": 0,
    "ACRO": 1,
    "ALT_HOLD": 2,
    "AUTO": 3,
    "GUIDED": 4,
    "LOITER": 5,
    "RTL": 6,
    "CIRCLE": 7,
    "LAND": 9,
    "DRIFT": 11,
    "SPORT": 13,
    "FLIP": 14,
    "AUTOTUNE": 15,
    "POSHOLD": 16,
    "BRAKE": 17,
    "THROW": 18,
    "AVOID_ADSB": 19,
    "GUIDED_NOGPS": 20,
    "SMART_RTL": 21,
    "FLOWHOLD": 22,
    "FOLLOW": 23,
    "ZIGZAG": 24,
    "SYSTEMID": 25,
    "AUTOROTATE": 26,
    "AUTO_RTL": 27,
    "TURTLE": 28,
}

_NAMES = {number: name for name, number in COPTER_MODES.items()}


def mode_id(name: str) -> int:
    try:
        return COPTER_MODES[name.upper()]
    except KeyError as exc:
        raise ValueError(f"unknown Copter mode {name!r}") from exc


def mode_name(number: int | None) -> str:
    if number is None:
        return ""
    return _NAMES.get(number, f"MODE_{number}")
