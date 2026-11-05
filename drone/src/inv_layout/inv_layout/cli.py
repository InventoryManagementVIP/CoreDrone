"""`ros2 run inv_layout layout_info <warehouse>`: validate a layout and print its slots."""

from __future__ import annotations

import argparse
import sys

from inv_layout.loader import LayoutError, load_layout


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("warehouse", help="warehouse name (config/warehouses/<name>.yaml) or path")
    parser.add_argument("--slots", action="store_true", help="print every slot")
    args = parser.parse_args(argv)
    try:
        layout = load_layout(args.warehouse)
    except (LayoutError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    lo, hi = layout.bounds()
    print(f"{layout.warehouse_id}: {layout.name}")
    print(f"  aisles={len(layout.aisles)} racks={len(layout.racks())} slots={len(layout.slots)}")
    print(f"  bounds min={tuple(round(v, 2) for v in lo)} max={tuple(round(v, 2) for v in hi)}")
    if args.slots:
        for s in layout.slots:
            lx, ly, lz, _ = s.location_label.as_tuple()
            print(f"  {s.code:16s} {s.location_code:16s} label=({lx:6.2f},{ly:6.2f},{lz:5.2f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
