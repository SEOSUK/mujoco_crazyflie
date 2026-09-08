#!/usr/bin/env python3
"""Generate the precise MuJoCo heightfield joining the two cone surfaces."""

import math
import struct
from pathlib import Path

CONE_ANGLE_DEG = 15.0
SHORT_RADIUS = 0.30
TALL_RADIUS = 0.50
SHORT_CENTER_Y = 1.0
TALL_CENTER_Y = -1.0
BRIDGE_HALF_WIDTH_X = 0.25
SHORT_BOUNDARY_Y = 0.90
TALL_BOUNDARY_Y = -0.60
NROW_Y = 121
NCOL_X = 41
DECK_HEIGHT = 0.02
HEIGHT_SCALE = 0.7264101615


def cone_boundary(radius: float, radial_y: float, x: float) -> tuple[float, float]:
    """Return height and inward descent slope on a cone's fixed-y cut."""
    cot_angle = 1.0 / math.tan(math.radians(CONE_ANGLE_DEG))
    rho = math.hypot(x, radial_y)
    return cot_angle * (radius - rho), cot_angle * radial_y / rho


def transition_height(boundary_height: float, slope: float, distance: float) -> float:
    """Match cone slope at d=0 and reach deck height and zero slope at d=L."""
    height_above_deck = boundary_height - DECK_HEIGHT
    length = 2.0 * height_above_deck / slope
    if distance >= length:
        return DECK_HEIGHT
    ratio = 1.0 - distance / length
    return DECK_HEIGHT + height_above_deck * ratio * ratio


def surface_height(x: float, y: float) -> float:
    if y >= 0.0:
        h, slope = cone_boundary(SHORT_RADIUS, SHORT_CENTER_Y - SHORT_BOUNDARY_Y, x)
        return transition_height(h, slope, SHORT_BOUNDARY_Y - y)
    h, slope = cone_boundary(TALL_RADIUS, TALL_BOUNDARY_Y - TALL_CENTER_Y, x)
    return transition_height(h, slope, y - TALL_BOUNDARY_Y)


def main() -> None:
    # cf21B_500.xml sets compiler meshdir="assets", which is also the asset
    # base directory used when resolving this heightfield file.
    output = Path(__file__).with_name("assets") / "contact_track.bin"
    values = []
    for row in range(NROW_Y):
        y = TALL_BOUNDARY_Y + (SHORT_BOUNDARY_Y - TALL_BOUNDARY_Y) * row / (NROW_Y - 1)
        for col in range(NCOL_X):
            x = -BRIDGE_HALF_WIDTH_X + 2.0 * BRIDGE_HALF_WIDTH_X * col / (NCOL_X - 1)
            values.append((surface_height(x, y) - DECK_HEIGHT) / HEIGHT_SCALE)
    if min(values) < 0.0 or max(values) > 1.0 + 1.0e-9:
        raise ValueError("heightfield values must remain in [0, 1]")
    with output.open("wb") as stream:
        stream.write(struct.pack("ii", NROW_Y, NCOL_X))
        stream.write(struct.pack(f"{len(values)}f", *values))
    print(f"wrote {output}: {NROW_Y}x{NCOL_X}, range [{min(values):.6f}, {max(values):.6f}]")


if __name__ == "__main__":
    main()
