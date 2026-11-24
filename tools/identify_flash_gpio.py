#!/usr/bin/env python3
"""Walk through unpowered multimeter tracing of an ESP32-S3 camera flashlight."""

import argparse
import math
import re
import sys


# ESP32-S3 pads and peripherals on the camera pin map used by this project.
GPIO_NUMBERS = {*range(22), *range(26, 49)}
BOOT_STRAPS = {0, 3, 45, 46}
MEMORY_BUS = set(range(26, 38))
CAMERA_PINS = {*range(4, 14), *range(15, 19)}
TRACE_TOLERANCE_OHMS = 2.0


def pin_note(pin):
    if pin in BOOT_STRAPS:
        return "ESP32-S3 boot strap"
    if pin in MEMORY_BUS:
        return "flash/PSRAM bus on the N16R8"
    if pin in CAMERA_PINS:
        return "camera signal on this project's camera pin map"
    if pin in (19, 20):
        return "native USB"
    if pin in (43, 44):
        return "UART0, typically used by the CH340 programmer"
    return "board-specific function"


def read_resistance(prompt, *, allow_open=True):
    while True:
        raw = input(prompt).strip().lower()
        if allow_open and raw in ("ol", "open"):
            return math.inf
        try:
            value = float(raw)
            if math.isfinite(value) and value >= 0:
                return value
        except ValueError:
            pass
        print("Enter a steady reading in ohms" + (" or OL for open circuit." if allow_open else "."))


def trace():
    print("""ESP32-S3-CAM flashlight tracing

1. Disconnect USB, the CH340 programmer, batteries, and all other wires.
   Check the power rails against GND in DC voltage mode before measuring
   resistance. Wait for the voltage to fall to zero.
2. Locate the driver for the white flashlight using a matching schematic or
   a visible PCB trace. Find the GPIO side of its input resistor, or the
   documented GPIO control pad. The LED's power terminals and the driver's
   output are different points.
3. Use a digital multimeter in low-ohms mode. Measure between that control
   point and one accessible, labelled GPIO at a time. Reverse the probes and
   record both steady readings. If you cannot identify the control point,
   stop and obtain the board's schematic or clear photos.
""")
    ready = input("Is the board unpowered and the control point identified? [y/N] ").strip().lower()
    if ready not in ("y", "yes"):
        print("Stopped before any measurements.")
        return 1

    lead_resistance = read_resistance("Resistance with the probe tips touching (ohms): ", allow_open=False)
    matches = set()
    measured = set()
    while True:
        raw = input("Labelled GPIO to record, or Enter to finish: ").strip().upper()
        if not raw:
            break
        match = re.fullmatch(r"(?:GPIO|IO)?\s*(\d{1,2})", raw)
        if not match or int(match.group(1)) not in GPIO_NUMBERS:
            print("Enter a labelled ESP32-S3 GPIO, for example IO14.")
            continue
        pin = int(match.group(1))
        print(pin_note(pin))
        forward = read_resistance("First reading (ohms or OL): ")
        reverse = read_resistance("Probes reversed (ohms or OL): ")
        measured.add(pin)
        if max(forward, reverse) <= lead_resistance + TRACE_TOLERANCE_OHMS:
            matches.add(pin)
        else:
            matches.discard(pin)

    if len(matches) == 1:
        pin, = matches
        print(f"GPIO{pin} has near-direct continuity to the selected control point ({pin_note(pin)}).")
        print("Verify the PCB trace, driver type, and active level before configuring a firmware output.")
        return 0
    if matches:
        pins = ", ".join(f"GPIO{pin}" for pin in sorted(matches))
        print(f"Ambiguous continuity on {pins}. Recheck the control point and PCB trace.")
    else:
        print(f"No near-direct continuity match across {len(measured)} measured GPIOs.")
        print("Recheck the GPIO side of the driver input resistor or consult the board schematic.")
    return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    try:
        return trace()
    except (EOFError, KeyboardInterrupt):
        print("\nStopped.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
