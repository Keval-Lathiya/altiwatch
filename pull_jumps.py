#!/usr/bin/env python3
"""
pull_jumps.py — pull AltiWatch jump CSV files from device over serial.

Protocol (device side):
  PING  → PONG
  LIST  → LIST_START / <filename> <bytes>\n / ... / LIST_END
  DUMP  → DUMP_START / FILE_START <filename>\n / ...csv... / FILE_END <filename>\n / DUMP_END

Usage:
  python pull_jumps.py [port] [--baud N] [--out DIR] [--list-only]

Requires:
  pip install pyserial
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    sys.exit("pyserial not found — run: pip install pyserial")


# -- tunables ------------------------------------------------------------------
DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 115_200
DEFAULT_OUT  = "jumps"
READLINE_TIMEOUT_S = 2.0   # per-line read timeout passed to Serial
COMMAND_TIMEOUT_S  = 30.0  # max time to wait for a full LIST/DUMP response
PING_TOTAL_TIMEOUT = 20.0  # how long to keep trying PING before giving up


# -- helpers -------------------------------------------------------------------

def _readline(ser: serial.Serial) -> str:
    """Read one line, strip CR/LF, decode as UTF-8 (replace bad bytes)."""
    return ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")


def wait_ready(ser: serial.Serial) -> bool:
    """
    Send PING every readline-timeout until we get PONG, or give up.
    Handles the device still printing boot / sensor loop output.
    """
    deadline = time.monotonic() + PING_TOTAL_TIMEOUT
    ser.write(b"PING\n")
    while time.monotonic() < deadline:
        line = _readline(ser)
        if line == "PONG":
            return True
        # readline timed out (empty string) or we got loop noise — retry
        if time.monotonic() < deadline:
            ser.write(b"PING\n")
    return False


def cmd_list(ser: serial.Serial) -> list[tuple[str, int]]:
    """Send LIST, return [(filename, size_bytes), ...]."""
    ser.write(b"LIST\n")
    files: list[tuple[str, int]] = []
    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        line = _readline(ser)
        if line == "LIST_START":
            continue
        if line == "LIST_END":
            return files
        if line.startswith("WARN "):
            print(f"  ⚠  {line[5:]}", file=sys.stderr)
            continue
        if line:
            parts = line.split(maxsplit=1)
            if len(parts) == 2:
                try:
                    files.append((parts[0], int(parts[1])))
                except ValueError:
                    pass  # malformed line — ignore

    print("Timed out waiting for LIST_END.", file=sys.stderr)
    return files


def cmd_dump(ser: serial.Serial, out_dir: Path) -> tuple[list[Path], list[str]]:
    """
    Send DUMP, stream each file to out_dir.
    Returns (saved_paths, skipped_names).
    Files already present in out_dir are skipped without re-downloading.
    """
    ser.write(b"DUMP\n")
    saved:   list[Path] = []
    skipped: list[str]  = []

    current_name:  str | None  = None
    current_lines: list[str]   = []
    in_dump = False
    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        line = _readline(ser)

        if line == "DUMP_START":
            in_dump = True
            continue
        if not in_dump:
            continue
        if line.startswith("WARN "):
            print(f"  ⚠  {line[5:]}", file=sys.stderr)
            continue
        if line.startswith("FILE_START "):
            current_name  = line[len("FILE_START "):]
            current_lines = []
            deadline = time.monotonic() + COMMAND_TIMEOUT_S  # reset per file
            continue
        if line.startswith("FILE_END "):
            if current_name is not None:
                dest = out_dir / current_name
                if dest.exists():
                    skipped.append(current_name)
                else:
                    # Reconstruct file with Unix line endings
                    dest.write_text(
                        "\n".join(current_lines) + ("\n" if current_lines else ""),
                        encoding="utf-8",
                    )
                    saved.append(dest)
            current_name  = None
            current_lines = []
            continue
        if line == "DUMP_END":
            return saved, skipped

        if current_name is not None:
            current_lines.append(line)

    print("Timed out waiting for DUMP_END.", file=sys.stderr)
    return saved, skipped


# -- main ---------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Pull AltiWatch jump CSVs from device over serial.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "port", nargs="?", default=DEFAULT_PORT,
        help="Serial port  (e.g. COM3, /dev/ttyACM0)",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--out", default=DEFAULT_OUT,
                        help="Output directory for downloaded CSVs")
    parser.add_argument("--list-only", action="store_true",
                        help="Show files on device without downloading")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # -- connect ---------------------------------------------------------------
    print(f"Connecting to {args.port} @ {args.baud} baud …")
    try:
        ser = serial.Serial(
            args.port,
            args.baud,
            timeout=READLINE_TIMEOUT_S,
            dsrdtr=False,   # don't toggle DTR — avoids accidental board reset
            rtscts=False,
        )
    except serial.SerialException as exc:
        sys.exit(f"Cannot open port: {exc}")

    # Short drain — let any in-flight bytes clear
    time.sleep(0.3)
    ser.reset_input_buffer()

    # -- handshake -------------------------------------------------------------
    print(f"Waiting for device (up to {PING_TOTAL_TIMEOUT:.0f} s) …")
    if not wait_ready(ser):
        ser.close()
        sys.exit(
            "Device did not respond to PING.\n"
            "  • Is AltiWatch firmware running?\n"
            "  • Is boot still in progress?  Wait ~12 s after power-on.\n"
            "  • Correct port?  Run: python -m serial.tools.list_ports"
        )
    print("Device ready.\n")

    # -- list ------------------------------------------------------------------
    files = cmd_list(ser)

    if not files:
        print("No files on device.")
        ser.close()
        return

    total_bytes = sum(size for _, size in files)
    print(f"Files on device ({len(files)},  {total_bytes:,} bytes total):")
    for name, size in files:
        already = (out_dir / name).exists()
        tag = "  [already downloaded]" if already else ""
        print(f"  {name:<42}  {size:>8,} B{tag}")

    if args.list_only:
        ser.close()
        return

    # -- dump ------------------------------------------------------------------
    new_files = [(n, s) for n, s in files if not (out_dir / n).exists()]
    if not new_files:
        print("\nAll files already downloaded — nothing to pull.")
        ser.close()
        return

    print(f"\nPulling {len(new_files)} new file(s) into {out_dir}/ …")
    saved, skipped = cmd_dump(ser, out_dir)
    ser.close()

    if skipped:
        print(f"  Skipped (already present): {', '.join(skipped)}")

    if saved:
        print(f"\n{'-'*50}")
        print(f"  {len(saved)} file(s) saved:")
        for p in saved:
            print(f"    {p}  ({p.stat().st_size:,} bytes)")
        print(f"{'-'*50}")
    else:
        print("Nothing saved.")


if __name__ == "__main__":
    main()
