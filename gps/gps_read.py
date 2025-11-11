#!/usr/bin/env python3
"""
gps_read.py — BU-353S4 test script
Opens the USB GPS (usually /dev/ttyUSB0 at 4800 baud), parses NMEA, and prints coordinates.
"""

import argparse
import glob
import sys
import time
import serial
import pynmea2

def find_port(cli_port):
    if cli_port:
        return [cli_port]
    return sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))

def try_open(port, bauds):
    for b in bauds:
        try:
            ser = serial.Serial(port, b, timeout=1)
            return ser, b
        except Exception:
            continue
    return None, None

def dm_to_dd(val, hemi):
    """
    Convert NMEA DDMM.MMMM (lat) or DDDMM.MMMM (lon) to decimal degrees.
    val is a string like '4042.6149'; hemi is 'N','S','E','W'.
    """
    if not val:
        return None
    try:
        v = float(val)               # <-- cast the string to float
    except ValueError:
        return None
    deg = int(v // 100)              # safe after casting
    minutes = v - deg * 100
    dd = deg + minutes / 60.0
    if hemi in ("S", "W"):
        dd = -dd
    return dd

def main():
    ap = argparse.ArgumentParser(description="Read GPS coords from BU-353S4")
    ap.add_argument("--port", help="Serial device (e.g., /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, help="Baud rate (default 4800)")
    ap.add_argument("--show-nmea", action="store_true", help="Echo raw NMEA lines")
    args = ap.parse_args()

    ports = find_port(args.port)
    if not ports:
        print("No serial ports found. Plug in the GPS and re-run.")
        sys.exit(1)

    baud_candidates = [args.baud] if args.baud else [4800, 9600]

    ser = None
    used_baud = None
    for p in ports:
        ser, used_baud = try_open(p, baud_candidates)
        if ser:
            print(f"Connected on {p} @ {used_baud} baud")
            break
    if not ser:
        print(f"Could not open any of: {ports} at bauds {baud_candidates}")
        sys.exit(1)

    last_fix = None
    last_heartbeat = 0.0

    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                # heartbeat every ~5s while waiting for first fix
                now = time.time()
                if last_fix is None and now - last_heartbeat > 5:
                    print("Receiving satellites data; waiting for first fix...")
                    last_heartbeat = now
                continue

            if args.show_nmea:
                print(f"NMEA: {line}")

            if not line.startswith("$"):
                continue

            try:
                msg = pynmea2.parse(line)
            except pynmea2.nmea.ChecksumError:
                continue
            except Exception:
                continue

            # Prefer RMC; fall back to GGA
            if msg.sentence_type == "RMC":
                if getattr(msg, "status", "V") == "A" and msg.lat and msg.lon:
                    lat = dm_to_dd(msg.lat, msg.lat_dir)
                    lon = dm_to_dd(msg.lon, msg.lon_dir)
                    if lat is not None and lon is not None:
                        last_fix = (lat, lon)
                        print(f"Fix: lat={lat:.6f}, lon={lon:.6f}  time={getattr(msg,'datestamp','')} {getattr(msg,'timestamp','')} UTC")

            elif msg.sentence_type == "GGA":
                qual = int(getattr(msg, "gps_qual", "0") or "0")
                if qual > 0 and msg.lat and msg.lon:
                    lat = dm_to_dd(msg.lat, msg.lat_dir)
                    lon = dm_to_dd(msg.lon, msg.lon_dir)
                    alt = getattr(msg, "altitude", "")
                    sats = getattr(msg, "num_sats", "")
                    if lat is not None and lon is not None:
                        last_fix = (lat, lon)
                        print(f"Fix: lat={lat:.6f}, lon={lon:.6f}, alt={alt} m, sats={sats}  time={getattr(msg,'timestamp','')} UTC")

            # soft heartbeat when seeing satellite messages but no fix yet
            if last_fix is None and msg.sentence_type in ("GSV", "GSA"):
                now = time.time()
                if now - last_heartbeat > 5:
                    print("Receiving satellites data; waiting for first fix...")
                    last_heartbeat = now

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
