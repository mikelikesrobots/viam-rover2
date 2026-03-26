#!/usr/bin/env python3
"""Live telemetry monitor for RP2040 stream.

Usage: telemetry_monitor.py --port /dev/ttyS4 --baud 460800

Displays seven lines that update in-place:
  Encoder: left right
  IMU: ax ay az
  Battery: 30%
    I2C detect: pins and sensor addresses
    I2C scan: discovered device addresses
    Lock-in: ready when both sensors are detected
  Malformed packets: n

Parses frames prefixed with STX (0x02) or legacy LF-terminated lines.
Each frame format: (STX)payload CRC CRLF
Where payload looks like: T <E|I|B> <seq> ...
CRC is unsigned 8-bit XOR of the payload bytes.
"""

import argparse
import serial
import time
import sys


def compute_crc_xor(payload_bytes: bytes) -> int:
    crc = 0
    for b in payload_bytes:
        crc ^= b
    return crc & 0xFF


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyS4")
    p.add_argument("--baud", type=int, default=460800)
    p.add_argument("--verbose", action="store_true", help="print malformed packet details")
    p.add_argument(
        "--bat-min-mv",
        type=int,
        default=14800,
        help="Battery voltage at 0%% SoC in mV (default: 14800)",
    )
    p.add_argument(
        "--bat-max-mv",
        type=int,
        default=16800,
        help="Battery voltage at 100%% SoC in mV (default: 16800)",
    )
    args = p.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print("Failed to open serial:", e)
        return

    buf = bytearray()

    enc = {"left": 0, "right": 0, "seq": 0}
    imu = {"ax": 0.0, "ay": 0.0, "az": 0.0, "seq": 0}
    bat = {"voltage_mV": 0, "current_mA": 0, "soc": 0.0, "seq": 0}
    detect = {
        "sda": None,
        "scl": None,
        "bus": None,
        "mpu": None,
        "ina": None,
    }

    malformed = 0
    total = 0

    # Print initial block
    sys.stdout.write("\n" * 5)
    sys.stdout.flush()

    def redraw():
        # Move cursor up 5 lines and overwrite
        sys.stdout.write("\x1b[5A")
        sys.stdout.write(
            "\x1b[2K\rEncoder: {} {}  (seq={})\n".format(enc["left"], enc["right"], enc["seq"])
        )
        sys.stdout.write(
            "\x1b[2K\rIMU: {:.6f} {:.6f} {:.6f}  (seq={})\n".format(
                imu["ax"], imu["ay"], imu["az"], imu["seq"]
            )
        )
        sys.stdout.write(
            "\x1b[2K\rBattery: {:.1f}%  ({} mV)\n".format(bat["soc"], bat["voltage_mV"])
        )
        pins = "unknown"
        if detect["sda"] is not None and detect["scl"] is not None:
            if detect["bus"] is not None:
                pins = "bus=i2c{} SDA={} SCL={}".format(detect["bus"], detect["sda"], detect["scl"])
            else:
                pins = "SDA={} SCL={}".format(detect["sda"], detect["scl"])
        mpu = detect["mpu"] if detect["mpu"] is not None else "--"
        ina = detect["ina"] if detect["ina"] is not None else "--"
        sys.stdout.write("\x1b[2K\rI2C: {} | MPU6050={} INA219={}\n".format(pins, mpu, ina))
        sys.stdout.write("\x1b[2K\rMalformed packets: {}  Total: {}\n".format(malformed, total))
        sys.stdout.flush()

    def handle_status_line(line_bytes: bytes) -> bool:
        # Parse plain ASCII boot/status lines emitted by firmware.
        # Examples:
        #   I2C_PINS 10 11
        #   SENSOR MPU6050 0x68
        #   SENSOR INA219 0x40
        try:
            s = line_bytes.decode("ascii", errors="ignore").strip()
        except Exception:
            return False
        if not s:
            return False

        if s.startswith("I2C_PINS "):
            parts = s.split()
            # Supports either:
            #   I2C_PINS <sda> <scl>
            #   I2C_PINS bus=<n> <sda> <scl>
            if len(parts) >= 3:
                try:
                    idx = 1
                    if parts[1].startswith("bus="):
                        detect["bus"] = int(parts[1].split("=", 1)[1])
                        idx = 2
                    detect["sda"] = int(parts[idx])
                    detect["scl"] = int(parts[idx + 1])
                    return True
                except Exception:
                    return False

        if s.startswith("SENSOR "):
            parts = s.split()
            if len(parts) >= 3:
                if parts[1] == "MPU6050":
                    detect["mpu"] = parts[2]
                    return True
                if parts[1] == "INA219":
                    detect["ina"] = parts[2]
                    return True

        if s.startswith("I2C_STATUS "):
            parts = s.split()
            for token in parts[1:]:
                if token.startswith("pins="):
                    try:
                        pair = token.split("=", 1)[1]
                        sda_s, scl_s = pair.split(",", 1)
                        detect["sda"] = int(sda_s)
                        detect["scl"] = int(scl_s)
                    except Exception:
                        pass
                elif token.startswith("bus="):
                    val = token.split("=", 1)[1]
                    try:
                        detect["bus"] = int(val)
                    except Exception:
                        pass
                elif token.startswith("mpu="):
                    val = token.split("=", 1)[1]
                    if val != "0x00":
                        detect["mpu"] = val
                elif token.startswith("ina="):
                    val = token.split("=", 1)[1]
                    if val != "0x00":
                        detect["ina"] = val
            return True

        return False

    def handle_line(line_bytes: bytes):
        nonlocal malformed, total
        # strip trailing CR if present
        if line_bytes.endswith(b"\r"):
            line_bytes = line_bytes[:-1]
        if not line_bytes:
            return
        # find last space (before CRC token)
        last_sp = line_bytes.rfind(b" ")
        if last_sp == -1:
            malformed += 1
            return
        payload = line_bytes[:last_sp]
        crc_token = line_bytes[last_sp + 1 :]
        try:
            expected_crc = int(crc_token.decode("ascii", errors="ignore")) & 0xFF
        except Exception:
            if args.verbose:
                print("Malformed CRC token:", crc_token.hex(), file=sys.stderr)
            malformed += 1
            return
        seen_crc = compute_crc_xor(payload)
        if seen_crc != expected_crc:
            if args.verbose:
                print(
                    "CRC mismatch: payload=",
                    payload.hex(),
                    "seen=",
                    seen_crc,
                    "expected=",
                    expected_crc,
                    file=sys.stderr,
                )
            malformed += 1
            return

        # valid frame
        total += 1
        try:
            s = payload.decode("ascii")
        except Exception:
            if args.verbose:
                print("Malformed payload (decode failed):", payload.hex(), file=sys.stderr)
            malformed += 1
            return
        parts = s.split()
        if len(parts) < 3:
            malformed += 1
            return
        # parts[0] should be 'T'
        if parts[0] != "T":
            malformed += 1
            return
        typ = parts[1]
        if typ == "E":
            # T E <seq> <left> <right>
            if len(parts) >= 5:
                try:
                    seq = int(parts[2])
                    left = int(parts[3])
                    right = int(parts[4])
                    enc["seq"] = seq
                    enc["left"] = left
                    enc["right"] = right
                except Exception:
                    if args.verbose:
                        print("Malformed E frame payload:", payload.hex(), file=sys.stderr)
                    malformed += 1
            else:
                malformed += 1
        elif typ == "I":
            # T I <seq> ax ay az gx gy gz
            if len(parts) >= 9:
                try:
                    seq = int(parts[2])
                    ax = float(parts[3])
                    ay = float(parts[4])
                    az = float(parts[5])
                    # ignore gyros for display
                    imu["seq"] = seq
                    imu["ax"] = ax
                    imu["ay"] = ay
                    imu["az"] = az
                except Exception:
                    if args.verbose:
                        print("Malformed I frame payload:", payload.hex(), file=sys.stderr)
                    malformed += 1
            else:
                malformed += 1
        elif typ == "B":
            # T B <seq> voltage current soc
            if len(parts) >= 5:
                try:
                    seq = int(parts[2])
                    volt = int(parts[3])
                    curr = int(parts[4])
                    bat["seq"] = seq
                    bat["voltage_mV"] = volt
                    bat["current_mA"] = curr
                    v_range = args.bat_max_mv - args.bat_min_mv
                    frac = (volt - args.bat_min_mv) / v_range if v_range > 0 else 0.0
                    bat["soc"] = max(0.0, min(100.0, frac * 100.0))
                except Exception:
                    malformed += 1
            else:
                malformed += 1
        else:
            malformed += 1

    try:
        while True:
            data = ser.read(256)
            if data:
                buf.extend(data)
            # process available frames: prefer binary STX frames, fallback to legacy ASCII lines
            while True:
                if not buf:
                    break
                # find STX
                if 0x02 in buf:
                    stx = buf.index(0x02)
                    # drop leading garbage
                    if stx > 0:
                        del buf[:stx]
                        continue
                    # need at least STX + LEN
                    if len(buf) < 2:
                        break
                    payload_len = buf[1]
                    total_len = 2 + payload_len + 1
                    if len(buf) < total_len:
                        break
                    payload = bytes(buf[2 : 2 + payload_len])
                    crc = buf[2 + payload_len]
                    seen = compute_crc_xor(payload)
                    if seen != crc:
                        malformed += 1
                        if args.verbose:
                            print(
                                "CRC mismatch (binary): seen=0x{:02x} expected=0x{:02x} payload={}".format(
                                    seen, crc, payload.hex()
                                ),
                                file=sys.stderr,
                            )
                        # discard STX and continue
                        del buf[0]
                        continue

                    # valid binary payload
                    total += 1
                    typ = chr(payload[0]) if payload else "?"
                    if typ == "E":
                        if len(payload) >= 1 + 4 + 4 + 4:
                            seq = int.from_bytes(payload[1:5], "little", signed=False)
                            left = int.from_bytes(payload[5:9], "little", signed=True)
                            right = int.from_bytes(payload[9:13], "little", signed=True)
                            enc["seq"] = seq
                            enc["left"] = left
                            enc["right"] = right
                    elif typ == "I":
                        if len(payload) >= 1 + 4 + 6 * 4:
                            import struct

                            seq = int.from_bytes(payload[1:5], "little", signed=False)
                            vals = struct.unpack("<6f", payload[5 : 5 + 24])
                            imu["seq"] = seq
                            (
                                imu["ax"],
                                imu["ay"],
                                imu["az"],
                                imu["gx"],
                                imu["gy"],
                                imu["gz"],
                            ) = vals
                    elif typ == "B":
                        if len(payload) >= 1 + 4 + 4 + 4 + 4:
                            seq = int.from_bytes(payload[1:5], "little", signed=False)
                            volt = int.from_bytes(payload[5:9], "little", signed=True)
                            curr = int.from_bytes(payload[9:13], "little", signed=True)
                            bat["seq"] = seq
                            bat["voltage_mV"] = volt
                            bat["current_mA"] = curr
                            v_range = args.bat_max_mv - args.bat_min_mv
                            frac = (volt - args.bat_min_mv) / v_range if v_range > 0 else 0.0
                            bat["soc"] = max(0.0, min(100.0, frac * 100.0))

                    # consume frame
                    del buf[:total_len]
                    redraw()
                    continue

                # no STX: fallback to LF-terminated legacy lines
                lf = None
                for i in range(len(buf)):
                    if buf[i] == 0x0A:
                        lf = i
                        break
                if lf is None:
                    break
                line = bytes(buf[:lf])
                del buf[: lf + 1]
                if not handle_status_line(line):
                    handle_line(line)
                redraw()

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass
        # move cursor down and show cursor
        sys.stdout.write("\x1b[5B\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
