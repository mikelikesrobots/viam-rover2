#!/usr/bin/env python3
"""Motor diagnostic: sweep raw PWM -> speed data and auto-calculate calibration parameters.

Calibration model (linear with dead zone):
    actual_speed = PWM_FACTOR * (pwm - dead_zone)   [when pwm > dead_zone]
    actual_speed = 0                                  [when pwm <= dead_zone]

Inverse (used to command a desired speed):
    pwm = speed / PWM_FACTOR + dead_zone

Output at the end of a run prints the calibrated constants ready to paste into
rover2_system.cpp.

Requires: pyserial
"""

import math
import serial
import threading
import time
from typing import Optional, Tuple

LEFT_TICKS_PER_REV = 1570
RIGHT_TICKS_PER_REV = 1570
MEASURE_TIME = 4.0  # seconds per PWM step


def _crc_xor(payload: bytes) -> int:
    crc = 0
    for b in payload:
        crc ^= b
    return crc & 0xFF


def _send_cmd(ser, cmd: str) -> None:
    if not cmd.endswith("\n"):
        cmd += "\n"
    ser.write(cmd.encode())
    ser.flush()


def _encoder_reader(ser, stop_event: threading.Event, callback) -> None:
    buf = bytearray()
    while not stop_event.is_set():
        try:
            data = ser.read(256)
        except Exception:
            continue
        if data:
            buf.extend(data)
        while buf:
            if 0x02 not in buf:
                buf.clear()
                break
            stx = buf.index(0x02)
            if stx > 0:
                del buf[:stx]
                continue
            if len(buf) < 2:
                break
            payload_len = buf[1]
            total_len = 2 + payload_len + 1
            if len(buf) < total_len:
                break
            payload = bytes(buf[2 : 2 + payload_len])
            crc = buf[2 + payload_len]
            if _crc_xor(payload) != crc:
                del buf[0]
                continue
            if payload_len >= 13 and chr(payload[0]) == "E":
                left = int.from_bytes(payload[5:9], byteorder="little", signed=True)
                right = int.from_bytes(payload[9:13], byteorder="little", signed=True)
                callback((left, right))
            del buf[:total_len]


class Rp2040Bridge:
    """Thread-safe serial interface for the RP2040 firmware."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        if serial is None:
            raise RuntimeError("pyserial required: pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._latest_enc: Optional[Tuple[int, int]] = None

    def _on_enc(self, tup) -> None:
        with self._lock:
            self._latest_enc = (int(tup[0]), int(tup[1]))

    def start_reader(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=_encoder_reader, args=(self.ser, self._stop, self._on_enc), daemon=True
        )
        self._thread.start()

    def stop_reader(self) -> None:
        if self._thread:
            self._stop.set()
            self._thread.join(timeout=1.0)

    def stop_motors(self) -> None:
        _send_cmd(self.ser, "S")

    def set_motors(self, left: float, right: float) -> None:
        def norm(v):
            if isinstance(v, int) or abs(v) > 1.0:
                return float(v) / 255.0
            return float(v)
        _send_cmd(self.ser, f"M {norm(left):.3f} {norm(right):.3f}")

    def get_latest_encoders(self, timeout: float = 1.0) -> Optional[Tuple[int, int]]:
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                if self._latest_enc is not None:
                    return self._latest_enc
            time.sleep(0.01)
        return None


def linfit(data: list[tuple[float, float]]) -> tuple[float, float, float]:
    """OLS linear fit y = a*x + b. Returns (slope, intercept, dead_zone)."""
    n = len(data)
    sx = sum(x for x, y in data)
    sy = sum(y for x, y in data)
    sx2 = sum(x * x for x, y in data)
    sxy = sum(x * y for x, y in data)
    denom = n * sx2 - sx * sx
    if denom == 0:
        raise ValueError("Degenerate data: all x values are identical")
    a = (n * sxy - sx * sy) / denom
    b = (sy - a * sx) / n
    return a, b, -b / a


class MotorDiagnostic:
    def __init__(self):
        self.bridge = Rp2040Bridge("/dev/ttyS4", 115200, timeout=0.5)
        self.bridge.start_reader()

    def stop(self):
        self.bridge.stop_motors()
        self.bridge.stop_reader()

    def test_raw_pwm(self, pwm_left, pwm_right) -> tuple[float, float] | None:
        """Command a fixed PWM for MEASURE_TIME seconds and return (left, right) rad/s."""
        print(f"\nTesting PWM: left={pwm_left:.3f}, right={pwm_right:.3f}")

        start_enc = self.bridge.get_latest_encoders(timeout=1.0)
        if start_enc is None:
            print("  ERROR: No encoder data!")
            return None
        start_time = time.time()

        while time.time() - start_time < MEASURE_TIME:
            self.bridge.set_motors(pwm_left, pwm_right)
            time.sleep(0.1)

        self.bridge.stop_motors()
        time.sleep(0.2)

        end_enc = self.bridge.get_latest_encoders(timeout=1.0)
        dt = time.time() - start_time
        if end_enc is None or dt <= 0:
            print("  ERROR: No encoder data after run!")
            return None

        delta_left = end_enc[0] - start_enc[0]
        delta_right = end_enc[1] - start_enc[1]
        left_rad_s = (delta_left / LEFT_TICKS_PER_REV) * 2 * math.pi / dt
        right_rad_s = (delta_right / RIGHT_TICKS_PER_REV) * 2 * math.pi / dt

        print(f"  Elapsed: {dt:.2f}s")
        print(f"  Tick delta:  left={delta_left:+8d}, right={delta_right:+8d}")
        print(f"  Speed:       left={left_rad_s:+7.3f} rad/s, right={right_rad_s:+7.3f} rad/s")

        if delta_left == 0 and delta_right == 0:
            print("  WARNING: No motion detected!")
        elif delta_left == 0 or delta_right == 0:
            print("  WARNING: One motor not responding!")
        if abs(left_rad_s - right_rad_s) > 5:
            print("  WARNING: Large motor asymmetry!")

        return left_rad_s, right_rad_s

    def run_calibration_sweep(self):
        """Sweep motors across the full PWM range, fit a linear model, and print calibration constants."""
        print("=" * 60)
        print("Motor Calibration Sweep")
        print("=" * 60)

        pwm_values = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00]

        print("\nSweeping both motors across full PWM range...")
        print("-" * 60)

        left_data: list[tuple[float, float]] = []
        right_data: list[tuple[float, float]] = []

        for pwm in pwm_values:
            result = self.test_raw_pwm(pwm, pwm)
            time.sleep(0.5)
            if result is None:
                continue
            left_rad_s, right_rad_s = result
            if left_rad_s > 0.1:
                left_data.append((pwm, left_rad_s))
            if right_rad_s > 0.1:
                right_data.append((pwm, right_rad_s))

        print("\n" + "=" * 60)
        print("Calibration Results")
        print("=" * 60)

        ld = rd = None
        if len(left_data) < 3:
            print("ERROR: Not enough left motor data for a fit (need >= 3 moving points).")
        else:
            la, lb, ld = linfit(left_data)
            print(f"\nLeft motor:")
            print(f"  PWM factor (slope) : {la:.4f} rad/s per PWM unit")
            print(f"  Dead zone          : {ld:.4f} PWM")
            print(f"  Data points        : {left_data}")

        if len(right_data) < 3:
            print("ERROR: Not enough right motor data for a fit (need >= 3 moving points).")
        else:
            ra, rb, rd = linfit(right_data)
            print(f"\nRight motor:")
            print(f"  PWM factor (slope) : {ra:.4f} rad/s per PWM unit")
            print(f"  Dead zone          : {rd:.4f} PWM")
            print(f"  Data points        : {right_data}")

        if ld is not None and rd is not None:
            print("\n" + "=" * 60)
            print("Deadzones (PWM threshold where motion begins):")
            print("-" * 60)
            print(f"  Left  deadzone: {ld:.4f} PWM")
            print(f"  Right deadzone: {rd:.4f} PWM")
            self._print_lut(left_data, ld, right_data, rd)

    def _print_lut(self, left_data, left_deadzone, right_data, right_deadzone):
        """Print C++ LUT initialiser blocks ready to paste into rover2_system.cpp."""
        print("\n" + "=" * 60)
        print("Paste the block below into rover2_system.cpp on_init(),")
        print("replacing the existing motor_lut_ placeholder entries:")
        print("-" * 60)
        for side, data, dz in (("left", left_data, left_deadzone), ("right", right_data, right_deadzone)):
            idx = 0 if side == "left" else 1
            print(f"  motor_lut_[{idx}] = {{  // {side} motor")
            print(f"      {{{dz:.4f}, 0.0000}},  // deadzone — minimum PWM to produce motion")
            for pwm, rad_s in data:
                print(f"      {{{pwm:.4f}, {rad_s:.4f}}},")
            print("  };")
        print("=" * 60)


def main():
    diag = MotorDiagnostic()
    try:
        diag.run_calibration_sweep()
    finally:
        diag.stop()


if __name__ == "__main__":
    main()
