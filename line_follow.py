#!/usr/bin/env python3
"""
Instrumented 8-channel line follower for Yahboom Mini Car chassis.
Includes benchmarks for I²C and motor commands, loop timing measurements, and spin test.
Use --bench to run benchmarks only, --debug to enable detailed prints.
"""

import smbus
import time
import argparse
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS       = 1
I2C_ADDR      = 0x12       # sensor I²C address
SENSOR_REG    = 0x30       # data register (8-bit mask)
SENSOR_EN_REG = 0x01       # control register for IR LEDs

MD_PORT       = "/dev/ttyUSB0"
MD_TYPE       = 2
UPLOAD_DATA   = 0          # telemetry off for lean loop

BASE_SPEED    = 350        # forward speed (200–500)
KP            = 70         # proportional gain (10–100)
MAX_CMD       = 600        # clamp for wheel commands

# If + speeds drive you backward, set to -1 (else +1)
FORWARD_SIGN  = -1        

# Weights for sensors 1…8 (leftmost → rightmost)
WEIGHTS       = [-9.0, -7.0, -4.0, -2.0, +2.0, +4.0, +7.0, +9.0]

LOOP_DELAY    = 0.01      # 10ms loop delay
LOOP_DEBUG_INTERVAL = 50  # print loop timing every N iterations
# ————————————————————————————————————————————————————————

# Global debug flag; set from CLI
DEBUG = False


def clamp(v, lo, hi):
    return max(min(v, hi), lo)


def init_sensor():
    """Wake up sensor by toggling IR emitters."""
    bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
    time.sleep(0.02)
    bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
    time.sleep(0.02)


def read_sensors():
    """
    Read raw byte; return (raw, bits[0..7]) where bits[0]=leftmost.
    Invert so black=1, white=0. Retries on I/O errors.
    """
    while True:
        try:
            raw = bus.read_byte_data(I2C_ADDR, SENSOR_REG)
            bits = [
                1 - ((raw >> (7 - i)) & 1)
                for i in range(8)
            ]
            return raw, bits
        except BlockingIOError:
            time.sleep(0.005)
        except OSError as e:
            if e.errno == 121:
                time.sleep(0.01)
            else:
                raise


def compute_error(bits):
    """Weighted sum: negative=line left, positive=line right."""
    return sum(w * b for w, b in zip(WEIGHTS, bits))


def bench_i2c(n=200):
    """Benchmark average I2C read time."""
    start = time.perf_counter()
    for _ in range(n):
        bus.read_byte_data(I2C_ADDR, SENSOR_REG)
    end = time.perf_counter()
    avg = (end - start) / n * 1000
    print(f"I2C read avg = {avg:.2f} ms over {n} reads")


def bench_motor():
    """Benchmark average motor_driver.control_speed call."""
    N = 50
    start = time.perf_counter()
    for _ in range(N):
        md.control_speed(200,200,200,200)
    end = time.perf_counter()
    avg = (end - start) / N * 1000
    print(f"Motor write avg = {avg:.2f} ms over {N} commands")


def spin_test():
    """Spin in place right then left to confirm FORWARD_SIGN and wiring."""
    print("→ Spinning right (200,-200,200,-200)...")
    md.control_speed(200, -200, 200, -200)
    time.sleep(1)
    md.control_speed(0,0,0,0)
    time.sleep(0.5)
    print("← Spinning left (-200,200,-200,200)...")
    md.control_speed(-200, 200, -200, 200)
    time.sleep(1)
    md.control_speed(0,0,0,0)
    print("Spin test complete.\n")


def follow_line():
    """Main line-following loop with timing instrumentation."""
    print("Starting line follower. Press Ctrl-C to stop.")
    loop_count = 0
    last_time = time.perf_counter()
    try:
        while True:
            now = time.perf_counter()
            dt = now - last_time
            last_time = now
            loop_count += 1

            raw, bits = read_sensors()
            total = sum(bits)
            if total == 0:
                # lost line → stop
                md.control_speed(0,0,0,0)
                if DEBUG:
                    print(f"LOST: raw=0x{raw:02X} bits={bits}")
            else:
                err = compute_error(bits)
                turn = KP * err
                l = clamp(BASE_SPEED + turn, -MAX_CMD, MAX_CMD)
                r = clamp(BASE_SPEED - turn, -MAX_CMD, MAX_CMD)
                if DEBUG:
                    print(f"bits={bits} err={err:.2f} l={l:.0f} r={r:.0f}")
                md.control_speed(
                    FORWARD_SIGN * r,
                    FORWARD_SIGN * r,
                    FORWARD_SIGN * l,
                    FORWARD_SIGN * l
                )

            if loop_count % LOOP_DEBUG_INTERVAL == 0:
                print(f"Loop Δt = {dt*1000:.1f} ms")

            time.sleep(LOOP_DELAY)
    except KeyboardInterrupt:
        print("Stopping motors.")
        md.control_speed(0,0,0,0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bench", action="store_true", help="Run benchmarks only and exit.")
    parser.add_argument("--debug", action="store_true", help="Enable debug prints.")
    args = parser.parse_args()
    DEBUG = args.debug

    # Initialize I2C and sensor
    bus = smbus.SMBus(I2C_BUS)
    init_sensor()

    # Initialize motor driver
    md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)

    if args.bench:
        bench_i2c()
        bench_motor()
        exit(0)

    # Validate wiring and direction
    spin_test()

    # Start line-following
    follow_line()
