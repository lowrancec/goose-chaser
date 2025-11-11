#!/usr/bin/env python3
import time, math, sys
import board, busio
from adafruit_lsm303_accel import LSM303_Accel
from adafruit_lsm303dlh_mag import LSM303DLH_Mag
try:
    from adafruit_l3gd20 import L3GD20_I2C
    HAVE_GYRO = True
except Exception:
    HAVE_GYRO = False

# ---- Configure here ----
# Magnetic declination (degrees). West Point, NY is about -13° (W) circa 2025.
MAG_DECLINATION_DEG = -13.0
# Complementary filter weight for gyro integration (0..1). 0 = mag only.
GYRO_ALPHA = 0.92
# Magnetometer hard-iron offsets (set to your calibrated values later)
MAG_OFFSETS = {'x': 0.0, 'y': 0.0, 'z': 0.0}
# ------------------------

def clamp_angle_deg(deg):
    while deg < 0: deg += 360.0
    while deg >= 360.0: deg -= 360.0
    return deg

def tilt_comp_heading_deg(ax, ay, az, mx, my, mz):
    """
    Compute tilt-compensated heading using accel (for roll/pitch) + mag.
    Axis assumptions match Adafruit LSM303 drivers:
      Accel: m/s^2 (x,y,z), Mag: microtesla (x,y,z)
    """
    # Normalize accelerometer (avoid NaNs)
    g = math.sqrt(ax*ax + ay*ay + az*az)
    if g == 0:
        return None

    axn, ayn, azn = ax/g, ay/g, az/g

    # Roll, Pitch (right-hand; radians)
    roll  = math.atan2(ayn, azn)
    pitch = math.atan2(-axn, math.sqrt(ayn*ayn + azn*azn))

    # Apply hard-iron offsets to mag
    mx -= MAG_OFFSETS['x']
    my -= MAG_OFFSETS['y']
    mz -= MAG_OFFSETS['z']

    # Tilt compensation
    mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
    my2 = (mx * math.sin(roll) * math.sin(pitch)
           + my * math.cos(roll)
           - mz * math.sin(roll) * math.cos(pitch))

    heading = -math.degrees(math.atan2(-my2, mx2))  # atan2(y, x); sign matches LSM303 orientation
    heading = (heading + MAG_DECLINATION_DEG)  # add declination for TRUE heading
    return clamp_angle_deg(heading)

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    # Sensors
    accel = LSM303_Accel(i2c)             # LSM303 accel
    mag   = LSM303DLH_Mag(i2c)            # LSM303 magnetometer
    gyro  = None
    if HAVE_GYRO:
        try:
            gyro = L3GD20_I2C(i2c, address=0x6B)  # try 0x6B
        except Exception:
            try:
                gyro = L3GD20_I2C(i2c, address=0x6A)  # fallback
            except Exception:
                gyro = None

    print("Reading IMU... Ctrl+C to stop.")
    prev_t = time.monotonic()
    fused_yaw = None  # degrees

    # Simple moving baseline for mag-only yaw if starting fresh
    while True:
        try:
            ax, ay, az = accel.acceleration  # m/s^2
            mx, my, mz = mag.magnetic        # microtesla

            now = time.monotonic()
            dt = now - prev_t
            if dt <= 0: dt = 1e-3
            prev_t = now

            # Magnetic (tilt-comp) yaw
            mag_yaw = tilt_comp_heading_deg(ax, ay, az, mx, my, mz)

            # Initialize fused yaw from mag on first pass
            if fused_yaw is None and mag_yaw is not None:
                fused_yaw = mag_yaw

            # Gyro fusion (optional)
            if gyro is not None:
                # L3GD20 returns dps (x,y,z). z ≈ yaw rate (about vertical) if board flat.
                gx, gy, gz = gyro.gyro  # degrees/sec
                if fused_yaw is None:
                    fused_yaw = 0.0
                fused_yaw = fused_yaw + gz * dt  # integrate
                fused_yaw = clamp_angle_deg(fused_yaw)

                if mag_yaw is not None:
                    # Complementary blend: high-pass gyro, low-pass mag
                    fused_yaw = GYRO_ALPHA * fused_yaw + (1.0 - GYRO_ALPHA) * mag_yaw
                    fused_yaw = clamp_angle_deg(fused_yaw)
                out_yaw = fused_yaw
            else:
                # No gyro: just use mag-based yaw
                out_yaw = mag_yaw if mag_yaw is not None else float('nan')

            print(f"Yaw (deg): {out_yaw:7.2f}   (mag:{mag_yaw:7.2f}  dt:{dt*1000:5.1f} ms)")
            time.sleep(0.05)  # ~20 Hz
        except KeyboardInterrupt:
            print("\nStopping.")
            break
        except Exception as e:
            # Keep running on sporadic read hiccups
            print("Read error:", e)
            time.sleep(0.05)

if __name__ == "__main__":
    main()
