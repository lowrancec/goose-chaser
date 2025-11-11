#!/usr/bin/env python3
import time, math
import board, busio
from adafruit_lsm9ds1 import LSM9DS1_I2C

# --------- CONFIGURE HERE ----------
# True-heading correction (mag declination). West Point, NY ≈ -13° (W).
MAG_DECLINATION_DEG = -13.0

# If your board is mounted rotated relative to robot frame (+CW from above)
MOUNT_YAW_DEG = 0.0

# If your *robot* uses X-forward, Y-right, Z-up but sensor arrows differ,
# remap magnetometer axes here (choose from 'x','y','z' with optional '-').
# Common cases:
#   no rotation:        MX_MAP='x',  MY_MAP='y'
#   sensor 90° CW:      MX_MAP='y',  MY_MAP='-x'
#   sensor 90° CCW:     MX_MAP='-y', MY_MAP='x'
MX_MAP, MY_MAP = 'x', 'y'  # start with this; change if you see a 90° offset

# Hard-iron offsets (set after calibrating; temporary zeros are fine)
MAG_OFFSETS = {'x': 0.0, 'y': 0.0, 'z': 0.0}

# Complementary filter weight for gyro+mag fusion (0..1). 0 = mag only.
GYRO_ALPHA = 0.92
# -----------------------------------

def clamp_angle_deg(a):
    while a < 0: a += 360.0
    while a >= 360.0: a -= 360.0
    return a

def remap_axes(mx, my, mz, MX_MAP, MY_MAP):
    def pick(spec):
        s = -1.0 if spec.startswith('-') else 1.0
        key = spec[1:] if spec.startswith('-') else spec
        return s * {'x': mx, 'y': my, 'z': mz}[key]
    return pick(MX_MAP), pick(MY_MAP), mz  # keep z as-is

def tilt_comp_heading_deg(ax, ay, az, mx, my, mz):
    # Normalize accel to get roll/pitch
    g = math.sqrt(ax*ax + ay*ay + az*az)
    if g == 0:
        return None
    axn, ayn, azn = ax/g, ay/g, az/g

    # Roll & pitch (radians)
    roll  = math.atan2(ayn, azn)
    pitch = math.atan2(-axn, math.sqrt(ayn*ayn + azn*azn))

    # Hard-iron correction
    mx -= MAG_OFFSETS['x']
    my -= MAG_OFFSETS['y']
    mz -= MAG_OFFSETS['z']

    # Tilt compensation
    mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
    my2 = (mx * math.sin(roll) * math.sin(pitch)
           + my * math.cos(roll)
           - mz * math.sin(roll) * math.cos(pitch))

    # Heading (CW-positive from +X forward, +Y right)
    heading = math.degrees(math.atan2(-my2, mx2))
    heading = clamp_angle_deg(heading + MAG_DECLINATION_DEG)
    return heading

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    imu = LSM9DS1_I2C(i2c)
    #imu = LSM9DS1_I2C(i2c, mag_address=0x1E, accel_gyro_address=0x6B)
    # If your addresses differ, change the two above (try 0x1C and/or 0x6A)

    print("Reading LSM9DS1... Ctrl+C to stop.")
    prev_t = time.monotonic()
    fused_yaw = None

    while True:
        try:
            # Units: accel m/s^2, gyro rad/s, mag gauss (CircuitPython returns gauss)
            ax, ay, az = imu.acceleration
            gx, gy, gz = imu.gyro
            mx, my, mz = imu.magnetic

            # Convert gyro to deg/s
            gz_dps = gz * (180.0 / math.pi)

            # Remap mag to robot frame if needed
            mx, my, mz = remap_axes(mx, my, mz, MX_MAP, MY_MAP)

            now = time.monotonic()
            dt = now - prev_t
            if dt <= 0: dt = 1e-3
            prev_t = now

            mag_yaw = tilt_comp_heading_deg(ax, ay, az, mx, my, mz)
            if mag_yaw is not None:
                mag_yaw = clamp_angle_deg(mag_yaw + MOUNT_YAW_DEG)

            # Initialize fused yaw from mag on first good sample
            if fused_yaw is None and mag_yaw is not None:
                fused_yaw = mag_yaw

            # Complementary fusion: integrate gyro Z and blend with mag yaw
            if fused_yaw is None:
                fused_yaw = 0.0
            fused_yaw = clamp_angle_deg(fused_yaw + gz_dps * dt)  # integrate gyro
            if mag_yaw is not None:
                fused_yaw = clamp_angle_deg(GYRO_ALPHA * fused_yaw + (1.0 - GYRO_ALPHA) * mag_yaw)

            print(f"Yaw (deg): {fused_yaw:7.2f}   (mag:{(mag_yaw if mag_yaw is not None else float('nan')):7.2f}  dt:{dt*1000:5.1f} ms)")
            time.sleep(0.05)  # ~20 Hz
        except KeyboardInterrupt:
            print("\nStopping.")
            break
        except Exception as e:
            print("Read error:", e)
            time.sleep(0.05)

if __name__ == "__main__":
    main()

