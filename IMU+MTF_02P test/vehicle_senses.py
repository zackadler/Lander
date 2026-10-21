"""
Poll the IMU and the MTF-02P together on a single, fixed cycle,
instead of each running its own independent loop. Both sensors are
constructed once at startup -- which runs the IMU calibration and
opens the MTF-02P's serial port -- and then .read() is called on each
once per cycle.
"""

import time

from IMU_function import ImuSensor
from MTF_02P_function import Mtf02pSensor

POLL_INTERVAL_S = 0.02


def format_reading(imu_reading, flow_reading):
    accel = (
        f"accel x={imu_reading['accel_x']:6.2f} y={imu_reading['accel_y']:6.2f} "
        f"z={imu_reading['accel_z']:6.2f} m/s^2  "
        f"roll={imu_reading['roll']:6.1f} pitch={imu_reading['pitch']:6.1f} deg"
    )

    if flow_reading["distance_m"] is not None:
        dist = f"{flow_reading['distance_m']:.2f} m (q={flow_reading['rangefinder_quality']})"
    else:
        dist = "no reading yet"

    if flow_reading["motion_x"] is not None:
        flow = (
            f"({flow_reading['motion_x']:+d}, {flow_reading['motion_y']:+d}) "
            f"(q={flow_reading['flow_quality']})"
        )
    else:
        flow = "no reading yet"

    return f"{accel}  |  dist={dist}  |  flow={flow}"


def main():
    imu = ImuSensor()
    flow_sensor = Mtf02pSensor()

    print(f"\nPolling both sensors every {POLL_INTERVAL_S}s. Ctrl+C to stop.\n")

    while True:
        imu_reading = imu.read()
        flow_reading = flow_sensor.read()

        print(format_reading(imu_reading, flow_reading))

        time.sleep(POLL_INTERVAL_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")