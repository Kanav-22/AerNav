import numpy as np
import matplotlib.pyplot as plt
from gym_pybullet_drones.envs import HoverAviary


class GPSSensor:
    def __init__(self):
        self.noise_std = 0.02

    def read(self, true_position):
        noise = np.random.normal(0, self.noise_std, size=3)
        return true_position + noise


class IMUSensor:
    def __init__(self):
        self.accel_noise_std = 0.01
        self.gyro_noise_std  = 0.001
        self.accel_bias      = np.zeros(3)
        self.gyro_bias       = np.zeros(3)

    def update_bias(self):
        self.accel_bias += np.random.normal(0, 0.0001, size=3)
        self.gyro_bias  += np.random.normal(0, 0.00001, size=3)

    def read(self, true_acceleration, true_angular_velocity):
        self.update_bias()
        accel_noise   = np.random.normal(0, self.accel_noise_std, size=3)
        gyro_noise    = np.random.normal(0, self.gyro_noise_std, size=3)
        accel_reading = true_acceleration     + accel_noise + self.accel_bias
        gyro_reading  = true_angular_velocity + gyro_noise  + self.gyro_bias
        return accel_reading, gyro_reading


class BarometerSensor:
    def __init__(self):
        self.noise_std = 0.1

    def read(self, true_position):
        true_altitude = true_position[2]
        noise = np.random.normal(0, self.noise_std)
        return true_altitude + noise


class OpticalFlowSensor:
    def __init__(self):
        self.noise_std    = 0.05
        self.max_altitude = 5.0

    def read(self, true_velocity, true_altitude):
        if true_altitude > self.max_altitude:
            return None
        true_horizontal_vel = np.array([true_velocity[0], true_velocity[1]])
        noise = np.random.normal(0, self.noise_std, size=2)
        return true_horizontal_vel + noise


class SensorSuite:
    def __init__(self, gps_enabled=True):
        self.gps          = GPSSensor()
        self.imu          = IMUSensor()
        self.barometer    = BarometerSensor()
        self.optical_flow = OpticalFlowSensor()
        self.gps_enabled  = gps_enabled

    def read_all(self, true_position, true_velocity, true_acceleration, true_angular_velocity):
        readings = {}
        readings["gps"] = self.gps.read(true_position) if self.gps_enabled else None
        readings["accel"], readings["gyro"] = self.imu.read(true_acceleration, true_angular_velocity)
        readings["baro"] = self.barometer.read(true_position)
        readings["optical_flow"] = self.optical_flow.read(true_velocity, true_position[2])
        return readings
