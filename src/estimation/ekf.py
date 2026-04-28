import numpy as np


class ExtendedKalmanFilter:
    def __init__(self):
        self.state    = np.zeros(9)
        self.state[2] = 0.1125
        self.P        = np.eye(9) * 0.1
        self.Q        = np.eye(9) * 0.01
        self.R_baro   = np.array([[0.1]])
        self.R_flow   = np.eye(2) * 0.05
        self.dt       = 1.0 / 100.0

    def predict(self, accel_reading, gyro_reading):
        px, py, pz       = self.state[0], self.state[1], self.state[2]
        vx, vy, vz       = self.state[3], self.state[4], self.state[5]
        roll, pitch, yaw = self.state[6], self.state[7], self.state[8]
        dt = self.dt

        new_px = px + vx * dt
        new_py = py + vy * dt
        new_pz = pz + vz * dt
        new_vx = vx + accel_reading[0] * dt
        new_vy = vy + accel_reading[1] * dt
        new_vz = vz + (accel_reading[2] - 9.81) * dt
        new_roll  = roll  + gyro_reading[0] * dt
        new_pitch = pitch + gyro_reading[1] * dt
        new_yaw   = yaw   + gyro_reading[2] * dt

        self.state = np.array([
            new_px, new_py, new_pz,
            new_vx, new_vy, new_vz,
            new_roll, new_pitch, new_yaw
        ])

        F       = np.eye(9)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        self.P  = F @ self.P @ F.T + self.Q

    def update_baro(self, baro_reading):
        H       = np.zeros((1, 9))
        H[0, 2] = 1.0
        z          = np.array([[baro_reading]])
        z_pred     = H @ self.state.reshape(-1, 1)
        innovation = z - z_pred
        S = H @ self.P @ H.T + self.R_baro
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + (K @ innovation).flatten()
        self.P     = (np.eye(9) - K @ H) @ self.P

    def update_optical_flow(self, flow_reading):
        H       = np.zeros((2, 9))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        z          = flow_reading.reshape(-1, 1)
        z_pred     = H @ self.state.reshape(-1, 1)
        innovation = z - z_pred
        S = H @ self.P @ H.T + self.R_flow
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + (K @ innovation).flatten()
        self.P     = (np.eye(9) - K @ H) @ self.P

    def get_position(self):    return self.state[0:3]
    def get_velocity(self):    return self.state[3:6]
    def get_orientation(self): return self.state[6:9]
    def get_uncertainty(self): return np.trace(self.P)
