import numpy as np

class ExtendedKalmanFilter:
    """9-state EKF from scratch. State: [px,py,pz,vx,vy,vz,roll,pitch,yaw]"""
    def __init__(self, dt=0.05):
        self.dt = dt; self.n = 9
        self.x  = np.zeros(9)
        self.P  = np.eye(9)*0.1
        self.Q  = np.eye(9)
        self.Q[:3,:3] *= 0.01; self.Q[3:6,3:6] *= 0.1; self.Q[6:,6:] *= 0.001
        self.R_baro = np.array([[0.01]]); self.R_flow = np.eye(2)*0.0025
        self.R_gps  = np.eye(3)*0.0004

    def predict(self, accel, gyro):
        a = accel - np.array([0,0,9.81])
        self.x[:3]  += self.x[3:6]*self.dt + 0.5*a*self.dt**2
        self.x[3:6] += a*self.dt; self.x[6:] += gyro*self.dt
        F = np.eye(9); F[:3,3:6] = np.eye(3)*self.dt
        self.P = F@self.P@F.T + self.Q

    def update_baro(self, baro):
        H = np.zeros((1,9)); H[0,2]=1.0
        self._update(np.array([baro]), H, self.R_baro)

    def update_flow(self, flow):
        H = np.zeros((2,9)); H[0,3]=1.0; H[1,4]=1.0
        self._update(flow, H, self.R_flow)

    def update_gps(self, gps):
        H = np.zeros((3,9)); H[:3,:3]=np.eye(3)
        self._update(gps, H, self.R_gps)

    def _update(self, z, H, R):
        y = z - H@self.x; S = H@self.P@H.T+R
        K = self.P@H.T@np.linalg.inv(S)
        self.x += K@y; self.P = (np.eye(9)-K@H)@self.P

    def get_position(self): return self.x[:3].copy()
    def get_velocity(self): return self.x[3:6].copy()
