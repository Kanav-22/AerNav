import numpy as np

class ObstacleAvoider:
    """Artificial Potential Fields obstacle avoidance."""
    def __init__(self, attract_gain=1.5, repel_gain=80.0,
                 repel_radius=12.0, max_speed=5.0):
        self.k_att = attract_gain; self.k_rep = repel_gain
        self.r_rep = repel_radius; self.v_max = max_speed

    def compute_velocity(self, pos, goal, obstacles):
        d = np.linalg.norm(goal-pos)
        v = self.k_att*(goal-pos)/(d+1e-6)
        for obs in obstacles:
            to  = pos-obs.center; dist = np.linalg.norm(to)
            rad = np.linalg.norm(obs.size)+self.r_rep
            if dist < rad:
                v += self.k_rep*(1/dist-1/rad)/(dist**2)*to/(dist+1e-6)
        sp = np.linalg.norm(v)
        return v/sp*self.v_max if sp > self.v_max else v

class InterceptorDrone:
    """Simulated hostile drone that pursues target."""
    def __init__(self, start_pos, speed=4.5, aggression=0.85, dt=0.05):
        self.pos=start_pos.copy(); self.vel=np.zeros(3)
        self.speed=speed; self.agg=aggression; self.dt=dt

    def update(self, target):
        d = np.linalg.norm(target-self.pos)
        if d > 0.1:
            self.vel = (1-self.agg)*self.vel + self.agg*(target-self.pos)/d*self.speed
            self.pos += self.vel*self.dt
        return self.pos.copy()

    def is_intercept(self, target, threshold=2.5):
        return float(np.linalg.norm(self.pos-target)) < threshold
