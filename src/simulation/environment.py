import numpy as np
from dataclasses import dataclass

@dataclass
class DroneState:
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    t:   float = 0.0

@dataclass
class Obstacle:
    center: np.ndarray
    size:   np.ndarray
    def contains(self, point, margin=0.5):
        return np.all(np.abs(point - self.center) < self.size + margin)

class AerNavEnvironment:
    def __init__(self, bounds=np.array([100.,100.,50.]), n_obstacles=8, seed=42):
        np.random.seed(seed)
        self.bounds    = bounds
        self.obstacles = []
        self.start     = np.array([5., 5., 5.])
        self.goal      = np.array([90., 90., 20.])
        self._place_obstacles(n_obstacles)

    def _place_obstacles(self, n):
        placed = 0
        while placed < n:
            c = np.array([np.random.uniform(15,85),
                          np.random.uniform(15,85),
                          np.random.uniform(2,25)])
            s = np.array([np.random.uniform(3,8),
                          np.random.uniform(3,8),
                          np.random.uniform(5,20)])
            obs = Obstacle(c, s)
            if obs.contains(self.start, 5.0): continue
            if obs.contains(self.goal,  5.0): continue
            self.obstacles.append(obs)
            placed += 1

    def check_collision(self, pos):
        return any(o.contains(pos) for o in self.obstacles)
    def distance_to_goal(self, pos):
        return float(np.linalg.norm(self.goal - pos))
    def is_goal_reached(self, pos, threshold=3.0):
        return self.distance_to_goal(pos) < threshold

class DroneDynamics:
    def __init__(self, max_speed=5.0, max_acc=3.0, drag=0.1, dt=0.05):
        self.max_speed = max_speed
        self.max_acc   = max_acc
        self.drag      = drag
        self.dt        = dt

    def step(self, state, vel_cmd, bounds):
        speed = np.linalg.norm(vel_cmd)
        if speed > self.max_speed:
            vel_cmd = vel_cmd / speed * self.max_speed
        acc     = (vel_cmd - state.vel)*self.max_acc - state.vel*self.drag
        acc     = np.clip(acc, -self.max_acc, self.max_acc)
        new_vel = state.vel + acc*self.dt
        new_pos = np.clip(state.pos + state.vel*self.dt + 0.5*acc*self.dt**2,
                          [0,0,0.5], bounds)
        return DroneState(pos=new_pos, vel=new_vel, acc=acc, t=state.t+self.dt)
