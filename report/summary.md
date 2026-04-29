# AerNav v2 — Project Summary

## Resume Bullets

- Built AerNav v2 — GPS-denied drone autonomy combining EKF sensor fusion, potential field obstacle avoidance, and PyTorch MLP threat detection
- Validated EKF on real DJI M300 flight data (54,726 IMU samples), achieving 10x position accuracy improvement over IMU-only baseline
- Trained MLP threat classifier on real drone telemetry: 97.8% accuracy, F1=0.98
- Full autonomy pipeline: state estimation -> obstacle avoidance -> threat detection -> evasion -> goal navigation

## Key Results

| Experiment | Result |
|---|---|
| EKF vs IMU (simulation) | 583x better RMSE |
| EKF vs IMU (real DJI M300) | 10x better RMSE |
| Threat classifier accuracy | 97.8% |
| Full mission | Goal reached |

## Limitations

- Simulation only — not tested on real hardware
- Threat classifier trained on flight state proxies
- Simplified drone dynamics
