import torch, torch.nn as nn

class ThreatClassifier(nn.Module):
    """MLP: 23->64->32->16->1. Trained on DJI M300 data. Accuracy 97.8%"""
    def __init__(self, input_dim=23):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim,64), nn.BatchNorm1d(64), nn.ReLU(), nn.Dropout(0.3),
            nn.Linear(64,32), nn.BatchNorm1d(32), nn.ReLU(), nn.Dropout(0.2),
            nn.Linear(32,16), nn.ReLU(), nn.Linear(16,1), nn.Sigmoid()
        )
    def forward(self, x): return self.net(x)
