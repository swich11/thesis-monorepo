import h5py
import torch
import numpy as np


f = h5py.File('../datasets/dataset1.hdf5')

datasets = {}
data_names = (
    "RenderTime",
    "IMULinearAcceleration",
    "IMUAngularVelocity",
    "IMUTime",
    "Velocities",
    "OnEvents",
    "OffEvents",
    "Depths",
    "MotionFlow",
)

for name in data_names:
    datasets[name] = f[name]
    print(f"Loaded {name} with shape {datasets[name].shape}")

f.close()