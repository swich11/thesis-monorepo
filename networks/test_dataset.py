import h5py
import torch
import numpy as np


f = h5py.File('../datasets/dataset0.hdf5')

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
    "GrayscaleImage",
)

for name in data_names:
    datasets[name] = f[name]
    print(f"Loaded {name} with shape {datasets[name].shape}")


print(datasets["GrayscaleImage"][0])



f.close()