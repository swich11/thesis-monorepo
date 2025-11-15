from sys import argv


import h5py
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
from snntorch import utils


from DepthMapNet import DepthMapNet
from DepthMapVelocityNet import DepthMapVelocityNet
from OpticalFlowNet import OpticalFlowNet
from OpticalFlowVelocityNet import OpticalFlowVelocityNet
from VelocityNet import VelocityNet
from utils import calculate_depth_loss, calculate_flow_loss, calculate_velocity_loss


from enum import Enum
from typing import List


class DataName(Enum):
    RENDER_TIME = 0,
    PHYSICS_TIME = 1,
    IMU_LIN_ACC = 2,
    IMU_ANG_VEL = 3,
    FRAME_VELOCITIES = 4,
    ON_EVENTS = 5,
    OFF_EVENTS = 6,
    DEPTH_MAP = 7,
    MOTION_FLOW = 8,

name_map = {
    DataName.RENDER_TIME: "RenderTime",
    DataName.PHYSICS_TIME: "IMUTime",
    DataName.IMU_LIN_ACC: "IMULinearAcceleration",
    DataName.IMU_ANG_VEL: "IMUAngularVelocity",
    DataName.FRAME_VELOCITIES: "Velocities",
    DataName.ON_EVENTS: "OnEvents",
    DataName.OFF_EVENTS: "OffEvents",
    DataName.DEPTH_MAP: "Depths",
    DataName.MOTION_FLOW: "MotionFlow",
}


class H5Dataset(Dataset):
    def __init__(self, dataset_path: str, keys: List[str]):
        self.file = None
        self.keys = keys
        self.transform = None
        self.dataset_path = dataset_path

        with h5py.File(dataset_path) as f:
            self.length = f[name_map[DataName.RENDER_TIME]].shape[0]


    def __len__(self):
        return self.length
    

    def __getitem__(self, idx):
        if self.file is None:
            self.file = h5py.File(self.dataset_path)
        
        ret: List[torch.Tensor] = []
        for key in self.keys:
            # Always include the on and off events together
            ret.append(torch.stack([
                    torch.Tensor(self.file[name_map[DataName.ON_EVENTS]][idx]),
                    torch.Tensor(self.file[name_map[DataName.OFF_EVENTS]][idx]),
                    ], 
            dim = 0))
            ret.append(torch.Tensor(self.file[key][idx]).unsqueeze(dim=0))

        return ret


def assert_wellformed(f: h5py.File) -> bool:
    assert all((f[name_map[key]].shape[0] == f[name_map[DataName.RENDER_TIME]].shape[0]) for key in name_map.keys())
        

def train(dataset_path: str, net: nn.Module, epochs: int = 1):
    device = torch.device("cuda")
    net.to(device)
    data_loader = DataLoader(H5Dataset(dataset_path, ["Depths"]), batch_size=1, num_workers=4, shuffle=False)
    optimizer = optim.Adam(net.parameters(), lr=5e-4)
    utils.reset(net)

    for epoch in range(epochs):
        train_batch: List[torch.Tensor] = iter(data_loader)
        for events, targets in train_batch:
            events = events.to(device)
            targets = targets.to(device)
            net.train()
            result = net(events)
            loss = calculate_depth_loss(result, targets)
            print(loss)
            # actually do the step
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()


input_file_paths: List[str] = []
# assume correct input
for dataset in argv[1:]:
    path = f'../datasets/{dataset}.hdf5'
    f = h5py.File(path)
    print(f"Loaded {dataset}.")
    assert_wellformed(f)
    f.close()
    input_file_paths.append(path)


# initialise models
depthNet = DepthMapNet()


for path in input_file_paths:
    train(path, depthNet)

