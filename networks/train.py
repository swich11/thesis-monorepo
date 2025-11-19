from sys import argv


import h5py
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import matplotlib.pyplot as plt
import random
from torch.utils.data import DataLoader, Dataset
from snntorch import utils
import torchvision.transforms.functional as IF


from DepthMapNet import DepthMapNet
from DepthMapVelocityNet import DepthMapVelocityNet
from OpticalFlowNet import OpticalFlowNet
from OpticalFlowVelocityNet import OpticalFlowVelocityNet
from VelocityNet import VelocityNet
from VelometryComponent import VelometryComponent
from utils import calculate_depth_loss, calculate_flow_loss, calculate_velocity_loss, calculate_AAE, calculate_AAE_simple


from enum import Enum
from typing import List, Callable, Tuple


# additional types
LossFunction = Callable[[torch.Tensor, torch.Tensor], torch.Tensor]


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
    GRAYSCALE = 9,

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
    DataName.GRAYSCALE: "GrayscaleImage",
}


# Data map allows training code to be reused for each net
data_map = {
    DepthMapNet: ([DataName.DEPTH_MAP], calculate_depth_loss),
    DepthMapVelocityNet: ([DataName.FRAME_VELOCITIES, DataName.DEPTH_MAP], calculate_depth_loss),
    OpticalFlowNet: ([DataName.MOTION_FLOW], calculate_flow_loss),
    OpticalFlowVelocityNet: ([DataName.FRAME_VELOCITIES, DataName.MOTION_FLOW], calculate_flow_loss),
    VelocityNet: ([DataName.FRAME_VELOCITIES], calculate_velocity_loss),
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
        ret.append(torch.stack([
                torch.Tensor(self.file[name_map[DataName.ON_EVENTS]][idx]),
                torch.Tensor(self.file[name_map[DataName.OFF_EVENTS]][idx]),
                ], 
        dim = 0))
        for key in self.keys:
            t = torch.Tensor(self.file[key][idx])
            if (key == name_map[DataName.MOTION_FLOW]):
                # motion flow outlier has 4 channels (only need the first 2) (x, y)
                ret.append(t.permute(2, 0, 1)[:2])
            else:
                ret.append(t.unsqueeze(dim=0))
        return ret


class BalancedDataLoader:
    def __init__(self, datasets: List[Dataset], batch_size: int, num_workers=4, drop_last=True):
        self.batch_size = batch_size
        self.data_loaders = [DataLoader(d, batch_size=batch_size, num_workers=num_workers, shuffle=True, drop_last=drop_last) for d in datasets]


    def __len__(self):
        return sum(len(loader) for loader in self.data_loaders)


    def __iter__(self):
        self.data_iterators = []
        self.iterator_lengths = []
        self.length = 0
        for loader in self.data_loaders:
            length = len(loader)
            self.data_iterators.append(iter(loader))
            self.iterator_lengths.append(length)
            self.length += length
        return self


    def __next__(self):
        if self.length == 0:
            raise StopIteration
        idx = random.choices(range(len(self.data_iterators)), self.iterator_lengths, k=1)[0] # get random weighted index
        # increment iterator lengths
        self.iterator_lengths[idx] -= 1
        self.length -= 1
        return next(self.data_iterators[idx])
    



def assert_wellformed(f: h5py.File) -> bool:
    assert all((f[name_map[key]].shape[0] == f[name_map[DataName.RENDER_TIME]].shape[0]) for key in name_map.keys())
        

def train_from_image(datasets: Dataset, net: VelometryComponent, loss_fn: LossFunction, batch_size: int = 16, epochs: int = 5) -> Tuple[List[float], List[float]]:
    device = torch.device("cuda")
    net = net.to(device)
    data_loader = BalancedDataLoader(datasets, batch_size=batch_size, num_workers=4, drop_last=True)
    optimizer = optim.Adam(net.parameters(), lr=5e-4)


    losses: List[float] = []
    AAE: List[float] = []
    for epoch in range(epochs):
        print(f"Training Epoch {epoch}.")
        epoch_loss = 0.0
        for events, targets, images in data_loader:
            images: torch.Tensor = images.to(device)
            images = images.squeeze(dim=1)
            images = torch.stack((images, images), dim=1) / torch.max(images) # normalise to an event input
            targets: torch.Tensor = targets.to(device)
            net.train()
            # back propogate through time for the batch
            utils.reset(net) # reset SNN memories at the start of the batch
            mem_batches = net.init_mems(events, device)
            batch_loss = 0.0
            for i in range(1, events.shape[0]):
                result, mem_batches[i] = net(images[i-1].unsqueeze(0), mem_batches[i - 1]) # pass one set of spikes in at a time
                # use the same flow loss as before (it should work)
                loss = loss_fn(result, targets[i].unsqueeze(0), images[i-1].unsqueeze(0), images[i].unsqueeze(0))
                AAE.append(float(calculate_AAE_simple(result, targets[i].unsqueeze(0)).detach()))
                batch_loss += loss
            # do backwards pass per batch loss
            optimizer.zero_grad()
            torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0)
            batch_loss.backward()
            optimizer.step()
            batch_loss = batch_loss / events.shape[0]
            losses.append(float(batch_loss.detach()))
            epoch_loss += float(batch_loss.detach())
        # print the weights per epoch
        for name, p in net.named_parameters():
            if p.grad is not None:
                print(name, p.grad.norm())
        print(f"Loss: {epoch_loss}")
        torch.save({
            "model": net.state_dict(),
            "optimizer": optimizer.state_dict(),
            "epoch": epoch,
            "loss": epoch_loss,
        }, f"models/checkpoint{epoch}.pth")

    return losses, AAE


def train(datasets: Dataset, net: VelometryComponent, loss_fn: LossFunction, batch_size: int  = 16, epochs: int = 5) -> Tuple[List[float], List[float]]:
    device = torch.device("cuda")
    data_loader = BalancedDataLoader(datasets, batch_size=batch_size, num_workers=4, drop_last=True)
    optimizer = optim.Adam(net.parameters(), lr=1e-3)


    losses: List[float] = []
    AAE: List[float] = []
    for epoch in range(epochs):
        print(f"Training Epoch {epoch}.")
        epoch_loss = 0.0
        for events, targets in data_loader:
            events = events.to(device)
            targets = targets.to(device)
            events = IF.gaussian_blur(events, [5, 5]) # get the damn network moving
            net.train()
            # back propogate through time for the batch
            utils.reset(net) # reset SNN memories at the start of the batch
            mem_batches = net.init_mems(events, device)
            batch_loss = 0.0
            for i in range(1, events.shape[0]):
                result, mem_batches[i] = net(events[i].unsqueeze(0), mem_batches[i-1]) # pass one set of spikes in at a time
                loss = loss_fn(result, targets[i].unsqueeze(0), events[i-1].unsqueeze(0), events[i].unsqueeze(0))
                AAE.append(float(calculate_AAE(result, targets[i].unsqueeze(0), events[i-1].unsqueeze(0), events[i].unsqueeze(0)).detach()))
                batch_loss = batch_loss + loss
            # do backwards pass per batch loss
            optimizer.zero_grad()
            batch_loss.backward()
            torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0)
            optimizer.step()
            batch_loss = batch_loss / events.shape[0]
            losses.append(float(batch_loss.detach()))
            epoch_loss += float(batch_loss.detach())
        for name, p in net.named_parameters():
            if p.grad is not None:
                print(name, p.grad.norm())
        print(epoch_loss)
    return losses, AAE


def train_velocity(datasets: Dataset, net: VelometryComponent, loss_fn: LossFunction, batch_size: int  = 16, epochs: int = 1) -> Tuple[List[float], List[float]]:
    device = torch.device("cuda")
    data_loader = BalancedDataLoader(datasets, batch_size=batch_size, num_workers=4, drop_last=True)
    optimizer = optim.Adam(net.parameters(), lr=1e-4)


    losses: List[float] = []
    AAE: List[float] = []
    for epoch in range(epochs):
        print(f"Training Epoch {epoch}.")
        for events, velocities, targets in data_loader:
            events = events.to(device)
            targets = targets.to(device)
            velocities = velocities.to(device)
            net.train()
            # back propogate through time for the batch
            utils.reset(net) # reset SNN memories at the start of the batch
            mem_batches = net.init_mems(events, device)
            loss = 0.0
            for i in range(1, events.shape[0]):
                result, mem_batches[i] = net(events[i].unsqueeze(0), velocities[i-1], mem_batches[i-1]) # pass one set of spikes in at a time, and previous velocity estimate
                loss += loss_fn(result, targets[i].unsqueeze(0), events[i-1].unsqueeze(0), events[i].unsqueeze(0))
                AAE.append(float(calculate_AAE(result, targets[i].unsqueeze(0), events[i-1].unsqueeze(0), events[i].unsqueeze(0)).detach()))
            # do backwards pass per batch loss
            optimizer.zero_grad()
            loss.backward()
            total_norm = 0
            for name, p in net.named_parameters():
                if p.grad is not None:
                    print(f"{name}, {p.grad.norm()}")
            # torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0)
            optimizer.step()
            loss = loss / events.shape[0]
            losses.append(float(loss.detach()))
    return losses, AAE


def train_velometry_component(input_file_paths: List[str], net: VelometryComponent, epochs: int  = 1) -> Tuple[List[float], List[float]]:
    data_names = [name_map[dataName] for dataName in data_map[type(net)][0]] + [name_map[DataName.GRAYSCALE]]
    datasets: List[Dataset] = [H5Dataset(path, data_names) for path in input_file_paths]
    if len(data_map[type(net)][0]) > 1:
        print("velocity training")
        losses, AAE = train_velocity(datasets, net, data_map[type(net)][1], epochs=epochs)
    else:
        losses, AAE = train_from_image(datasets, net, data_map[type(net)][1], epochs=epochs)
    return losses, AAE


if __name__ == "__main__":
    input_file_paths: List[str] = []
    # assume correct input
    for dataset in argv[1:]:
        path = f'../datasets/{dataset}.hdf5'
        f = h5py.File(path)
        print(f"Loaded {dataset}.")
        assert_wellformed(f)
        f.close()
        input_file_paths.append(path)

    loss, AAE = train_velometry_component(input_file_paths, OpticalFlowNet(), epochs=5)
    
    # loss_norm = train_velometry_component(input_file_paths, OpticalFlowNet())

    plt.plot(loss, label='velocity net', color='orange')
    # plt.plot(loss_norm, label='normal net', color='blue')
    plt.show()

    plt.plot(AAE, color='orange')
    # plt.plot(aae_norm, color='blue')
    plt.show()

