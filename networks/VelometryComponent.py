# Velometry Component parent class.
from abc import ABC


import torch
import torch.nn as nn
import snntorch as snn
from snntorch import surrogate


from typing import List


class EncoderLayer(nn.Module):
    def __init__(self, input_channels: int, output_channels: int):
        super().__init__()
        spike_grad = surrogate.fast_sigmoid()
        self.conv1 = nn.Conv2d(input_channels, output_channels, 3)
        self.leaky1 = snn.Leaky(0.85, spike_grad=spike_grad, init_hidden=False)
        self.conv2 = nn.Conv2d(output_channels, output_channels, 3)
        self.leaky2 = snn.Leaky(0.85, spike_grad=spike_grad, init_hidden=False)


    def forward(self, x, mem1, mem2):
        x = self.conv1(x)
        x, mem1 = self.leaky1(x, mem1)
        y = self.conv2(x)
        x, mem2 = self.leaky2(y, mem2)
        return x, y, mem1, mem2


class VelometryComponent(nn.Module, ABC):
    layers: nn.ModuleList | None = None

    def __init__(self):
        super().__init__()
        # Ensure there is only a single instance of the
        # encoder layers at all times
        if (VelometryComponent.layers is None):
            self._initialise_encoder_layers()
            VelometryComponent.layers = self.encoder_layers
        else:
            self.encoder_layers = VelometryComponent.layers
        self.decoder_layers = []
        self.output_channels = 1024
        self.input_channels = 512
        self.depth = 4
        self.max_pool = nn.MaxPool2d(2, 2)
        
    

    def _initialise_encoder_layers(self):
        """
            Initialises the encoder layers for the network. In most cases
            these should be shared for depth map and optical flow inference
        """
        self.depth = 4
        self.input_channels = 2
        self.output_channels = 64

        self.encoder_layers = [EncoderLayer(self.input_channels, self.output_channels)]

        for _ in range(self.depth):
            self.input_channels = self.output_channels
            self.output_channels = self.output_channels * 2
            self.encoder_layers.append(EncoderLayer(self.input_channels, self.output_channels))

        # add the upconvolution to the last encoder layer
        self.encoder_layers.append(nn.ConvTranspose2d(self.output_channels, self.output_channels // 2, 2, 2))
        self.encoder_layers = nn.ModuleList(self.encoder_layers)


    def init_mems(self, x: torch.Tensor, device: torch.device) -> List[List[torch.Tensor]]:
        mems: List[torch.Tensor] = []
        output_channels = 64
        batch_size = x.shape[0]
        h = x.shape[2]
        w = x.shape[3]

        for _ in range(self.depth + 1):
            # conv layer
            h -= 2
            w -= 2
            mems.append(torch.zeros(1, output_channels, h, w, device=device))
            # conv layer
            h -= 2
            w -= 2 
            mems.append(torch.zeros(1, output_channels, h, w, device=device))
            # max pooling
            h = int(h / 2)
            w = int(w / 2)

        mem_batches = [[t.clone() for t in mems] for _ in range(batch_size)]     
        return mem_batches

    
