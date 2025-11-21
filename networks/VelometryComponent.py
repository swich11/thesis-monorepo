# Velometry Component parent class.
from abc import ABC


import torch
import torch.nn as nn
import snntorch as snn
from snntorch import surrogate
from utils import crop


from typing import List


class EncoderLayer(nn.Module):
    def __init__(self, input_channels: int, output_channels: int):
        super().__init__()
        spike_grad = surrogate.fast_sigmoid()
        self.conv1 = nn.Conv2d(input_channels, output_channels, 3)
        self.leaky1 = snn.Leaky(0.5, learn_beta=True, learn_threshold=True, spike_grad=spike_grad, init_hidden=False)
        self.conv2 = nn.Conv2d(output_channels, output_channels, 3)
        self.leaky2 = snn.Leaky(0.5, learn_beta=True, learn_threshold=True, spike_grad=spike_grad, init_hidden=False)


    def forward(self, x, mem1, mem2):
        x = self.conv1(x)
        x, mem1 = self.leaky1(x, mem1)
        y = self.conv2(x)
        x, mem2 = self.leaky2(y, mem2)
        return x, mem1, mem2
    

class ResidualLayer(nn.Module):
    def __init__(self, input_channels: int):
        super().__init__()
        self.conv1 = nn.Conv2d(input_channels, input_channels, 3)
        self.relu1 = nn.LeakyReLU()
        self.conv2 = nn.Conv2d(input_channels, input_channels, 3)
        self.relu2 = nn.LeakyReLU()


    def forward(self, x):
        """
            Altered forward from the Encoder Layer where the residuals are passed through.
        """
        r1 = x
        x = self.conv1(x)
        x = self.relu1(x)
        r2 = x
        x = self.conv2(x + crop(r1, x))
        x = self.relu2(x)
        x = x + crop(r2, x)
        return x


class VelometryComponent(nn.Module, ABC):
    layers: nn.ModuleList | None = None

    def __init__(self):
        super().__init__()
        # Ensure there is only a single instance of the
        # encoder layers at all times
        self.residual_layer = None
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

        for _ in range(self.depth - 1):
            self.input_channels = self.output_channels
            self.output_channels = self.output_channels * 2
            self.encoder_layers.append(EncoderLayer(self.input_channels, self.output_channels))

        # add the residual block (helps the deep network learn)
        self.input_channels = self.output_channels
        self.output_channels = self.output_channels * 2
        self.residual_layer = ResidualLayer(self.input_channels)

        # add the upconvolution to the residual layer
        # self.encoder_layers.append(nn.ConvTranspose2d(self.output_channels, self.output_channels // 2, 2, 2))

        self.encoder_layers = nn.ModuleList(self.encoder_layers) # put layers on nn stack


    def init_mems(self, x: torch.Tensor, device: torch.device) -> List[List[torch.Tensor]]:
        mems: List[torch.Tensor] = []
        output_channels = 64
        batch_size = x.shape[0]
        h = x.shape[2]
        w = x.shape[3]

        for _ in range(self.depth):
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

    
