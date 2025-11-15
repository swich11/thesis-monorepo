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
        self.leaky1 = snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=False)
        self.conv2 = nn.Conv2d(output_channels, output_channels, 3)
        self.leaky2 = snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=False)

        # hold internal memory state
        device = torch.device("cuda")
        self.mem1 = self.leaky1.reset_mem().to(device)
        self.mem2 = self.leaky2.reset_mem().to(device)


    def forward(self, x):
        x = self.conv1(x)
        self.mem1 = self.mem1.detach()
        x, self.mem1 = self.leaky1(x, self.mem1)
        x = self.conv2(x)
        self.mem2 = self.mem2.detach()
        _, self.mem2 = self.leaky2(x, self.mem2)
        return x


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
        
    

    def _initialise_encoder_layers(self):
        """
            Initialises the encoder layers for the network. In most cases
            these should be shared for depth map and optical flow inference
        """
        self.depth = 4
        self.input_channels = 2
        self.output_channels = 64

        self.encoder_layers = [nn.Sequential(
            EncoderLayer(self.input_channels, self.output_channels)
        )]

        for _ in range(self.depth):
            self.input_channels = self.output_channels
            self.output_channels = self.output_channels * 2
            self.encoder_layers.append(nn.Sequential(
                nn.MaxPool2d(2, 2),
                EncoderLayer(self.input_channels, self.output_channels)
            ))
        # add the upconvolution to the last encoder layer
        self.encoder_layers[-1].append(nn.ConvTranspose2d(self.output_channels, self.output_channels // 2, 2, 2))
        self.encoder_layers = nn.ModuleList(self.encoder_layers)
    
