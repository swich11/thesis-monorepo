# Velometry Component parent class.
from abc import ABC, abstractmethod


import torch
import torch.nn as nn
import snntorch as snn
from snntorch import surrogate


from typing import List


class VelometryComponent(nn.Module, ABC):
    encoder_layers: List[nn.Sequential] = []

    def __init__(self):
        super().__init__()
        # Ensure there is only a single instance of the
        # encoder layers at all times
        if (VelometryComponent.encoder_layers == []):
            self._initialise_encoder_layers()
        self.decoder_layers: List[nn.Sequential] = []
    

    def _initialise_encoder_layers(self):
        """
            Initialises the encoder layers for the network. In most cases
            these should be shared for depth map and optical flow inference
        """
        self.depth = 4
        self.input_channels = 2
        self.output_channels = 64
        spike_grad = surrogate.fast_sigmoid()
        self.encoder_layers = [nn.Sequential(
            nn.Conv2d(self.input_channels, self.output_channels, 3),
            snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=True),
            nn.Conv2d(self.output_channels, self.output_channels, 3),
            snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=True)
        )]
        for _ in range(self.depth):
            self.input_channels = self.output_channels
            self.output_channels = self.output_channels * 2
            self.encoder_layers.append(nn.Sequential(
                nn.MaxPool2d(2, 2),
                # Add SNN layer here?
                nn.Conv2d(self.input_channels, self.output_channels, 3),
                snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=True),
                nn.Conv2d(self.output_channels, self.output_channels, 3),
                snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=True),
            ))
        # add the upconvolution to the last encoder layer
        self.encoder_layers[-1].append(nn.ConvTranspose2d(self.output_channels, self.output_channels // 2, 2, 2))
    
