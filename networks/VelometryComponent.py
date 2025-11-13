# Velometry Component parent class.
from abc import ABC


import torch
import torch.nn as nn

import snntorch as snn
from snntorch import surrogate

from typing import List, Tuple


class VelometryComponent(nn.Module, ABC):
    encoder_layers = None

    def __init__(self):
        super().__init__()
        if (VelometryComponent.encoder_layers):
            self._initialise_encoder_layers()
    

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
            input_channels = self.output_channels
            output_channels = self.output_channels * 2
            self.encoder_layers.append(nn.Sequential(
                nn.MaxPool2d(2, 2),
                # Add SNN layer here?
                nn.Conv2d(input_channels, output_channels, 3),
                snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=True),
                nn.Conv2d(output_channels, output_channels, 3),
                snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=True),
            ))
        # add the upconvolution to the last encoder layer
        self.encoder_layers[-1].append(nn.ConvTranspose2d(self.output_channels, self.output_channels // 2, 2, 2))


    def _crop(self, x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        wd_x = (x.shape[2] - y.shape[2])
        wd_y = (x.shape[3] - y.shape[3])
        wd_xi = wd_x // 2
        wd_yi = wd_y // 2
        wd_xj = wd_xi + 1 if (wd_x % 2) else wd_xi
        wd_yj = wd_yi + 1 if (wd_y % 2) else wd_yi
        return x[:, :, wd_xi:-wd_xj, wd_yi:-wd_yj]
