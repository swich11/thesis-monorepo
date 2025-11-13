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


    def forward(self, x):
        return self._forward_recurse(x, 0)


    def _forward_recurse(self, x: torch.Tensor, n: int) -> torch.Tensor:
        """
            Recursive call to U-Net structure.
        """
        # Pass through encoder layer, -> output accumulator -> decoder layer
        #                             -> next layer
        x = self.encoder_layers[n](x)
        if n == self.depth:
            return x
        y = self._forward_recurse(x, n + 1)
        # crop for the skip connection here
        x = self._crop(x, y)
        x = self.decoder_layers[n](torch.cat([x, y], dim=1))
        return x
        

    def _crop(self, x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        wd_x = (x.shape[2] - y.shape[2])
        wd_y = (x.shape[3] - y.shape[3])
        wd_xi = wd_x // 2
        wd_yi = wd_y // 2
        wd_xj = wd_xi + 1 if (wd_x % 2) else wd_xi
        wd_yj = wd_yi + 1 if (wd_y % 2) else wd_yi
        return x[:, :, wd_xi:-wd_xj, wd_yi:-wd_yj]
    

    @abstractmethod
    def backward(self, x_res: torch.Tensor, x_real: torch.Tensor) -> None:
        """
            Call this after the forward pass to perform the backwards pass
            to train the network.
        """
        pass
