# Hybrid SNN-DNN network for depth map

import torch
import torch.nn as nn

import snntorch as snn
from snntorch import surrogate

from typing import List


class DepthMapNet(nn.Module):
    def __init__(self, depth: int = 4):
        super().__init__()
        self.depth = depth
        input_channels = 2
        output_channels = 64
        spike_grad = surrogate.fast_sigmoid()
        
        self.encoder_layers = [nn.Sequential(
            nn.Conv2d(input_channels, output_channels, 3),
            snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=True),
            nn.Conv2d(output_channels, output_channels, 3),
            snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=True)
        )]
        for _ in range(depth):
            input_channels = output_channels
            output_channels = output_channels * 2
            self.encoder_layers.append(nn.Sequential(
                nn.MaxPool2d(2, 2),
                # Add SNN layer here?
                nn.Conv2d(input_channels, output_channels, 3),
                snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=True),
                nn.Conv2d(output_channels, output_channels, 3),
                snn.Leaky(0.5, spike_grad=spike_grad, learn_beta=True, init_hidden=True),
            ))
        # add the upconvolution to the last encoder layer
        self.encoder_layers[-1].append(nn.ConvTranspose2d(output_channels, output_channels // 2, 2, 2))

        self.decoder_layers: List[nn.Sequential] = []
        for _ in range(depth - 1):
            input_channels = output_channels
            output_channels = input_channels // 2
            self.decoder_layers.append(nn.Sequential(
                nn.Conv2d(input_channels, output_channels, 3),
                nn.ReLU(),
                nn.Conv2d(output_channels, output_channels, 3),
                nn.ReLU(),
                nn.ConvTranspose2d(output_channels, output_channels // 2, 2, 2)
            ))
        # Last layer set for optical flow output
        input_channels = output_channels
        output_channels = input_channels // 2
        self.decoder_layers.append(nn.Sequential(
            nn.Conv2d(input_channels, output_channels, 3),
            nn.ReLU(),
            nn.Conv2d(output_channels, output_channels, 3),
            nn.ReLU(),
            nn.Conv2d(output_channels, 1, 1) # 1 output channel for depth map
        ))
        self.decoder_layers.reverse()


    def forward(self, x):
        return self._forward_recurse(x, 0)
        

    def _forward_recurse(self, x: torch.Tensor, n: int) -> torch.Tensor:
        # Pass through encoder layer, -> output accumulator -> decoder layer
        #                             -> next layer
        x = self.encoder_layers[n](x)
        if n == self.depth:
            return x
        y = self._forward_recurse(x, n + 1)
        wd = (x.shape[3] - y.shape[3]) // 2 # crop the skip connection
        x = self.decoder_layers[n](torch.cat([x[:, :, wd:-wd, wd:-wd], y], dim=1))
        return x


# Testing works
if __name__ == "__main__":
    net = DepthMapNet(4)
    x = torch.randn(1, 2, 572, 572)
    y: torch.Tensor = net(x)
    y.mean().backward()






