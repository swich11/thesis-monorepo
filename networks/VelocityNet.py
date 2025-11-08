# Hybrid SNN-DNN network for Velocity. Uses Optical Flow and Depth Map as intermediaries.


import torch
import torch.nn as nn

import snntorch as snn
from snntorch import surrogate

from typing import List, Tuple


class VelocityNet(nn.Module):
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

        input_channels_save = output_channels
        self.optical_decoder_layers: List[nn.Sequential] = []
        for _ in range(depth - 1):
            input_channels = output_channels
            output_channels = input_channels // 2
            self.optical_decoder_layers.append(nn.Sequential(
                nn.Conv2d(input_channels + 6, output_channels, 3), # add 6 for velocity input channels
                nn.ReLU(),
                nn.Conv2d(output_channels, output_channels, 3),
                nn.ReLU(),
                nn.ConvTranspose2d(output_channels, output_channels // 2, 2, 2)
            ))
        # Last layer set for optical flow output
        input_channels = output_channels
        output_channels = input_channels // 2
        self.optical_decoder_layers.append(nn.Sequential(
            nn.Conv2d(input_channels + 6, output_channels, 3), # add 6 for velocity input channels
            nn.ReLU(),
            nn.Conv2d(output_channels, output_channels, 3),
            nn.ReLU(),
            nn.Conv2d(output_channels, 2, 1) # 2 output channels for optical flow
        ))
        self.optical_decoder_layers.reverse()


        self.depth_decoder_layers: List[nn.Sequential] = []
        output_channels = input_channels_save
        for _ in range(depth - 1):
            input_channels = output_channels
            output_channels = input_channels // 2
            self.depth_decoder_layers.append(nn.Sequential(
                nn.Conv2d(input_channels + 6, output_channels, 3), # add 6 for velocity input channels
                nn.ReLU(),
                nn.Conv2d(output_channels, output_channels, 3),
                nn.ReLU(),
                nn.ConvTranspose2d(output_channels, output_channels // 2, 2, 2)
            ))
        # Last layer set for depth map output
        input_channels = output_channels
        output_channels = input_channels // 2
        self.depth_decoder_layers.append(nn.Sequential(
            nn.Conv2d(input_channels + 6, output_channels, 3), # add 6 for velocity input channels
            nn.ReLU(),
            nn.Conv2d(output_channels, output_channels, 3),
            nn.ReLU(),
            nn.Conv2d(output_channels, 1, 1) # 1 output channel for depth
        ))
        self.depth_decoder_layers.reverse()
        
        # Velocity Layers
        self.velocity_layers = nn.Sequential(
            nn.Conv2d(3, 16, 5),
            nn.ReLU(),
            nn.Conv2d(16, 32, 5),
            nn.ReLU(),
            nn.Conv2d(32, 6, 1),
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten()
        )






    def forward(self, x, v):
        (d, o) = self._forward_recurse_unet(x, v, 0)
        v_out = self._forward_velocity_step(d, o)
        return (v_out, d, o)
        

    def _forward_recurse_unet(self, x: torch.Tensor, v: torch.Tensor, n: int) -> Tuple[torch.Tensor]:
        # Pass through encoder layer -> output accumulator -> optic decoder layers
        #                                                  -> depth decoder layers
        # depths ->
        # flow   -> velocities
        x = self.encoder_layers[n](x)
        if n == self.depth:
            return (x, x)

        d, o = self._forward_recurse_unet(self, x, v, n)       
        # crop the skip connection 
        wd = (x.shape[3] - x.shape[3]) // 2
        x = x[:, :, wd:-wd, wd:-wd]
        # infer on the decoder layer
        d = self.depth_decoder_layers[n](torch.cat([x[:, :, wd:-wd, wd:-wd], d, v], dim=1))
        o = self.optical_decoder_layers[n](torch.cat([x[:, :, wd:-wd, wd:-wd], o, v], dim=1))
        return (d, o)


    def _forward_velocity_step(self, d, o) -> torch.Tensor:
        return self.velocity_layers(torch.cat([d, o], dim=1))


# Testing works
if __name__ == "__main__":
    net = VelocityNet(4)
    x = torch.randn(1, 2, 572, 572)
    y: torch.Tensor = net(x)
    y.mean().backward()