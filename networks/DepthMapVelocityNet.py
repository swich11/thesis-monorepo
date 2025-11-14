# Hybrid SNN-DNN network for depth map using latest velocity estimate
# as input to the network.
import torch
import torch.nn as nn


from typing import List
from VelocityComponentInterface import VelocityComponentInterface
from utils import calculate_depth_loss


class DepthMapVelocityNet(VelocityComponentInterface):
    def __init__(self):
        super().__init__()

        output_channels = self.output_channels
        self.decoder_layers: List[nn.Sequential] = []
        for _ in range(self.depth - 1):
            input_channels = output_channels
            output_channels = input_channels // 2
            self.decoder_layers.append(nn.Sequential(
                nn.Conv2d(input_channels + 6, output_channels, 3), # add 6 for velocity input channel
                nn.ReLU(),
                nn.Conv2d(output_channels, output_channels, 3),
                nn.ReLU(),
                nn.ConvTranspose2d(output_channels, output_channels // 2, 2, 2)
            ))
        # Last layer set for optical flow output
        input_channels = output_channels
        output_channels = input_channels // 2
        self.decoder_layers.append(nn.Sequential(
            nn.Conv2d(input_channels + 6, output_channels, 3), # add 6 for velocity input channel
            nn.ReLU(),
            nn.Conv2d(output_channels, output_channels, 3),
            nn.ReLU(),
            nn.Conv2d(output_channels, 1, 1) # 1 output channel for depth map
        ))
        self.decoder_layers.reverse()


# Testing works
if __name__ == "__main__":
    net = DepthMapVelocityNet()
    x = torch.randn(1, 2, 346, 260)
    v = torch.randn(1, 6)
    y: torch.Tensor = net(x, v)
    calculate_depth_loss(y, torch.randn(1, 1, 346, 260)).backward()

