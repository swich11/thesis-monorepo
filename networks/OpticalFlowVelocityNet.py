# Hybrid SNN-DNN network for Optical Flow with last velocity input on the decoder
# layers


# Encoder layers -> Decoder layers with velocity input on each layer


import torch
import torch.nn as nn


from typing import List
from VelocityComponentInterface import VelocityComponentInterface
from utils import calculate_flow_loss


class OpticalFlowVelocityNet(VelocityComponentInterface):
    def __init__(self):
        super().__init__()

        output_channels = self.output_channels
        self.decoder_layers: List[nn.Sequential] = []
        for _ in range(self.depth - 1):
            input_channels = output_channels
            output_channels = input_channels // 2
            self.decoder_layers.append(nn.Sequential(
                nn.Conv2d(input_channels + 6, output_channels, 3), # conv3d to add velocity channels, add 6 for the channels
                nn.ReLU(),
                nn.Conv2d(output_channels, output_channels, 3),
                nn.ReLU(),
                nn.ConvTranspose2d(output_channels, output_channels // 2, 2, 2)
            ))
        # Last layer set for optical flow output
        input_channels = output_channels
        output_channels = input_channels // 2
        self.decoder_layers.append(nn.Sequential(
            nn.Conv2d(input_channels + 6, output_channels, 3), # conv3d to add velocity channels
            nn.ReLU(),
            nn.Conv2d(output_channels, output_channels, 3),
            nn.ReLU(),
            nn.Conv2d(output_channels, 2, 1) # 2 output channels for optical flow
        ))
        self.decoder_layers.reverse()


# Testing works
if __name__ == "__main__":
    net = OpticalFlowVelocityNet()
    x = torch.randn(1, 2, 346, 260)
    v = torch.randn(1, 6)
    y: torch.Tensor = net(x, v)
    calculate_flow_loss(y, torch.randn(1, 2, 346, 260)).backward()