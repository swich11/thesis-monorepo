# Hybrid SNN-DNN network for optical flow

import torch
import torch.nn as nn


from typing import List
from ComponentInterface import ComponentInterface
from utils import calculate_flow_loss
from OpticalFlowVelocityNet import OpticalFlowVelocityNet


class OpticalFlowNet(ComponentInterface):
    def __init__(self):
        super().__init__()
        
        # initialise the optical flow component
        output_channels = self.output_channels
        self.decoder_layers: List[nn.Sequential] = []
        for _ in range(self.depth - 1):
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
            nn.Conv2d(output_channels, 2, 1) # 2 output channels for optical flow
        ))
        self.decoder_layers.reverse()
        self.decoder_layers = nn.ModuleList(self.decoder_layers)


# Testing works
if __name__ == "__main__":
    device = torch.device("cuda")
    net = OpticalFlowNet().to(device)
    x = torch.randn(1, 2, 346, 260).to(device)
    mems = net.init_mems(x, device)
    y: torch.Tensor
    y, mems = net(x, mems[0])
    torch.mean(y).backward()
    # calculate_flow_loss(y, torch.randn(1, 2, 346, 260)).backward()
