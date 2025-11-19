# Hybrid SNN-DNN network for depth map

import torch
import torch.nn as nn
import torchvision.transforms.functional as F


from snntorch import utils


from typing import List
from ComponentInterface import ComponentInterface
from utils import calculate_depth_loss


class DepthMapNet(ComponentInterface):
    def __init__(self):
        super().__init__()

        # initialise the depth component
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
            nn.Conv2d(output_channels, 1, 1) # 1 output channel for depth map
        ))
        self.decoder_layers.reverse()
        self.decoder_layers = nn.ModuleList(self.decoder_layers)


# Testing
if __name__ == "__main__":
    with torch.autograd.detect_anomaly():
        net = DepthMapNet()
        x = torch.randn(1, 2, 346, 260)
        mems = net.init_mems(x, x.device)
        y, mems[0] = net(x, mems[0])
        print(y)
        torch.mean(y).backward()
        x = torch.randn(1, 2, 346, 260)
        y, mems[0] = net(x, mems[0])
        print(y)

