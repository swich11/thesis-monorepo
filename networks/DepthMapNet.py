# Hybrid SNN-DNN network for depth map

import torch
import torch.nn as nn
import torchvision.transforms.functional as F

import snntorch as snn
from snntorch import surrogate

from typing import List


from VelometryComponent import VelometryComponent


class DepthMapNet(VelometryComponent):
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


    def backward(self, x_res: torch.Tensor, x_real: torch.Tensor) -> None:
        x_real = self._crop(x_real, x_res)
        assert(x_res.shape == x_real.shape)

        # apply gaussian smoothing on the result to ensure it doesn't overfit
        # we are not trying to get a 1 to 1 depth map here
        # just a very good, averaged approximation
        d_res_blur = F.gaussian_blur(x_res, [5, 5])
        loss: torch.Tensor = torch.linalg.norm((d_res_blur - x_real), 2, dim=(2, 3))
        # this doesn't really work for these purposes. We don't need a spicy depth map
        loss.backward()



# Testing
if __name__ == "__main__":
    net = DepthMapNet()
    x = torch.randn(1, 2, 346, 260)
    y: torch.Tensor = net(x)
    net.backward(y, torch.randn(1, 1, 346, 260))

