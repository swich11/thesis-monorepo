# Hybrid SNN-DNN network for depth map

import torch
import torch.nn as nn
import torchvision.transforms.functional as F

import snntorch as snn
from snntorch import surrogate

from typing import List


class DepthMapNet(nn.Module):
    def __init__(self, depth: int = 4):
        super().__init__()
        self.depth = depth

        # parameters for keeping track of crop amount (for calculating losses)


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
        """
            Recursive call to U-Net structure.
        """
        # Pass through encoder layer, -> output accumulator -> decoder layer
        #                             -> next layer
        x = self.encoder_layers[n](x)
        if n == self.depth:
            return x
        y = self._forward_recurse(x, n + 1)
        # crop the skip connection 
        wd_x = (x.shape[2] - y.shape[2])
        wd_y = (x.shape[3] - y.shape[3])
        wd_xi = wd_x // 2
        wd_yi = wd_y // 2
        wd_xj = wd_xi + 1 if (wd_x % 2) else wd_xi
        wd_yj = wd_yi + 1 if (wd_y % 2) else wd_yi

        x = self.decoder_layers[n](torch.cat([x[:, :, wd_xi:-wd_xj, wd_yi:-wd_yj], y], dim=1))
        return x
    

    def backward(self, d_res: torch.Tensor, d_real: torch.Tensor) -> None:
        """
            Call this after the forward pass to perform the backwards pass
            to train the network.
        """
        # crop code
        wd_x = (d_real.shape[2] - d_res.shape[2])
        wd_y = (d_real.shape[3] - d_res.shape[3])
        wd_xi = wd_x // 2
        wd_yi = wd_y // 2
        wd_xj = wd_xi + 1 if (wd_x % 2) else wd_xi
        wd_yj = wd_yi + 1 if (wd_y % 2) else wd_yi

        # crop d_real to size of d_res
        d_real = d_real[:, :, wd_xi:-wd_xj, wd_yi:-wd_yj]
        assert(d_res.shape == d_real.shape)

        # apply gaussian smoothing on the result to ensure it doesn't overfit
        # we are not trying to get a 1 to 1 depth map here
        # just a very good, averaged approximation
        d_res_blur = F.gaussian_blur(d_res, [5, 5])
        loss: torch.Tensor = torch.linalg.norm((d_res_blur - d_real), 2, dim=(2, 3))
        # this doesn't really work for these purposes. We don't need a spicy depth map
        loss.backward()



# Testing works
if __name__ == "__main__":
    net = DepthMapNet(4)
    x = torch.randn(1, 2, 346, 260)
    y: torch.Tensor = net(x)
    net.backward(y, torch.randn(1, 1, 346, 260))






