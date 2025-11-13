# Hybrid SNN-DNN network for Optical Flow with last velocity input on the decoder
# layers


# Encoder layers -> Decoder layers with velocity input on each layer


import torch
import torch.nn as nn

import snntorch as snn
from snntorch import surrogate

from typing import List


class OpticalFlowVelocityNet(nn.Module):
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


    def forward(self, x, v):
        return self._forward_recurse(x, v, 0)
        

    def _forward_recurse(self, x: torch.Tensor, v: torch.Tensor, n: int) -> torch.Tensor:
        # Pass through encoder layer, -> output accumulator -> decoder layer
        #                             -> next layer
        x = self.encoder_layers[n](x)
        if n == self.depth:
            return x
        y = self._forward_recurse(x, v, n + 1)
        # Crop the skip connection
        wd_x = (x.shape[2] - y.shape[2])
        wd_y = (x.shape[3] - y.shape[3])
        wd_xi = wd_x // 2
        wd_yi = wd_y // 2
        wd_xj = wd_xi + 1 if (wd_x % 2) else wd_xi
        wd_yj = wd_yi + 1 if (wd_y % 2) else wd_yi

        x = torch.cat([x[:, :, wd_xi:-wd_xj, wd_yi:-wd_yj], y], dim=1)

        v = v.unsqueeze(-1).unsqueeze(-1) # add 2 spatial dimensions
        v = v.expand(-1, -1, x.shape[2], x.shape[3]) # match height and width
        
        # concatenate on input channels, combining image and velocity
        x = self.decoder_layers[n](torch.cat([x, v], dim=1))
        return x


# Testing works
if __name__ == "__main__":
    net = OpticalFlowVelocityNet(4)
    x = torch.randn(1, 2, 346, 260)
    v = torch.randn(1, 6)
    y: torch.Tensor = net(x, v)
    print(y.shape)
    y.mean().backward()