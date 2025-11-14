# Hybrid SNN-DNN network for Velocity. Uses Optical Flow and Depth Map as intermediaries.


import torch
import torch.nn as nn


from typing import Tuple
from VelometryComponent import VelometryComponent
from DepthMapVelocityNet import DepthMapVelocityNet
from OpticalFlowVelocityNet import OpticalFlowVelocityNet
from utils import crop


class VelocityNet(VelometryComponent):
    def __init__(self):
        super().__init__()
        
        self.depth_net = DepthMapVelocityNet()
        self.flow_net = OpticalFlowVelocityNet()
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
        

    def _forward_recurse_unet(self, x: torch.Tensor, v: torch.Tensor, n: int) -> Tuple[torch.Tensor, torch.Tensor]:
        # Pass through encoder layer -> output accumulator -> optic decoder layers
        #                                                  -> depth decoder layers
        # depths ->
        # flow   -> velocities
        x = self.encoder_layers[n](x)
        if n == self.depth:
            return (x, x)
        d, o = self._forward_recurse_unet(x, v, n + 1)    
        # Crop the skip connection, assumes d and o have same shape
        x = crop(x, d)
        v = v.unsqueeze(-1).unsqueeze(-1) # add 2 spatial dimensions
        v = v.expand(-1, -1, d.shape[2], d.shape[3])

        d = self.depth_net.decoder_layers[n](torch.cat([x, d, v], dim=1))
        o = self.flow_net.decoder_layers[n](torch.cat([x, o, v], dim=1))
        return (d, o)


    def _forward_velocity_step(self, d: torch.Tensor, o: torch.Tensor) -> torch.Tensor:
        return self.velocity_layers(torch.cat([d, o], dim=1))


# Testing works
if __name__ == "__main__":
    net = VelocityNet()
    x = torch.randn(1, 2, 346, 260)
    v = torch.randn(1, 6)
    v_out: torch.Tensor
    d: torch.Tensor
    o: torch.Tensor
    v_out, d, o = net(x, v)
    print(v_out.shape)
    print(d.shape)
    print(o.shape)
    v_out.mean().backward()