# Hybrid SNN-DNN network for optical flow

import torch
import torch.nn as nn

import snntorch as snn
from snntorch import surrogate


class OpticalFlowNet(nn.Module):
    def __init__(self):
        super().__init__()
        depth = 4
        input_channels = 2
        output_channels = 64
        spike_grad = surrogate.FastSigmoid()
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
        self.encoder_layers[-1].append(nn.ConvTranspose2d(output_channels, output_channels // 2, 2))

        self.decoder_layers = []
        for _ in range(depth - 1):
            input_channels = output_channels
            output_channels = input_channels // 2
            self.decoder_layers.append(nn.Sequential(
                nn.Conv2d(input_channels, output_channels, 3),
                nn.ReLU(),
                nn.Conv2d(output_channels, output_channels, 3),
                nn.ReLU(),
                nn.ConvTranspose2d(output_channels, output_channels // 2, 2)
            ))
        # Last layer set for optical flow output
        input_channels = output_channels
        output_channels = input_channels // 2
        self.decoder_layers.append(nn.Sequential(
            nn.Conv2d(input_channels, output_channels, 3),
            nn.ReLU(),
            nn.Conv2d(output_channels, output_channels, 3),
            nn.ReLU(),
            nn.Conv2d(input_channels, 2, 1)
        ))


    def forward(self, x):
        return self._forward_recurse(x, 0)
        

    def _forward_recurse(self, x, n) -> torch.Tensor:
        # Pass through encoder layer, -> output accumulator -> decoder layer
        #                             -> next layer
        if n >= 4:
            return x
        x = self.encoder_layers[n](x)
        x = self.decoder_layers[n](torch.cat([x, self._forward_recurse(x, n + 1)], dim=1))
        return x

