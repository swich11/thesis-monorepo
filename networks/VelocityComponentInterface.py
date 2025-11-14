from abc import ABC


import torch
from VelometryComponent import VelometryComponent
from utils import crop


class VelocityComponentInterface(VelometryComponent, ABC):
    def forward(self, x, v):
        """
            Forward call with velocity input.
        """
        return self._forward_recurse(x, v, 0)
    

    def _forward_recurse(self, x: torch.Tensor, v: torch.Tensor, n: int) -> torch.Tensor:
        """
            Recursive call to U-Net structure
        """
        # Pass through encoder layer, -> output accumulator -> decoder layer
        #                             -> next layer
        x = self.encoder_layers[n](x)
        if n == self.depth:
            return x
        y = self._forward_recurse(x, v, n + 1)
        # Crop the skip connection
        x = crop(x, y)
        v = v.unsqueeze(-1).unsqueeze(-1) # add 2 spatial dimensions
        v = v.expand(-1, -1, x.shape[2], x.shape[3]) # match height and width
        
        # concatenate on input channels, combining image and velocity
        x = self.decoder_layers[n](torch.cat([x, y, v], dim=1))
        return x