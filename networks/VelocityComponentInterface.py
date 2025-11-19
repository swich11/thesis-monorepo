from abc import ABC


import torch
from VelometryComponent import VelometryComponent
from utils import crop


from typing import List, Tuple


class VelocityComponentInterface(VelometryComponent, ABC):
    def forward(self, x, v, mems) -> Tuple[torch.Tensor, torch.Tensor]:
        """
            Forward call with velocity input.
        """
        return self._forward_recurse(x, v, mems, 0)
    

    def _forward_recurse(self, x: torch.Tensor, v: torch.Tensor, mems: List[torch.Tensor], n: int) -> Tuple[torch.Tensor, torch.Tensor]:
        """
            Recursive call to U-Net structure
        """
        # Pass through encoder layer, -> output accumulator -> decoder layer
        #                             -> next layer
        if n > 0:
            x = self.max_pool(x)
        x, z, mems[0], mems[1] = self.encoder_layers[n](x, mems[0], mems[1])
        if n == self.depth:
            x = self.encoder_layers[n + 1](x) # do the first upconvolution
            return x, mems
        y, mems[2:] = self._forward_recurse(x, v, mems[2:], n + 1)
        # Crop the skip connection
        z = crop(z, y)
        v = v.unsqueeze(-1).unsqueeze(-1) # add 2 spatial dimensions
        v = v.expand(-1, -1, z.shape[2], z.shape[3]) # match height and width
        
        # concatenate on input channels, combining image and velocity
        x = self.decoder_layers[n](torch.cat([z, y, v], dim=1))
        return (x, mems)