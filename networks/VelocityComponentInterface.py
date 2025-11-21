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
        if n == self.depth:
            x = self.residual_layer(x)
            return x, mems
        else:
            x, mems[0], mems[1] = self.encoder_layers[n](x, mems[0], mems[1])
        y, mems[2:] = self._forward_recurse(x, v, mems[2:], n + 1)
        # Crop the skip connection
        x = crop(x, y)
        v = v.unsqueeze(-1).unsqueeze(-1) # add 2 spatial dimensions
        v = v.expand(-1, -1, x.shape[2], x.shape[3]) # match height and width
        
        # concatenate on input channels, combining image and velocity
        x = self.decoder_layers[n](torch.cat([x, y, v], dim=1))
        return (x, mems)