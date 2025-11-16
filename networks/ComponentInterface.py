from abc import ABC

import torch
from VelometryComponent import VelometryComponent
from utils import crop


from typing import List, Tuple


class ComponentInterface(VelometryComponent, ABC):
    # Forward passes will just error if the wrong one is used
    def forward(self, x, mems) -> Tuple[torch.Tensor, List[torch.Tensor]]:
        """
            Forward call without velocity input.
        """
        return self._forward_recurse(x, mems, 0)
    

    def _forward_recurse(self, x: torch.Tensor, mems: List[torch.Tensor], n: int) -> Tuple[torch.Tensor, List[torch.Tensor]]:
        """
            Recursive call to U-Net structure.
        """
        # Pass through encoder layer, -> output accumulator -> decoder layer
        #                             -> next layer
        if n > 0:
            x = self.max_pool(x) # pass x through max pooling
        x, z, mems[0], mems[1] = self.encoder_layers[n](x, mems[0], mems[1])
        if n == self.depth:
            x = self.encoder_layers[n + 1](mems[1]) # do the first upconvolution
            return x, mems
        y, mems[2:] = self._forward_recurse(x, mems[2:], n + 1)
        # crop for the skip connection here
        z = crop(z, y) # skip connection comes from conv layer
        x = self.decoder_layers[n](torch.cat([z, y], dim=1))
        return (x, mems)