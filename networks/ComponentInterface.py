from abc import ABC

import torch
from VelometryComponent import VelometryComponent
from utils import crop


class ComponentInterface(VelometryComponent, ABC):
    # Forward passes will just error if the wrong one is used
    def forward(self, x):
        """
            Forward call without velocity input.
        """
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
        # crop for the skip connection here
        x = crop(x, y)
        x = self.decoder_layers[n](torch.cat([x, y], dim=1))
        return x