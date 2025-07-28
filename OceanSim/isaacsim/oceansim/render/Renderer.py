import warp as wp


from abc import abstractmethod # type: ignore
from typing import Tuple # type: ignore


class Renderer():
    """
        Base renderer class
    """
    def __init__(self, resolution: Tuple[int, int]):
        self.resolution: Tuple[int, int] = resolution


    def getResolution(self) -> Tuple[int, int]:
        return self.resolution
    
    
    @abstractmethod
    def render(self) -> wp.array:
        """
            Abstract method, Renderer classes must provide their own renderer function.
            This should call a warp kernel to perform rendering on a gpu in most cases.
        """
        pass