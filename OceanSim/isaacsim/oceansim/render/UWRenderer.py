import warp as wp
import numpy as np


from .Renderer import Renderer
from .util import vec3_exp, vec3_mul


@wp.kernel
def gpu_render(raw_image: wp.array(ndim=3, dtype=wp.uint8),
             depth_image: wp.array(ndim=2, dtype=wp.float32),
             backscatter_value: wp.vec3,
             atten_coeff: wp.vec3,
             backscatter_coeff: wp.vec3,
             uw_image: wp.array(ndim=3, dtype=wp.uint8)):
    i,j = wp.tid()
    raw_RGB = wp.vec3(wp.float32(raw_image[i,j,0]), wp.float32(raw_image[i,j,1]), wp.float32(raw_image[i,j,2]), dtype=wp.float32)
    depth = depth_image[i,j]
    exp_atten = vec3_exp(- depth * atten_coeff)
    exp_back = vec3_exp(- depth * backscatter_coeff)
    UW_RGB = vec3_mul(raw_RGB, exp_atten) + vec3_mul(backscatter_value * wp.float32(255), (wp.vec3f(1.0,1.0,1.0) - exp_back) )
    uw_image[i,j,0] = wp.uint8(wp.clamp(UW_RGB[0], wp.float32(0), wp.float32(255)))
    uw_image[i,j,1] = wp.uint8(wp.clamp(UW_RGB[1], wp.float32(0), wp.float32(255)))
    uw_image[i,j,2] = wp.uint8(wp.clamp(UW_RGB[2], wp.float32(0), wp.float32(255)))
    uw_image[i,j,3] = raw_image[i, j, 3]


class UWRenderer(Renderer):
    """
        Renderer produces RGBA images with underwater lighting effects.
    """
    def __init__(self, 
                 resolution=(1920, 1080),
                 backscatter_value: wp.vec3f = wp.vec3f(0.0, 0.0, 0.0),
                 atten_coeff: wp.vec3f = wp.vec3f(0.0, 0.0, 0.0),
                 backscatter_coeff: wp.vec3f = wp.vec3f(0.0, 0.0, 0.0)):
        self._backscatter_value: wp.vec3f = backscatter_value
        self._atten_coeff: wp.vec3f = atten_coeff
        self._backscatter_coeff: wp.vec3f = backscatter_coeff
        super().__init__(resolution)


    def render(self, raw_image: wp.array, depth_image: wp.array) -> wp.array:
        uw_image = wp.zeros_like(raw_image)
        wp.launch(
            dim=np.flip(self.resolution),
            kernel=gpu_render,
            inputs=[
                raw_image,
                depth_image,
                self._backscatter_value,
                self._atten_coeff,
                self._backscatter_coeff
            ],
            outputs=[
                uw_image
            ]
        )
        return uw_image



