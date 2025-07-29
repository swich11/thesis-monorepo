import warp as wp
import numpy as np


from .Renderer import Renderer
from .util import vec3_exp, vec3_mul


@wp.kernel
def gpu_render(hdr_curr: wp.array(ndim=3, dtype=wp.float16),
            pixel_last: wp.array(ndim=2, dtype=wp.float32),
            depth_image: wp.array(ndim=2, dtype=wp.float32),
            backscatter_value: wp.vec3,
            atten_coeff: wp.vec3,
            backscatter_coeff: wp.vec3,
            threshold_on: wp.float32,
            threshold_off: wp.float32,
            std: wp.float32,
            seed: wp.uint32, # for random
            frame_image: wp.array(ndim=3, dtype=wp.uint8)):
    i, j  = wp.tid()
    # attenuation from oceansim
    hdrNow = wp.vec3(wp.float32(hdr_curr[i, j, 0]), wp.float32(hdr_curr[i, j, 1]), wp.float32(hdr_curr[i, j, 2]))
    exp_atten = vec3_exp(- depth_image[i, j] * atten_coeff)
    exp_back = vec3_exp(- depth_image[i, j] * backscatter_coeff)
    hdrAttenuated = vec3_mul(hdrNow, exp_atten) + vec3_mul(backscatter_value, (wp.vec3f(1.0, 1.0, 1.0) - exp_back))

    # sum pixel intensities and add random noise
    pixelIntensity = (wp.log2(hdrAttenuated[0] + hdrAttenuated[1] + hdrAttenuated[2]) 
                    + wp.float32(wp.randn(seed + wp.uint32(i) 
                               + wp.uint32(j)*wp.uint32(wp.pow(2.0, 16.0))))*std)

    # grab on and off pixels
    on = pixelIntensity - pixel_last[i,j] > threshold_on
    off = pixel_last[i,j] - pixelIntensity > threshold_off
    pixel_last[i,j] = wp.select((on or off), pixel_last[i,j], pixelIntensity)

    # save the output image for actual render
    frame_image[i, j, 0] = wp.select(on, wp.uint8(0), wp.uint8(255))
    frame_image[i, j, 2] = wp.select(off, wp.uint8(0), wp.uint8(255))
    frame_image[i, j, 3] = wp.uint8(255) #alpha channel


class EventRenderer(Renderer):
    def __init__(self, 
                 resolution = (346, 260),
                 threshold_on: float = 0.143,
                 threshold_off: float = 0.225,
                 std: float = 0.03,
                 backscatter_value: wp.vec3f = wp.vec3f(0.0, 0.0, 0.0),
                 atten_coeff: wp.vec3f = wp.vec3f(0.0, 0.0, 0.0),
                 backscatter_coeff: wp.vec3f = wp.vec3f(0.0, 0.0, 0.0)):
        log_val = np.log2(10)
        self._threshold_on: wp.float32 = wp.float32(threshold_on*log_val) # shift thresholds to log2 space for computational efficiency
        self._threshold_off: wp.float32 = wp.float32(threshold_off*log_val)
        self.pixel_store: wp.array = wp.zeros(np.flip(resolution), wp.float32)
        self._noise_std: wp.float32 = wp.float32(std*log_val)

        self._backscatter_value: wp.vec3f = backscatter_value
        self._atten_coeff: wp.vec3f = atten_coeff
        self._backscatter_coeff: wp.vec3f = backscatter_coeff
        
        self._generator = np.random.default_rng(seed=42) # random number generator

        super().__init__(np.flip(resolution)) # flip to match annotator shape


    def render(self, hdrCurr: wp.array, depths: wp.array) -> wp.array:
        seed = self._generator.integers(2**32, dtype=np.uint32) # grab a random seed
        frame_image = wp.zeros((260, 346, 4), dtype=wp.uint8)
        wp.launch(
            dim=self.resolution,
            kernel=gpu_render,
            inputs=[hdrCurr, self.pixel_store, depths, self._backscatter_value,
                      self._atten_coeff, self._backscatter_coeff, self._threshold_on,
                      self._threshold_off, self._noise_std, seed],  # time is for random standard deviation
            outputs=[frame_image],

        )
        return frame_image
    
