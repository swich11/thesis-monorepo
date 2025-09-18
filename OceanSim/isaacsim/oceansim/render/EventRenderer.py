import warp as wp
import numpy as np


from typing import Tuple # type:ignore


from .Renderer import Renderer
from .util import vec3_exp, vec3_mul


"""
    Calculates events from high dynamic range graphics inputs
"""
@wp.kernel
def calc_events(hdr_curr: wp.array(ndim=3, dtype=wp.float16),
            pixel_last: wp.array(ndim=2, dtype=wp.float32),
            depth_image: wp.array(ndim=2, dtype=wp.float32),
            backscatter_value: wp.vec3,
            atten_coeff: wp.vec3,
            backscatter_coeff: wp.vec3,
            threshold_on: wp.float32,
            threshold_off: wp.float32,
            std: wp.float32,
            seed: wp.uint32,
            on_output: wp.array(ndim=2, dtype=wp.bool),
            off_output: wp.array(ndim=2, dtype=wp.bool)):
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
    # check if pixel is on or off
    on = pixelIntensity - pixel_last[i, j] > threshold_on
    off = pixel_last[i, j] - pixelIntensity > threshold_off
    pixel_last[i, j] = wp.select((on or off), pixel_last[i,j], pixelIntensity)
    # save outputs to matrices
    on_output[i, j] = wp.bool(on)
    off_output[i, j] = wp.bool(off)


"""
    Produces output matrix for calculated discrete events.
"""
@wp.kernel
def gpu_render(on_input: wp.array(ndim=2, dtype=wp.bool),
               off_input: wp.array(ndim=2, dtype=wp.bool),
            frame_image: wp.array(ndim=3, dtype=wp.uint8)):
    i, j = wp.tid()
    frame_image[i, j, 0] = wp.select(on_input[i, j], wp.uint8(0), wp.uint8(255))
    frame_image[i, j, 2] = wp.select(off_input[i, j], wp.uint8(0), wp.uint8(255))
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


    """
        Calculates events which occured between 2 event frames. This is not adaptive because the
        frame times for isaacsim aren't easily controlled.

        Inputs:
            hdrCurr: 3D wp.array<float32> HDR AGBR
            depths: 2D wp.array<float32> depth map
        Outputs:
            on_pixels: 2D wp.array<bool>
            off_pixels: 2D wp.array<bool>
    """
    def calculate_events(self, hdrCurr: wp.array, depths: wp.array) -> Tuple[wp.array, wp.array]:
        seed = self._generator.integers(2**32, dtype=np.uint32) # randomisation seed
        on_pixels = wp.zeros(self.resolution, dtype=wp.bool)
        off_pixels = wp.zeros(self.resolution, dtype=wp.bool)
        wp.launch(
            dim=self.resolution,
            kernel=calc_events,
            inputs=[hdrCurr, self.pixel_store, depths, self._backscatter_value,
                    self._atten_coeff, self._backscatter_coeff, self._threshold_on,
                    self._threshold_off, self._noise_std, seed],
            outputs=[on_pixels, off_pixels],
        )
        return (on_pixels, off_pixels)


    """
        Takes calculated events and produces a render of the on and off pixels as per
        the event camera convention.

        Inputs:
            on_pixels: 2D wp.array<bool>
            off_pixels: 2D wp.array<bool>
        Outputs:
            event_frame: 3D wp.array<uint8> AGBR
    """
    def render(self, on_pixels: wp.array, off_pixels: wp.array) -> wp.array:
        frame_image = wp.zeros((self.resolution[0], self.resolution[1], 4), dtype=wp.uint8)
        wp.launch(
            dim=self.resolution,
            kernel=gpu_render,
            inputs=[on_pixels, off_pixels],
            outputs=[frame_image],
        )
        return frame_image
    
