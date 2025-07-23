import warp as wp


@wp.func
def vec3_exp(exponent: wp.vec3):
    return wp.vec3(wp.exp(exponent[0]), wp.exp(exponent[1]), wp.exp(exponent[2]), dtype=type(exponent[0]))

@wp.func
def vec3_mul(vec_1: wp.vec3,
            vec_2: wp.vec3):
    return wp.vec3(vec_1[0] * vec_2[0], vec_1[1] * vec_2[1], vec_1[2] * vec_2[2], dtype=type(vec_1[0]))

@wp.kernel
def UW_render(raw_image: wp.array(ndim=3, dtype=wp.uint8),
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
    uw_image[i,j,3] = raw_image[i,j,3]


@wp.kernel
def EVrender(hdr_curr: wp.array(ndim=3, dtype=wp.float16),
            pixel_last: wp.array(ndim=2, dtype=wp.float32),
            depth_image: wp.array(ndim=2, dtype=wp.float32),
            backscatter_value: wp.vec3,
            atten_coeff: wp.vec3,
            backscatter_coeff: wp.vec3,
            threshold_on: wp.float32,
            threshold_off: wp.float32,
            std: wp.float32,
            time: wp.uint32, # for random
            frame_image: wp.array(ndim=3, dtype=wp.uint8)):
    i, j  = wp.tid()
    hdrNow = wp.vec3(wp.float32(hdr_curr[i, j, 0]), wp.float32(hdr_curr[i, j, 1]), wp.float32(hdr_curr[i, j, 2]))
    exp_atten = vec3_exp(- depth_image[i, j] * atten_coeff)
    exp_back = vec3_exp(- depth_image[i, j] * backscatter_coeff)
    hdrAttenuated = vec3_mul(hdrNow, exp_atten) + vec3_mul(backscatter_value, (wp.vec3f(1.0, 1.0, 1.0) - exp_back))

    pixelIntensity = wp.log2(hdrAttenuated[0] + hdrAttenuated[1] + hdrAttenuated[2]) + wp.float32(wp.randn(time + wp.uint32(i) + wp.uint32(j)))*std # add random threshold diff to the pixel value

    # grab on and off pixels
    on = pixelIntensity - pixel_last[i,j] > threshold_on
    off = pixel_last[i,j] - pixelIntensity > threshold_off
    pixel_last[i,j] = wp.select((on or off), pixel_last[i,j], pixelIntensity)

    # save the output image for actual render
    frame_image[i, j, 0] = wp.select(on, wp.uint8(0), wp.uint8(255))
    frame_image[i, j, 2] = wp.select(off, wp.uint8(0), wp.uint8(255))
    frame_image[i, j, 3] = wp.uint8(255) #alpha channel