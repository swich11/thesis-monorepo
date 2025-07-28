import warp as wp


@wp.func
def vec3_exp(exponent: wp.vec3):
    """
        Element-wise exponentiation by e^(exponent)
    """
    return wp.vec3(wp.exp(exponent[0]), wp.exp(exponent[1]), wp.exp(exponent[2]), dtype=type(exponent[0]))


@wp.func
def vec3_mul(vec_1: wp.vec3,
            vec_2: wp.vec3):
    """
        Element-wise multiplication
    """
    return wp.vec3(vec_1[0] * vec_2[0], vec_1[1] * vec_2[1], vec_1[2] * vec_2[2], dtype=type(vec_1[0]))
