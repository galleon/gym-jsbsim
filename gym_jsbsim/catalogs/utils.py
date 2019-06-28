def reduce_reflex_angle_deg(angle):
    """ Given an angle in degrees, normalises in [-179, 180] """

    new_angle = angle % 360

    if new_angle > 180:
        new_angle -= 360

    return new_angle