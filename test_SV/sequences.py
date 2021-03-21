# These are in the Crazyflie frame, which happens to be set as the body frame
# of the drone when initiating before take-off

import script_variables as sc_v

square = [
    (0.5, 0.5, sc_v.DEFAULT_HEIGHT, 0),
    (-0.5, 0.5, sc_v.DEFAULT_HEIGHT, 0),
    (-0.5, -0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0.5, -0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0.5, 0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0)  # back to the center
]

square_more = [
    (0.5, 0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0.5, sc_v.DEFAULT_HEIGHT, 0),
    (-0.5, 0.5, sc_v.DEFAULT_HEIGHT, 0),
    (-0.5, 0, sc_v.DEFAULT_HEIGHT, 0),
    (-0.5, -0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0, -0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0.5, -0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0.5, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0.5, 0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0)  # back to the center
]

square_or = [
    (0.5, 0.5, sc_v.DEFAULT_HEIGHT, 90),
    (0.5, 0.5, sc_v.DEFAULT_HEIGHT, 180),
    (-0.5, 0.5, sc_v.DEFAULT_HEIGHT, 180),
    (-0.5, 0.5, sc_v.DEFAULT_HEIGHT, 270),
    (-0.5, -0.5, sc_v.DEFAULT_HEIGHT, 270),
    (-0.5, -0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0.5, -0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0.5, -0.5, sc_v.DEFAULT_HEIGHT, 90),
    (0, 0, sc_v.DEFAULT_HEIGHT, 180)  # back to the center
]

square_more_or = [
    (0.5, 0.5, sc_v.DEFAULT_HEIGHT, 90),
    (0.5, 0.5, sc_v.DEFAULT_HEIGHT, 180),
    (0, 0.5, sc_v.DEFAULT_HEIGHT, 180),
    (-0.5, 0.5, sc_v.DEFAULT_HEIGHT, 180),
    (-0.5, 0.5, sc_v.DEFAULT_HEIGHT, 270),
    (-0.5, 0, sc_v.DEFAULT_HEIGHT, 270),
    (-0.5, -0.5, sc_v.DEFAULT_HEIGHT, 270),
    (-0.5, -0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0, -0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0.5, -0.5, sc_v.DEFAULT_HEIGHT, 0),
    (0.5, -0.5, sc_v.DEFAULT_HEIGHT, 90),
    (0.5, 0, sc_v.DEFAULT_HEIGHT, 90),
    (0.5, 0.5, sc_v.DEFAULT_HEIGHT, 90),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0)  # back to the center
]

triangle = [
    (0.5, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT + 0.5, 0),
    (-0.5, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0),

    (0, 0.5, sc_v.DEFAULT_HEIGHT + 0.5, 0),
    (0, -0.5, sc_v.DEFAULT_HEIGHT + 0.5, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0)  # back to the center
]
