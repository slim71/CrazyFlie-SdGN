from own_module import script_variables as sc_v

test = [
    (0, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0.3, 0.3, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0)  # back to the center
]

line = [
    (0, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0.3, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0),
    (-0.3, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0)  # back to the center
]

square = [
    (0, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0.3, 0.3, sc_v.DEFAULT_HEIGHT, 0),
    (-0.3, 0.3, sc_v.DEFAULT_HEIGHT, 0),
    (-0.3, -0.3, sc_v.DEFAULT_HEIGHT, 0),
    (0.3, -0.3, sc_v.DEFAULT_HEIGHT, 0),
    (0.3, 0.3, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0)  # back to the center
]

square_more = [
    (0, 0, sc_v.DEFAULT_HEIGHT, 0),
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
    (0, 0, sc_v.DEFAULT_HEIGHT, 0),
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
    (0, 0, sc_v.DEFAULT_HEIGHT, 0),
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
    (0, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0.5, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT + 0.5, 0),
    (-0.5, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0),

    (0, 0.5, sc_v.DEFAULT_HEIGHT + 0.5, 0),
    (0, -0.5, sc_v.DEFAULT_HEIGHT + 0.5, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0)  # back to the center
]

hover = [
    (0, -1.5, sc_v.DEFAULT_HEIGHT, 0),
    (0, -1.5, sc_v.DEFAULT_HEIGHT, 0),
    (0, -1.5, sc_v.DEFAULT_HEIGHT, 0),
    (0, -1.5, sc_v.DEFAULT_HEIGHT, 0),
    (0, -1.5, sc_v.DEFAULT_HEIGHT, 0),
    (0, -1.5, sc_v.DEFAULT_HEIGHT, 0),
]

other_square = [
    (0, 0-0.3, sc_v.DEFAULT_HEIGHT, 0),
    (0.3, 0.3-0.3, sc_v.DEFAULT_HEIGHT, 0),
    (-0.3, 0.3-0.3, sc_v.DEFAULT_HEIGHT, 0),
    (-0.3, -0.3-0.3, sc_v.DEFAULT_HEIGHT, 0),
    (0.3, -0.3-0.3, sc_v.DEFAULT_HEIGHT, 0),
    (0.3, 0.3-0.3, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0-0.3, sc_v.DEFAULT_HEIGHT, 0),  # back to the center
(0, 0-0.3, 0.3, 0)
]