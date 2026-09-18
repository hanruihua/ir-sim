"""
Library-wide default colors.

Every default IR-SIM draws with is defined here, so the look of a whole
scene can be changed in one place. The values come from the Okabe-Ito
palette (with a second green from Paul Tol's muted scheme), which stays
distinguishable under common forms of colour-vision deficiency and keeps
robots, pushable objects, and static obstacles apart in grayscale print.
Any Matplotlib color given in YAML or code overrides these defaults.
"""

# Robots: green, as IR-SIM has always drawn them, in a shade that prints well.
ROBOT_COLOR = "#009E73"  # diff, omni, omni_angular and custom kinematics
ROBOT_ACKER_COLOR = "#117733"  # a darker green for car-like robots

# Obstacles: black when static, orange once a finite ``mass`` makes them pushable.
OBSTACLE_COLOR = "k"
DYNAMIC_BODY_COLOR = "#E69F00"

# Object decorations.
ARROW_COLOR = "#F0E442"  # heading arrow drawn on top of the body
FOV_COLOR = "#56B4E9"  # field-of-view fill
FOV_EDGE_COLOR = "#0072B2"  # field-of-view outline

# Sensors.
LIDAR_COLOR = "#CC3311"  # lidar beams
LASER_HIGHLIGHT_COLOR = "#56B4E9"  # beams singled out with ``set_laser_color``
FMCW_ZERO_VELOCITY_COLOR = "#56B4E9"
FMCW_POSITIVE_VELOCITY_COLOR = "#CC3311"
FMCW_NEGATIVE_VELOCITY_COLOR = "#0072B2"
