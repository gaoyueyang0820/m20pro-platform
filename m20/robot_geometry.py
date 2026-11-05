"""Shared M20 navigation geometry; not a certification of terrain passability.

The installed robot selects `m20` in planner/config/robot_type.yaml. Its
planner/config/robot_profiles.yaml reports 0.84 m length and 0.45 m width
(read-only capture: data/network/resume-native-planner-config-20261030.json).
The console adds 3 cm per side. Raster cell and rotational-sweep uncertainty
are handled by each planner separately, rather than changing body dimensions.
No factory configuration is modified by this module.
"""
BODY_LENGTH=.84
BODY_WIDTH=.45
CLEARANCE_MARGIN=.03
BODY_HALF_LENGTH=BODY_LENGTH/2
BODY_HALF_WIDTH=BODY_WIDTH/2
CLEARANCE_HALF_LENGTH=BODY_HALF_LENGTH+CLEARANCE_MARGIN
CLEARANCE_HALF_WIDTH=BODY_HALF_WIDTH+CLEARANCE_MARGIN
