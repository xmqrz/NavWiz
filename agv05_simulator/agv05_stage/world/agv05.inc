define urg ranger
(
  sensor
  (
    range [ 0.0 30.0 ]
    fov 180.0
    samples 721
  )

  # generic model properties
  color "black"
  size [ 0.05 0.05 0.10 ]
)

define agv05 position
(
  name "agv05"
  color "blue"
  size [ 0.8 0.6 0.25 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  stack_children 0

  urg
  (
    pose [ 0.35 0.0 0.15 0.0 ]
  )

  localization "odom"
  odom_error [ 0.03 0.03 0.0 0.05 ]

  velocity_bounds [ -1 1 -1 1 -1 1 -90 90 ]
  acceleration_bounds [ -1 1 -1 1 -1 1 -90 90 ]
)
