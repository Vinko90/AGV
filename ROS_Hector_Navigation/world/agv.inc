define human model
(
  size [0.3 0.3 2.0]
)

define lidar ranger
(
  size [ 0.050 0.050 0.100 ]
  color "orange"
  update_interval 25

  sensor(
    range [ 0 15.0 ]
    fov 180
    samples 128
  )
)

define agv position
(
  size [1.100 0.900 0.300]
  origin [ -0.250 0 0.050 0 ]
  color "grey"

  ranger_return 1.0

  gui_nose 1

  drive "diff"
  update_interval 25

  # child nodes
  stack_children 1

  lidar(pose [ 0.300 0 -0.050 0 ])
)

define floorplan model
(
  # sombre, sensible, artistic
  color "gray50"

  # most maps will need a bounding box
  boundary 1
  gui_grid 0
  gui_outline 0
  gui_move 0
  ranger_return 1.0
)
