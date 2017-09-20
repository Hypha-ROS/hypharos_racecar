define kinect ranger
(
  sensor
  (
    range_max 10
    fov 270.0
    samples 1280
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  #odom_error [0.000 0.000 999999 999999 999999 0.000]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  kinect(pose [ -0.1 0.0 -0.11 0.0 ])
)
