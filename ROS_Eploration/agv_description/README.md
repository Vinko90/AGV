# Robot description (URDF)

If you want to change the dimensions of the robot, make sure to do it in all these places:

- URDF model: `agv_description/urdf/description.xacro`
    - edit the size parameters
- move_base: `agv_bringup/config/navigation/costmap_common.yaml`
	- footprint
- stage model: `agv_stage/worlds/agv.inc`
    - agv.size
    - agv.origin

If you want the change the dynamics of the robot, make sure to do it in all these places:

- move_base limits: `agv_bringup/config/navigation/planner.yaml`
- simulation dynamics (velocity smoother): `agv_bringup/launch/start.launch`