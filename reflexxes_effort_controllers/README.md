Reflexxes Effort Controllers
============================

Example rosparam controller configuration:

```yml
left_joint_position_controller:
  type: reflexxes_effort_controllers/JointTrajectoryController
  sampling_resolution: 0.001
  joint_names: 
    - 'joint_1'
    - 'joint_2'
    - 'joint_3'
  joints:
    joint_1:
      pid: {p: 10.0, i: 0.0, d: 2.0}
      position_tolerance: 0.1
      max_acceleration: 0.5
    joint_2:
      pid: {p: 10.0, i: 0.0, d: 2.0}
      position_tolerance: 0.1
      max_acceleration: 0.5
    joint_3:
      pid: {p: 10.0, i: 0.0, d: 2.0}
      position_tolerance: 0.1
      max_acceleration: 0.5
```
