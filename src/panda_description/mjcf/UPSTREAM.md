Model and meshes: [MuJoCo Menagerie / franka_emika_panda](https://github.com/google-deepmind/mujoco_menagerie/tree/cda5236f98383f6fbaa4820c6b18dba892cd087f/franka_emika_panda).

Commit: `cda5236f98383f6fbaa4820c6b18dba892cd087f`. Apache-2.0; see LICENSE.

Local changes: arm motors use direct torque commands, all moving bodies compensate gravity, arm names match ROS, timestep is 1 ms, fingers use symmetric prismatic joints, two 70 N force-limited motors and a coupling constraint, initially opened to 80 mm, home matches the ROS configuration, and a TCP site is included. The scene contains only the fixed robot and a floor at z = -0.01 m.
