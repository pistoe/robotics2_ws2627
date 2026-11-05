# Robotics 2 winter term 26/27 group challenge

ROS 2 Jazzy on Ubuntu 24.04, with a Franka Emika **Panda**, a physical gripper, MuJoCo, MoveIt 2 and RViz. This repository provides the simulator and its interfaces. Implement your own perception, planning and task execution in `workspace/src/`.

Read [CHALLENGE.md](CHALLENGE.md) or the [illustrated challenge description](assets/challenge-description.pdf) for the six tasks, workspace labels and completion rules. The tasks form one continuous run. No workpiece poses are supplied, including in Level 1.

## Installation

Use a Linux x86-64 host, preferably Ubuntu 24.04. MuJoCo and RViz need an X11 or XWayland desktop. The image includes ROS 2 Jazzy and the simulation dependencies. Allow about 15 GB if building from source. The RGB-D cameras need OpenGL even when running without windows.

Clone or unpack the student repository and run the host commands from its root directory. Choose one of the two installation paths below.

### Podman

Install rootless Podman with the `crun` runtime. On Ubuntu:

```bash
sudo apt update
sudo apt install podman crun xauth mesa-utils python3
```

Check the host setup, then build the image or load the course archive.

```bash
./tools/panda doctor
./tools/panda build
```

If the course provides a prebuilt image, load it instead of building:

```bash
./tools/panda load /path/to/robotics2-panda-student-jazzy.oci.tar
```

### Docker Engine on Linux

Use a local Docker Engine installation with the default UID/GID mapping. On Ubuntu, follow the Docker Engine installation guide, including the Buildx plugin if you plan to build the image. Run `docker info` from your normal desktop account to check that it can access the engine. Install `xauth` for the graphical launch. The commands below target native Linux and use its display socket and GPU devices. [Docker Engine installation on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)

```bash
sudo apt install xauth
docker info
```

Load the Docker archive from the course downloads. If the course supplies `SHA256SUMS`, run `sha256sum -c --ignore-missing SHA256SUMS` in the download directory first. [Docker image load](https://docs.docker.com/reference/cli/docker/image/load/)

```bash
docker image load -i /path/to/robotics2-panda-student-jazzy.docker.tar.gz
```

To build from the student repository, use the included `container/Containerfile`. `--load` makes the built image available to `docker run`. [Docker Buildx reference](https://docs.docker.com/reference/cli/docker/buildx/build/)

```bash
docker buildx build --load --platform linux/amd64 --target runtime \
  -f container/Containerfile -t localhost/robotics2-panda:student-jazzy .
```

Both paths use the tag `localhost/robotics2-panda:student-jazzy`. The prebuilt image gives everyone the same dependencies. Source builds need network access and rely on the continued availability of the versions in `versions.lock.yaml`. The image records the resolved Debian packages in `/opt/panda/packages.lock`.

## Three environments

Run one environment at a time. MuJoCo and RViz open as desktop windows; MoveIt starts with them.

| Podman command | ROS launch file | Environment |
| --- | --- | --- |
| `./tools/panda sim` | `simulation.launch.py` | Empty workbench with the Panda, hand and flat surface. |
| `./tools/panda competition` | `competition.launch.py` | Complete challenge with reference seed `0`. |
| `./tools/panda competition-randomized` | `competition_randomized.launch.py` | Complete challenge with a fresh randomized arrangement at launch. |

### Launch with Docker

Run this block in Bash from the student repository. Set `panda_launch` to a launch file from the table above. Put any ROS launch arguments in `panda_launch_args`, for example `(seed:=42)` for a repeatable randomized training run. Set `panda_gui=0` to run without the MuJoCo and RViz windows while keeping the challenge cameras and MoveIt available.

```bash
(
  set -e -o pipefail
  panda_launch=competition_randomized.launch.py
  panda_launch_args=()
  panda_gui=1
  panda_docker_args=()

  mkdir -p workspace/src logs
  if [[ "${PANDA_SOFTWARE_RENDERING:-0}" != 1 ]]; then
    for panda_device in /dev/dri/card* /dev/dri/renderD*; do
      if [[ -c "$panda_device" ]]; then
        panda_docker_args+=(--device "$panda_device" --group-add "$(stat -c '%g' "$panda_device")")
      fi
    done
  fi
  if ((${#panda_docker_args[@]} == 0)); then
    panda_docker_args+=(-e LIBGL_ALWAYS_SOFTWARE=1)
    if [[ "$panda_gui" == 0 ]]; then
      panda_docker_args+=(-e EGL_PLATFORM=surfaceless)
    fi
  fi

  if [[ "$panda_gui" == 1 ]]; then
    panda_xauth=$(mktemp)
    trap 'rm -f "$panda_xauth"' EXIT
    xauth nlist "${DISPLAY:?Run from your Linux desktop session}" |
      sed 's/^..../ffff/' | xauth -f "$panda_xauth" nmerge -
    if [[ ! -s "$panda_xauth" ]]; then
      echo "No Xauthority entry for $DISPLAY. Check your desktop session." >&2
      exit 1
    fi
    panda_docker_args+=(-e DISPLAY -e XAUTHORITY=/tmp/student.Xauthority
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro
      -v "$panda_xauth:/tmp/student.Xauthority:ro")
  else
    panda_launch_args+=(headless:=true use_rviz:=false)
  fi

  docker run --rm -it --init --stop-signal SIGINT \
    --name "robotics2-student-$(id -u)" --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/student-home -e ROS_LOG_DIR=/data/log -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}" \
    -v "$PWD/workspace:/workspaces/student" -v "$PWD/logs:/data/log" \
    "${panda_docker_args[@]}" localhost/robotics2-panda:student-jazzy \
    ros2 launch panda_bringup "$panda_launch" "${panda_launch_args[@]}"
)
```

From a second terminal, open a shell in the running container. The entrypoint loads ROS and your built workspace. You can run every `ros2` command from this README in that shell by dropping the `./tools/panda exec` prefix. Use Ctrl+C in the launch terminal or `docker stop --time 20 "robotics2-student-$(id -u)"` to stop the simulation. [Docker runtime options](https://docs.docker.com/engine/containers/run/)

```bash
docker exec -it "robotics2-student-$(id -u)" /opt/panda/entrypoint.sh bash
ros2 topic echo /panda/simulation/ready --once --qos-durability transient_local
```

### Launch options and readiness

With Podman, use a reproducible training seed:

```bash
./tools/panda competition-randomized seed:=42
```

Reference and randomized modes share the same models, fixtures, tasks and placement rules. An identical explicit seed produces the same initial arrangement in either mode. Randomization occurs once at launch; parts then move through physics. There are fifteen movable parts: fourteen on A0 and the D-shaped peg inside the closed drawer. Do not restart or reset between challenge levels.

Useful launch options:

```bash
# No desktop windows; keep both RGB-D streams and MoveIt.
./tools/panda competition-randomized seed:=42 headless:=true use_rviz:=false

# Minimal robot-only practice without MoveIt or windows.
./tools/panda sim headless:=true use_rviz:=false use_moveit:=false

# Begin paused after controllers are ready.
./tools/panda competition start_paused:=true

# Select either camera, or disable rendering for control-only practice.
./tools/panda competition wrist_camera:=false
./tools/panda competition external_camera:=false
./tools/panda competition camera:=false

# List launch arguments without starting an environment.
./tools/panda shell --headless
ros2 launch panda_bringup competition_randomized.launch.py --show-args
exit
```

Inside a container the corresponding launch files are `simulation.launch.py`, `competition.launch.py` and `competition_randomized.launch.py` in `panda_bringup`.

Wait for the startup barrier before issuing commands. Run these checks from a second host terminal with Podman, or run the `ros2` commands in the Docker shell above.

```bash
./tools/panda exec ros2 topic echo /panda/simulation/ready --once --qos-durability transient_local
./tools/panda exec ros2 control list_controllers -c /panda/controller_manager
```

Readiness must be `true`; `joint_state_broadcaster` and `panda_arm_controller` must be `active`. Physics is gated until controller startup completes. Shut down with Ctrl+C in the launch terminal, or:

```bash
./tools/panda stop
```

## Develop your application

The host directory `workspace/` is mounted at `/workspaces/student`. Your source, build and install outputs persist after the simulator exits. `logs/` contains ROS logs. Simulator runtime files remain inside the disposable container.

With an environment running, open a shell with `./tools/panda exec bash` for Podman or the `docker exec` command above. Then run the following inside the container.

```bash
cd /workspaces/student
ros2 pkg create --build-type ament_python my_robotics2_team --dependencies rclpy
colcon build --symlink-install
source install/setup.bash
# Run your own node or launch file here.
exit
```

Every new `./tools/panda exec ...` loads ROS, the simulator packages and an existing student workspace overlay. An already open shell must source the overlay again after building. Use `./tools/panda shell --headless` for development without starting the simulator. It uses the same container name, so stop the environment first.

Set `use_sim_time:=true` on your own nodes. All positions and dimensions use SI units: radians, metres and seconds. Use the timestamps in camera messages when looking up TF.

OpenCV (`cv2`), `cv_bridge`, NumPy and `tf2_geometry_msgs` are installed for perception and coordinate transforms. Add your own Python dependencies in a workspace virtual environment with `--system-site-packages` when needed; preserve the provided ROS and NumPy dependencies.

## Arm control

Default namespace: `/panda`. Inspect the interfaces:

```bash
./tools/panda exec ros2 topic list
./tools/panda exec ros2 action list -t
./tools/panda exec ros2 topic echo /panda/joint_states --once
```

Joint states may arrive as separate arm and finger messages. Merge values **by joint name**, rather than assuming one message contains all joints. Arm names are `panda_joint1` through `panda_joint7`.

The joint trajectory controller accepts:

| Interface | Type |
| --- | --- |
| `/panda/panda_arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` topic |
| `/panda/panda_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` action |
| `/panda/panda_arm_controller/controller_state` | Controller feedback topic |

Use the **empty environment** for these small examples. Direct trajectories interpolate joint positions; they do not check collisions or plan around the workspace. Use MoveIt for collision-aware paths and add collision geometry from your own object estimates.

Send a small five-second motion near the initial configuration:

```bash
./tools/panda exec ros2 topic pub --once \
  /panda/panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: [0.10, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 5}}]}'
```

Prefer an action in your application so that you can monitor feedback, cancellation and the result. This example returns to the initial configuration:

```bash
./tools/panda exec ros2 action send_goal --feedback \
  /panda/panda_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 5}}]}}'
```

A compact single-target action client is also included:

```bash
./tools/panda exec ros2 run panda_bringup joint_goal \
  0.10 -0.785398 0.0 -2.356194 0.0 1.570796 0.785398 --duration 5
```

## Gripper control

The hand exposes Franka-compatible action names. `width` is the **total opening between the fingers** in metres, from `0.0` to `0.08`; `speed` is in metres per second and `force` in newtons. There is no artificial attachment: objects are held by contact and friction.

```bash
# Open.
./tools/panda exec ros2 action send_goal --feedback \
  /panda/panda_gripper/move franka_msgs/action/Move '{width: 0.08, speed: 0.05}'

# Move to a 20 mm opening. Use move for positioning, grasp for holding a part.
./tools/panda exec ros2 action send_goal --feedback \
  /panda/panda_gripper/move franka_msgs/action/Move '{width: 0.02, speed: 0.03}'

# Example grasp request for an already aligned 40 mm part; this is not a pick routine.
./tools/panda exec ros2 action send_goal --feedback \
  /panda/panda_gripper/grasp franka_msgs/action/Grasp \
  '{width: 0.04, speed: 0.03, force: 40.0, epsilon: {inner: 0.005, outer: 0.005}}'

# Stop the hand, or perform homing with the hand clear of objects.
./tools/panda exec ros2 service call /panda/panda_gripper/stop std_srvs/srv/Trigger '{}'
./tools/panda exec ros2 action send_goal \
  /panda/panda_gripper/homing franka_msgs/action/Homing '{}'
```

Check both the action terminal status and its `success` result. A grasp request without a part between the fingers should fail. Force is a simulation control parameter, not a calibrated physical force sensor.

A standard interface is also available at `/panda/panda_gripper/gripper_action` (`control_msgs/action/GripperCommand`). Its `command.position` is **one finger's travel**, half the total opening: `0.04` means fully open. `max_effort: 0.0` selects position movement; a positive value selects grasp mode. Do not send competing goals through both interfaces.

```bash
./tools/panda exec ros2 action send_goal \
  /panda/panda_gripper/gripper_action control_msgs/action/GripperCommand \
  '{command: {position: 0.04, max_effort: 0.0}}'
```

The topics and Franka action types support later transfer to the matching Panda hardware stack. This student image contains the simulator, not a physical robot driver. Real hardware requires instructor setup, calibration and safety validation.

## MoveIt and RViz

The RViz MotionPlanning panel uses planning group `panda_arm` and TCP `panda_hand_tcp`. Select the `ready` named state, or set a target using the interactive marker, then **Plan** and inspect the path before **Execute**. The `panda_hand` group has `open` and `close` states. Robot state, the floor, fixed fixtures and camera collision geometry are available to MoveIt.

Movable workpieces and the moving drawer are **not** automatically added or updated using simulator truth. Estimate their state and add/update their collision geometry through the normal MoveIt planning scene interface. A planning attachment records a physical grasp for planning; it does not attach an object in MuJoCo.

## Cameras and transforms

Challenge modes provide an external RGB-D camera overlooking the scene and a wrist-mounted RGB-D camera. The current simulated optical profiles correspond to an external D435 and a wrist D405; rendering provides ideal images and clipped depth, without a full sensor noise model. Either camera or both may be used.

| Data | External | Wrist |
| --- | --- | --- |
| RGB | `/panda/camera/color/image_raw` | `/panda/wrist_camera/color/image_raw` |
| Depth | `/panda/camera/aligned_depth_to_color/image_raw` | `/panda/wrist_camera/aligned_depth_to_color/image_raw` |
| Calibration | `/panda/camera/color/camera_info` | `/panda/wrist_camera/color/camera_info` |
| Optical frame | `workspace_camera_optical_frame` | `wrist_camera_optical_frame` |

Both streams default to 1280 × 720 at 5 Hz. RGB uses `rgb8`; depth uses `16UC1` in **millimetres**, with zero for invalid/out-of-range pixels. External depth range is 0.28–3.0 m; wrist depth range is 0.10–0.50 m. Read the actual intrinsics from `CameraInfo`. Camera optical axes follow ROS conventions (+X right, +Y down, +Z forward). Images and depth are aligned to the same simulated camera projection.

Matching depth calibration is also on each camera's `aligned_depth_to_color/camera_info`. The external-only legacy aliases `camera/camera_info` and `camera/depth/image_raw` remain available; that legacy depth topic uses `32FC1` **metres**, unlike the recommended aligned-depth topics above.

TF is on `/tf` and `/tf_static`; the fixed frame is `world`. Robot and camera transforms are published. There are no dynamic workpiece transforms, pose topics or supplied initial cube poses. Fixed equipment geometry and coordinates are public. Camera rate can be changed with `camera_rate:=5.0` (allowed range: greater than zero, up to 30 Hz); higher rates increase GPU load.

## Pause, reset and repeat during practice

These controls are for practice between attempts. A competition attempt proceeds through all six levels without a reset.

```bash
# Pause and resume.
./tools/panda exec ros2 service call /panda/mujoco_ros2_control_node/set_pause \
  mujoco_ros2_control_msgs/srv/SetPause '{paused: true}'
./tools/panda exec ros2 service call /panda/mujoco_ros2_control_node/set_pause \
  mujoco_ros2_control_msgs/srv/SetPause '{paused: false}'

# Managed reset: cancels the arm goal, restores the launch state and ends PAUSED.
./tools/panda exec ros2 service call /panda/simulation/reset std_srvs/srv/Trigger '{}'
# Resume explicitly after a successful reset.
./tools/panda exec ros2 service call /panda/mujoco_ros2_control_node/set_pause \
  mujoco_ros2_control_msgs/srv/SetPause '{paused: false}'
```

A reset restores the **same** initial arrangement. Stop and relaunch with a different seed for another training arrangement. MuJoCo Backspace also resets; prefer the managed ROS reset while controlling the arm. Empty-scene practice has no loose objects.

## Training and assessment

Training seeds are public and repeatable. Use them to evaluate your method, not as a lookup key for object poses. During a challenge, workpiece state must come from the permitted camera observations and your action history. Robot state, calibration, object models and fixed fixture geometry are available. The repository supplies no task solver or reference trajectories.

A simulator running on a machine you control cannot conceal its generated scene or seed from its owner. The final unseen seed is therefore chosen and run by the instructor in a controlled evaluation environment; it is not included in this repository or image. Submit an application that uses the documented ROS interfaces and does not depend on simulator files, state editing, privileged state feeds or seed-to-pose lookup tables. Human intervention and resets during a scored attempt follow the course rules.

## Troubleshooting

- **Permission error from rootless Podman:** run as your normal desktop user. Check `podman info`, subordinate UID/GID configuration and rootless storage. Use `crun`; keeping supplementary GPU groups requires it.
- **No window or Xauthority error:** launch from the unlocked desktop session, check `echo "$DISPLAY"` and `xauth nlist "$DISPLAY"`. XWayland must be enabled under Wayland. Avoid `xhost +`.
- **GPU/OpenGL error:** check access to `/dev/dri` and run `glxinfo -B` from the Linux desktop if available. To try Mesa software rendering, run `export PANDA_SOFTWARE_RENDERING=1` before launching. NVIDIA hosts may need a GPU setup specific to their engine and drivers.
- **Container name already in use:** stop the previous environment with the stop command for your engine, or open a shell in that container.
- **No motion:** confirm readiness, controller activation and that simulation is resumed. Review action result/error strings and logs. Do not issue overlapping commands from RViz and your node.
- **High rendering load:** disable an unused camera, reduce `camera_rate`, close RViz, or use `use_rviz:=false`. Both camera streams need a rendering context even with `headless:=true`.
- **Namespace/domain changes:** launch with `namespace:=team1` and replace `/panda` in your commands. Set `ROS_DOMAIN_ID=43` before launching to choose another domain; commands executed inside that container inherit it. A ROS domain is a discovery boundary, not an assessment security boundary.

The Podman launcher reads `PANDA_IMAGE` for the image tag and `PANDA_CONTAINER` for the container name. `workspace/` and `logs/` persist on the host with either engine. Stop the container before removing generated workspace build and install directories.

## Licensing

See [LICENSE](LICENSE), [NOTICE](NOTICE) and the retained model/RViz licenses. `STUDENT_RELEASE.json` records the checksums of the distributed repository files. The image is built from this student-only context and contains installed simulator packages without course solution scripts or instructor validation artifacts.
