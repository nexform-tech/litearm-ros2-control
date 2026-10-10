# litearm

The URDF description of the 7-DOF litearm, and a display launch to look at it. For maintainers of
this repository, and for anyone about to edit the model — read the warning first.

## Do not edit the model here

`urdf/litearm.urdf` and `meshes/` are a copy of the litearm-stm32 firmware repository, where the same
file is the single source of truth: the firmware's kinematics table is generated from it, so its
geometry and its joint limits are what the arm actually does. Editing the copy here makes the
description disagree with the firmware, and the disagreement shows up as an arm that plans fine and
executes wrong.

Change the model upstream and re-sync this directory. Nothing here is hand-written.

## The names are a contract

The rest of the stack reads these names, not the file layout:

| Element | Names |
| --- | --- |
| Package | `litearm` — what `$(find litearm)` in [litearm.urdf.xacro](../litearm_ros2_control/urdf/litearm.urdf.xacro) and `get_package_share_directory("litearm")` in the MoveIt config resolve |
| URDF path | `urdf/litearm.urdf` |
| Links | `base_link`, `joint1`..`joint7`, `ee_link` |
| Joints | `joint1`..`joint7` (revolute), plus the fixed `ee_link` |

Renaming any of them breaks the `joint1..jointN` mapping the hardware component relies on, and the
`base_link` → `ee_link` chain the MoveIt configuration plans in.

## Contents

| Path | What it is |
| --- | --- |
| `urdf/litearm.urdf` | The model. Links, joints, inertial and collision geometry, joint limits. |
| `urdf/litearm.csv` | The SolidWorks export table the URDF came from. Provenance, not read at runtime. |
| `meshes/*.STL` | Visual and collision meshes, 61 MB across nine files, referenced as `package://litearm/meshes/*.STL`. |
| `config/joint_names_litearm.yaml` | The joint list, for tools that want it without parsing the URDF. |
| `rviz/display.rviz` | RViz layout for the display launch. |
| `launch/display.launch.py` | `robot_state_publisher` + `joint_state_publisher_gui` + RViz. A way to look at the model with no hardware and no controller manager. |

```bash
ros2 launch litearm display.launch.py
```

That launch brings up the model alone. To drive a real arm, use the ros2_control stack instead —
see [../docs/quickstart.md](../docs/quickstart.md).

The vendor's Gazebo launch and its `gazebo_ros` dependency are deliberately not carried here: this
repository is a real-hardware control stack, and keeping them would make `rosdep install` pull a
simulator into it.
