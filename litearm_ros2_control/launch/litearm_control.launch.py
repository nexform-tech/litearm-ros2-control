#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_control.launch.py — the direct-USB control stack.

Starts the three things a controller_manager deployment needs and nothing else:

    robot_state_publisher        TF from the description package's URDF
    ros2_control_node            controller_manager + the LitearmSystem component
    spawner                      joint_state_broadcaster, then joint_trajectory_controller

There is no helper process and no shared memory: the component owns the USB CDC
link itself, so this one launch is the whole stack.

    # real arm (board powered, USB connected, licence activated)
    ros2 launch litearm_ros2_control litearm_control.launch.py

    # a different USB device, or an explicit CAN-free bench unit
    ros2 launch litearm_ros2_control litearm_control.launch.py port:=/dev/ttyACM1

    # let a latched fault stop the launch instead of clearing it (after a crash, say)
    ros2 launch litearm_ros2_control litearm_control.launch.py clear_faults:=false

    # one side of a dual-arm deployment; start the other side separately
    ros2 launch litearm_ros2_control litearm_control.launch.py \
        side:=left port:=/dev/serial/by-id/<left-arm>

side:=left|right puts controller_manager, JSB and JTC under /<side>; parameter
keys are rewritten to /<side>/controller_manager and /<side>/joint_trajectory_controller.
robot_state_publisher defaults to off in side mode, because a dual-arm deployment
publishes TF once from the combined model.

By default this clears a latched fault on the arm while configuring it. The firmware
latches an EMERGENCY or a joint_fault and then answers ENABLE with an error until it is
reset, which made every bring-up after a fault a two-step operation. The reset happens
with the link up and nothing enabled or streamed; if the cause is still present the
firmware latches the fault again and ENABLE fails with a message that says so.

The MoveIt stack includes this file (litearm_moveit_config/litearm_moveit.launch.py),
which is why the ROS isolation arguments exist: they have to reach these nodes too,
or move_group cannot see the controller_manager actions.

⚠ Do not give ros2_control_node an explicit name='controller_manager'. launch_ros
  turns that into a `-r __node:=controller_manager` remapping, the controller_manager
  passes it on to every controller node it creates, and the per-controller parameter
  sections in the YAML then no longer match -- the symptom is "'joints' parameter was
  empty" and a controller that will not configure.
"""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_DOMAIN_ID = "42"
"""Deliberately not 0: domain 0 is the one other equipment on the network uses."""

VALID_SIDES = ("left", "right")


def _parse_bool(value, fallback=False):
    if value is None:
        return fallback
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off", ""):
        return False
    return fallback


def _isolation_env(domain_id, localhost_only):
    """The environment every node of this stack gets: fixed domain, localhost only."""
    return {
        "ROS_DOMAIN_ID": str(domain_id).strip() or DEFAULT_DOMAIN_ID,
        "ROS_LOCALHOST_ONLY": "1" if _parse_bool(localhost_only, True) else "0",
    }


def _isolation_hint(domain_id, localhost_only):
    domain = str(domain_id).strip() or DEFAULT_DOMAIN_ID
    if _parse_bool(localhost_only, True):
        return f"export ROS_DOMAIN_ID={domain}; export ROS_LOCALHOST_ONLY=1"
    return f"export ROS_DOMAIN_ID={domain}    (localhost-only is off: expect cross-machine traffic)"


def _normalize_side(value):
    """Return None for the single-arm root, or left/right for a dual-arm instance."""
    side = str(value).strip().lower()
    if not side:
        return None
    if side not in VALID_SIDES:
        raise RuntimeError(
            f"side must be empty, 'left', or 'right'; got '{value}'"
        )
    return side


def _controller_namespace(side):
    return f"/{side}" if side else None


def _controller_manager_name(side):
    return f"/{side}/controller_manager" if side else "/controller_manager"


def _start_robot_state_publisher(value, side):
    """auto starts RSP only at the single-arm root; a dual-arm stack publishes TF once."""
    setting = str(value).strip().lower()
    if setting == "auto":
        return side is None
    if setting in ("1", "true", "yes", "on"):
        return True
    if setting in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(
        "start_robot_state_publisher must be auto, true, or false"
    )


def _namespaced_controller_parameters(path, side):
    """Qualify the controller YAML node keys with the side namespace.

    The source file uses ``controller_manager`` / ``joint_trajectory_controller``.
    In side mode the controller_manager runs under ``/<side>`` and creates its
    controller nodes in that namespace, so the parameter keys must be
    ``/<side>/controller_manager`` and so on. When they do not match, controller
    configuration fails with an empty ``'joints' parameter``.
    """
    with open(path, "r", encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream) or {}
    if not isinstance(parameters, dict) or not parameters:
        raise RuntimeError(f"controller parameter file is empty or invalid: {path}")

    namespaced = {}
    for node_name, node_parameters in parameters.items():
        if not isinstance(node_name, str) or not node_name:
            raise RuntimeError(
                f"controller parameter file has an invalid node key: {path}"
            )
        if node_name.startswith("/") or "*" in node_name:
            raise RuntimeError(
                "side mode requires controller parameter keys without a "
                f"namespace or wildcard; got '{node_name}' in {path}"
            )
        if (
            not isinstance(node_parameters, dict)
            or "ros__parameters" not in node_parameters
        ):
            raise RuntimeError(
                f"node '{node_name}' has no ros__parameters section in {path}"
            )
        namespaced[f"/{side}/{node_name}"] = node_parameters
    return namespaced


def _write_namespaced_controller_file(path, side):
    """Materialize a side-specific controller parameter file and return its path.

    The file lives in the system temporary directory and is not removed when the
    launch exits. It contains parameters only, no hardware state, and is
    regenerated on every start.
    """
    parameters = _namespaced_controller_parameters(path, side)
    descriptor, namespaced_path = tempfile.mkstemp(
        prefix=f"litearm_{side}_controllers_",
        suffix=".yaml",
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                parameters,
                stream,
                default_flow_style=False,
                sort_keys=False,
            )
    except (OSError, yaml.YAMLError):
        os.unlink(namespaced_path)
        raise
    return namespaced_path


def _declare_arguments():
    return [
        DeclareLaunchArgument(
            "side", default_value="", choices=["", *VALID_SIDES],
            description="Dual-arm side namespace. Empty keeps the original "
                        "single-arm root interface; left/right start this stack "
                        "under /<side> for a shared dual-arm deployment"),
        DeclareLaunchArgument(
            "start_robot_state_publisher", default_value="auto",
            choices=["auto", "true", "false"],
            description="Start robot_state_publisher. auto starts it only when "
                        "side is empty; a dual-arm deployment must publish TF "
                        "once from the combined model"),
        DeclareLaunchArgument(
            "port", default_value="",
            description="USB CDC device path of the arm; empty = auto-discovery "
                        "by VID:PID 1d50:606f"),
        DeclareLaunchArgument(
            "controllers_file", default_value="",
            description="Controller parameter file; empty = the one shipped in "
                        "this package (config/litearm_controllers.yaml)"),
        DeclareLaunchArgument(
            "start_joint_state_broadcaster", default_value="true",
            description="Spawn and activate joint_state_broadcaster"),
        DeclareLaunchArgument(
            "start_joint_trajectory_controller", default_value="true",
            description="Spawn and activate joint_trajectory_controller"),
        DeclareLaunchArgument(
            "clear_faults", default_value="true",
            description="Clear a latched arm fault (EMERGENCY / joint_fault) while "
                        "configuring the hardware. The firmware latches a fault and then "
                        "refuses ENABLE until a reset, so without this a bring-up after one "
                        "needs a separate litearm_driver session first. Pass false to let a "
                        "latched fault stop the launch instead."),
        DeclareLaunchArgument(
            "ros_domain_id", default_value=DEFAULT_DOMAIN_ID,
            description="ROS domain of this stack; keep it unless you are "
                        "integrating with another system"),
        DeclareLaunchArgument(
            "ros_localhost_only", default_value="true",
            description="true = localhost discovery only, which keeps another "
                        "robot on the network out of /joint_states and /move_action"),
    ]


def _spawner(name, manager, env, namespace=None):
    # The isolation environment has to reach the spawner too: without it the spawner looks for
    # /controller_manager in the default domain while the manager sits in this stack's domain,
    # and it waits forever on "service not available".
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--controller-manager", manager],
        output="screen",
        additional_env=env,
        namespace=namespace,
        emulate_tty=True,
    )


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    side = _normalize_side(resolve("side"))
    namespace = _controller_namespace(side)
    manager_name = _controller_manager_name(side)
    start_robot_state_publisher = _start_robot_state_publisher(
        resolve("start_robot_state_publisher"), side
    )

    port = resolve("port").strip()
    controllers_file = resolve("controllers_file").strip()
    start_joint_state_broadcaster = _parse_bool(resolve("start_joint_state_broadcaster"), True)
    start_joint_trajectory_controller = _parse_bool(
        resolve("start_joint_trajectory_controller"), True)
    clear_faults = "true" if _parse_bool(resolve("clear_faults"), True) else "false"
    domain_id = resolve("ros_domain_id").strip() or DEFAULT_DOMAIN_ID
    localhost_only = resolve("ros_localhost_only")
    env = _isolation_env(domain_id, localhost_only)

    share = get_package_share_directory("litearm_ros2_control")
    xacro_file = os.path.join(share, "urdf", "litearm.urdf.xacro")
    if not controllers_file:
        controllers_file = os.path.join(share, "config", "litearm_controllers.yaml")
    source_controllers_file = controllers_file
    if side:
        controllers_file = _write_namespaced_controller_file(
            source_controllers_file, side
        )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file,
                 " litearm_port:=", port,
                 " litearm_reset_faults_on_configure:=", clear_faults]), value_type=str)

    actions = []
    if start_robot_state_publisher:
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                additional_env=env,
                namespace=namespace,
                parameters=[{"robot_description": robot_description}],
            )
        )
    actions.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            additional_env=env,
            namespace=namespace,
            parameters=[{"robot_description": robot_description}, controllers_file],
        )
    )

    broadcaster = None
    if start_joint_state_broadcaster:
        broadcaster = _spawner(
            "joint_state_broadcaster", manager_name, env, namespace
        )
        actions.append(broadcaster)

    if start_joint_trajectory_controller:
        trajectory = _spawner(
            "joint_trajectory_controller", manager_name, env, namespace
        )
        if broadcaster is not None:
            # The trajectory controller configures against joint states, so it
            # starts only after the broadcaster has been activated and exited.
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=broadcaster, on_exit=[trajectory])))
        else:
            actions.append(trajectory)

    side_note = ""
    if side:
        side_note = (
            f"  side namespace: {namespace}\n"
            f"  controller manager: {manager_name}\n"
            "  ⚠ Start the other side as a separate launch, with a different "
            "USB port.\n"
            "  ⚠ Keep robot_state_publisher on exactly one side (or on the "
            "combined dual-arm model);\n"
            "    two side RSPs publish duplicate base_link/joint TF frames.\n"
        )
    actions.insert(0, LogInfo(msg=(
        "──────── litearm control stack (direct USB) ────────\n"
        + side_note
        + f"  port: {port or '(auto-discovery, VID:PID 1d50:606f)'}\n"
        f"  clear a latched fault at configure: {clear_faults}\n"
        f"  controllers: {source_controllers_file}\n"
        f"  runtime controller parameters: {controllers_file}\n"
        f"  robot_state_publisher: "
        f"{'started' if start_robot_state_publisher else 'not started'}\n"
        "  ⚠ This stack owns the USB port: stop any other one before starting it.\n"
        "  ⚠ It also fixes the ROS domain; in your own terminal run\n"
        f"        {_isolation_hint(domain_id, localhost_only)}\n"
        "    or ros2 commands will not see this stack.\n"
        "────────────────────────────────────────────────────")))
    return actions


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
