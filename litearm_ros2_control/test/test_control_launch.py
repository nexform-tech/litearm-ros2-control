"""Tests for the single- and dual-arm (side namespace) launch modes."""

import ast
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "litearm_control.launch.py"


def _load_launch_module():
    spec = spec_from_file_location("litearm_control_launch", LAUNCH_FILE)
    assert spec is not None
    assert spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def launch_module():
    return _load_launch_module()


def test_single_arm_interface_remains_at_root(launch_module):
    assert launch_module._normalize_side("") is None
    assert launch_module._controller_namespace(None) is None
    assert launch_module._controller_manager_name(None) == "/controller_manager"
    assert launch_module._start_robot_state_publisher("auto", None)
    assert launch_module._isolation_env("42", "true") == {
        "ROS_DOMAIN_ID": "42",
        "ROS_LOCALHOST_ONLY": "1",
    }


@pytest.mark.parametrize("side", ["left", "right"])
def test_side_mode_uses_expected_namespace(launch_module, side):
    assert launch_module._normalize_side(side) == side
    assert launch_module._controller_namespace(side) == f"/{side}"
    assert (
        launch_module._controller_manager_name(side)
        == f"/{side}/controller_manager"
    )
    assert not launch_module._start_robot_state_publisher("auto", side)
    assert launch_module._start_robot_state_publisher("true", side)


def test_namespaced_controller_parameters(launch_module):
    source = PACKAGE_ROOT / "config" / "litearm_controllers.yaml"
    parameters = launch_module._namespaced_controller_parameters(source, "left")

    assert set(parameters) == {
        "/left/controller_manager",
        "/left/joint_trajectory_controller",
    }
    assert (
        parameters["/left/controller_manager"]["ros__parameters"]["update_rate"]
        == 100
    )
    assert parameters["/left/joint_trajectory_controller"][
        "ros__parameters"
    ]["joints"] == [f"joint{index}" for index in range(1, 8)]


def test_namespaced_controller_file_is_materialized(launch_module):
    source = PACKAGE_ROOT / "config" / "litearm_controllers.yaml"
    namespaced_path = launch_module._write_namespaced_controller_file(source, "right")
    try:
        parameters = yaml.safe_load(
            Path(namespaced_path).read_text(encoding="utf-8")
        )
        assert set(parameters) == {
            "/right/controller_manager",
            "/right/joint_trajectory_controller",
        }
    finally:
        Path(namespaced_path).unlink()


def test_invalid_side_is_rejected(launch_module):
    with pytest.raises(RuntimeError, match="side must be"):
        launch_module._normalize_side("center")


def test_invalid_start_robot_state_publisher_value_is_rejected(launch_module):
    with pytest.raises(RuntimeError, match="start_robot_state_publisher"):
        launch_module._start_robot_state_publisher("sometimes", "left")


def test_launch_arguments_preserve_the_single_arm_entrypoint(launch_module):
    from launch.actions import DeclareLaunchArgument

    description = launch_module.generate_launch_description()
    arguments = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert {
        "port",
        "controllers_file",
        "start_joint_state_broadcaster",
        "start_joint_trajectory_controller",
        "clear_faults",
        "ros_domain_id",
        "ros_localhost_only",
        "side",
        "start_robot_state_publisher",
    }.issubset(arguments)


def test_domain_defaults_are_the_workspace_wide_isolated_values(launch_module):
    assert launch_module.DEFAULT_DOMAIN_ID == "42"
    assert launch_module._isolation_env("", "true")["ROS_DOMAIN_ID"] == "42"
    assert launch_module._isolation_env("42", "true")["ROS_LOCALHOST_ONLY"] == "1"


def test_launch_source_contains_side_parameter_materialization():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    function_names = {
        node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
    }
    assert {
        "_normalize_side",
        "_controller_namespace",
        "_controller_manager_name",
        "_start_robot_state_publisher",
        "_namespaced_controller_parameters",
        "_write_namespaced_controller_file",
    } <= function_names


def test_side_launch_setup_constructs_namespaced_nodes(launch_module):
    from launch import LaunchContext
    from launch_ros.actions import Node

    context = LaunchContext()
    context.launch_configurations.update(
        {
            "side": "left",
            "start_robot_state_publisher": "auto",
            "port": "",
            "controllers_file": "",
            "start_joint_state_broadcaster": "false",
            "start_joint_trajectory_controller": "false",
            "clear_faults": "true",
            "ros_domain_id": "42",
            "ros_localhost_only": "true",
        }
    )
    actions = launch_module._launch_setup(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    assert nodes
    assert any(
        getattr(node, "_Node__node_namespace", None) == "/left" for node in nodes
    )
    # auto + side mode starts no RSP; only controller_manager remains.
    assert not any(
        getattr(node, "_Node__node_executable", None) == "robot_state_publisher"
        for node in nodes
    )
