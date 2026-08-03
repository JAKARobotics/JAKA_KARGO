# -*- coding: utf-8 -*-

import os
from pathlib import Path

from isaacsim import SimulationApp


GRAPH_PATH = "/ActionGraph"

CONFIG = {
    "renderer": "RayTracedLighting",
    "headless": False,
}

simulation_app = SimulationApp(CONFIG)

from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import is_stage_loading, open_stage

import omni.graph.core as og


enable_extension("isaacsim.ros2.bridge")

for _ in range(20):
    simulation_app.update()


def resolve_usd():
    path = os.environ.get("KARGO_USD_PATH", "")

    if path:
        return path

    try:
        from ament_index_python.packages import (
            get_package_share_directory,
        )

        package_share = get_package_share_directory(
            "jaka_kargo_description"
        )

        return str(
            Path(package_share)
            / "urdf"
            / "jaka_kargo"
            / "jaka_kargo_moveit.usd"
        )

    except Exception:
        return str(
            Path(__file__).resolve().parents[2]
            / "jaka_kargo_description"
            / "urdf"
            / "jaka_kargo"
            / "jaka_kargo_moveit.usd"
        )


def get_graph_attribute(attribute_path):
    attribute = og.Controller.attribute(attribute_path)

    if not attribute.is_valid():
        raise RuntimeError(
            f"OmniGraph attribute not found: {attribute_path}"
        )

    return attribute


KARGO_USD_PATH = resolve_usd()

print(
    f"Resolved USD path: {KARGO_USD_PATH}, "
    f"exists={os.path.exists(KARGO_USD_PATH)}"
)

if not os.path.exists(KARGO_USD_PATH):
    raise RuntimeError(
        f"USD not found: {KARGO_USD_PATH}"
    )

if not open_stage(KARGO_USD_PATH):
    raise RuntimeError(
        f"Failed to open USD stage: {KARGO_USD_PATH}"
    )

while is_stage_loading():
    simulation_app.update()

print(
    f"Stage opened successfully: {KARGO_USD_PATH}"
)

read_sim_time = get_graph_attribute(
    f"{GRAPH_PATH}/ReadSimTime.outputs:simulationTime"
)

publish_joint_time = get_graph_attribute(
    f"{GRAPH_PATH}/PublishJointState.inputs:timeStamp"
)

publish_clock_time = get_graph_attribute(
    f"{GRAPH_PATH}/PublishClock.inputs:timeStamp"
)

publish_joint_topic = get_graph_attribute(
    f"{GRAPH_PATH}/PublishJointState.inputs:topicName"
)

subscribe_joint_topic = get_graph_attribute(
    f"{GRAPH_PATH}/SubscribeJointState.inputs:topicName"
)

publish_clock_topic = get_graph_attribute(
    f"{GRAPH_PATH}/PublishClock.inputs:topicName"
)

publish_joint_topic.set("/isaac_joint_states")
subscribe_joint_topic.set("/isaac_joint_commands")
publish_clock_topic.set("/clock")

if not read_sim_time.is_connected(publish_joint_time):
    if not read_sim_time.connect(
        publish_joint_time,
        True,
    ):
        raise RuntimeError(
            "Failed to connect simulation time to "
            "PublishJointState"
        )

if not read_sim_time.is_connected(publish_clock_time):
    if not read_sim_time.connect(
        publish_clock_time,
        True,
    ):
        raise RuntimeError(
            "Failed to connect simulation time to PublishClock"
        )

simulation_app.update()
simulation_app.update()

simulation_context = SimulationContext(
    stage_units_in_meters=1.0
)

simulation_context.play()
simulation_app.update()

while simulation_app.is_running():
    simulation_context.step(render=True)

simulation_context.stop()
simulation_app.close()