# -*- coding: utf-8 -*-

import os
from pathlib import Path
from omni.isaac.kit import SimulationApp

KARGO_STAGE_PATH = "/World/jaka_kargo"
GRAPH_PATH = "/ActionGraph"
CONFIG = {"renderer": "RayTracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)

from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions
from omni.usd import get_context
from pxr import Usd

extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("isaacsim.ros2.urdf")
extensions.enable_extension("isaacsim.ros2.tf_viewer")
# extensions.enable_extension("isaacsim.code_editor.vscode")
# extensions.enable_extension("omni.kit.debug.vscode")

def _resolve_usd():
    p = os.environ.get("KARGO_USD_PATH", "")
    if p:
        return p
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory("jaka_kargo_description")
        return str(Path(pkg_share) / "urdf/jaka_kargo/jaka_kargo.usd")
    except Exception:
        return str(Path(__file__).resolve().parents[2] /
                   "jaka_kargo_description/urdf/jaka_kargo/jaka_kargo.usd")

KARGO_USD_PATH = _resolve_usd()

# print(f"Resolved USD path: {KARGO_USD_PATH}, exists={os.path.exists(KARGO_USD_PATH)}")

# print(f"🚀 Isaac Sim starting, loading USD: {KARGO_USD_PATH}")

# kargo_stage = get_context().open_stage(KARGO_USD_PATH)
# # if not isinstance(kargo_stage, Usd.Stage):
# #     raise RuntimeError("Failed to open stage.")

# print("Stage Opened")

# kargo_prim = get_context().get_stage().GetPrimAtPath(KARGO_STAGE_PATH)
# print("Prim found:", bool(kargo_prim) and kargo_prim.IsValid())

# if not kargo_prim or not kargo_prim.IsValid():
#     raise RuntimeError(f"Prim not found: {KARGO_STAGE_PATH}")

# # list children (quick view of links)
# print("Children names under /jaka_kargo:")
# for c in kargo_prim.GetChildren():
#     print("  -", c.GetName(), c.GetTypeName())


# sanity
print(f"Resolved USD path: {KARGO_USD_PATH}, exists={os.path.exists(KARGO_USD_PATH)}")
if not os.path.exists(KARGO_USD_PATH):
    raise RuntimeError(f"USD not found: {KARGO_USD_PATH}")

# request the stage open (do NOT use file:// prefix here)
ctx = get_context()
ctx.open_stage(KARGO_USD_PATH)    # non-blocking / asynchronous in many Kit builds


import time 

# poll until the stage is actually available and has the expected root
stage = None
basename = os.path.basename(KARGO_USD_PATH)
timeout = 10.0   # seconds
t0 = time.time()
while time.time() - t0 < timeout:
    # allow Kit to process events and background loading
    simulation_app.update()   # important: gives the app a chance to finish stage load
    stage = ctx.get_stage()
    if isinstance(stage, Usd.Stage):
        root = stage.GetRootLayer().realPath or ""
        # sometimes realPath may be empty; if so, also check GetSessionLayer etc
        if basename in root or stage.GetSessionLayer().identifier == KARGO_USD_PATH or stage.GetRootLayer().identifier == KARGO_USD_PATH:
            break
    time.sleep(0.05)

if not isinstance(stage, Usd.Stage):
    # helpful debug info before failing
    print("DEBUG: ctx.get_stage() ->", ctx.get_stage())
    raise RuntimeError(f"Stage never finished loading in {timeout}s: {KARGO_USD_PATH!r}")

print("Stage opened successfully:", stage.GetRootLayer().realPath)


simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Run app update for multiple frames to re-initialize the ROS action graph after setting new prim inputs
simulation_app.update()
simulation_app.update()

simulation_context.play()
simulation_app.update()

while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

simulation_context.stop()
simulation_app.close()