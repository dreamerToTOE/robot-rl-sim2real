"""Bounded empty-stage startup check: no robot, physics steps, policy or SDK."""
import argparse
import json
from pathlib import Path
import sys


def main():
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, required=True)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if not args.headless or args.device != "cpu":
        parser.error("This check requires --headless --device cpu")
    args.summary.unlink(missing_ok=True)
    launcher = AppLauncher(args)
    app = launcher.app
    try:
        import isaaclab
        from isaaclab.sim import SimulationCfg, SimulationContext
        from isaaclab.sim.converters import UrdfConverter
        import omni.usd
        import torch

        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        stage.DefinePrim("/B0R1StaticProbe", "Xform")
        for _ in range(3):
            app.update()
        assert app.is_running(), "Kit stopped during the empty-stage check"
        assert stage.GetPrimAtPath("/B0R1StaticProbe").IsValid()
        assert SimulationCfg(device="cpu").device == "cpu"
        assert SimulationContext is not None and UrdfConverter is not None
        summary = {
            "status": "PASS",
            "python": sys.version,
            "isaaclab_module": isaaclab.__file__,
            "torch": torch.__version__,
            "kit_updates": 3,
            "physics_steps": 0,
            "robot_loaded": False,
            "scope": "empty Kit stage and Lab simulation/converter API imports only",
        }
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(json.dumps(summary, indent=2) + "\n")
        print("B0_R1_SMOKE=" + json.dumps(summary), flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
