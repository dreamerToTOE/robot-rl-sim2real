#!/usr/bin/env python3
"""Independent TorchScript Actor check; never connects to a robot."""

import argparse
import hashlib
import json
from pathlib import Path
import time

import torch


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--actor", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--repetitions", type=int, default=1000)
    args = parser.parse_args()
    if args.device != "cpu":
        raise ValueError("independent export check is CPU-only")
    actor = torch.jit.load(str(args.actor), map_location="cpu").eval()
    observation = torch.zeros(1, 31)
    with torch.inference_mode():
        reference = actor(observation)
        if reference.shape != (1, 6) or not torch.isfinite(reference).all() or (reference.abs() > 1 + 1e-6).any():
            raise RuntimeError("exported Actor violated the finite bounded action contract")
        start = time.perf_counter()
        for _ in range(args.repetitions):
            output = actor(observation)
        forward_ms = 1000 * (time.perf_counter() - start) / args.repetitions
    result = {
        "status": "PASS",
        "parameter_class": "SIM_ONLY",
        "actor": str(args.actor),
        "actor_sha256": digest(args.actor),
        "observation_shape": list(observation.shape),
        "action_shape": list(output.shape),
        "action_min": float(output.min()),
        "action_max": float(output.max()),
        "forward_ms": forward_ms,
        "hardware_connected": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
