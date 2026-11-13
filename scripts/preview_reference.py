"""Math-only example with arbitrary illustrative numbers; NOT a safe robot target.

Usage: PYTHONPATH=src python3 scripts/preview_reference.py
"""
from robot_rl_sim2real.reference import CircleXY

# Arbitrary mathematical parameters. Never transmit these to hardware.
ref = CircleXY(center_m=(0.0, 0.0, 0.0), radius_m=1.0, period_s=4.0)
for t in (0, 1, 2, 3, 4):
    p = ref.sample(t)
    print(f"t={t:.1f}s  position={p.position_m}  velocity={p.velocity_mps}")
