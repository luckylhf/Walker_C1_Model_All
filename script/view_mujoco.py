#!/usr/bin/env python3
"""Start MuJoCo paused at the legal home keyframe; on macOS use mjpython."""
import argparse
from pathlib import Path
import time
import mujoco
import mujoco.viewer

p = argparse.ArgumentParser()
p.add_argument('--hands', action='store_true')
p.add_argument('--simulate', action='store_true', help='Run passive gravity simulation, without a standing controller')
args = p.parse_args()
stem = 'walker_c1_with_hands' if args.hands else 'walker_c1'
root = Path(__file__).resolve().parents[1]
m = mujoco.MjModel.from_xml_path(str(root / 'mujoco' / f'{stem}.xml'))
d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0)
mujoco.mj_forward(m, d)
print(f'已加载模型：{stem}；正在创建查看窗口…', flush=True)
with mujoco.viewer.launch_passive(m, d) as viewer:
    print('MuJoCo 查看窗口已打开。' + ('当前运行无控制被动仿真。' if args.simulate else '当前暂停，仅查看模型。'), flush=True)
    viewer.cam.lookat[:] = [0, 0, 0.9]
    viewer.cam.distance = 2.8
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -15
    viewer.opt.geomgroup[3] = 0
    while viewer.is_running():
        start = time.monotonic()
        if args.simulate:
            mujoco.mj_step(m, d)
        viewer.sync()
        time.sleep(max(0, (m.opt.timestep if args.simulate else 1 / 30) - (time.monotonic() - start)))
