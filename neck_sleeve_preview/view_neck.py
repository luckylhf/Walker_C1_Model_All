"""Open the local neck-sleeve sample in the native MuJoCo viewer."""
from pathlib import Path
from threading import Event
import time

import mujoco
import mujoco.viewer


def main():
    path = Path(__file__).with_name("walker_c1_neck.xml")
    model = mujoco.MjModel.from_xml_path(str(path))
    model.skin("neck_sleeve")
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    pitch = data.joint("head_pitch_joint")
    yaw = data.joint("head_yaw_joint")
    up, down = model.joint("head_pitch_joint").range
    right, left = model.joint("head_yaw_joint").range
    poses = [(0, 0), (up, 0), (down, 0), (0, 0), (0, left), (0, right)]
    paused = Event()

    def on_key(key):
        if key == ord(" "):
            if paused.is_set():
                paused.clear()
            else:
                paused.set()

    with mujoco.viewer.launch_passive(
        model, data, key_callback=on_key, show_left_ui=False, show_right_ui=False
    ) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = data.body("head_pitch_link").xpos
            viewer.cam.distance = 0.57
            viewer.cam.azimuth = 140
            viewer.cam.elevation = -12
            viewer.opt.geomgroup[3] = 0
            viewer.opt.sitegroup[:] = 0
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_SKIN] = 1
        viewer.sync()
        print(f"已打开 {path.name} | MuJoCo {mujoco.__version__} | 空格暂停或继续头部动作", flush=True)
        elapsed = 0.0
        last = time.monotonic()
        while viewer.is_running():
            now = time.monotonic()
            if not paused.is_set():
                elapsed += now - last
                index, offset = divmod(elapsed, 2.5)
                step = int(index) % len(poses)
                p0, y0 = poses[step]
                p1, y1 = poses[(step + 1) % len(poses)]
                t = min(1.0, max(0.0, (offset - 0.6) / 1.7))
                blend = t * t * (3 - 2 * t)
                with viewer.lock():
                    pitch.qpos[0] = p0 + (p1 - p0) * blend
                    yaw.qpos[0] = y0 + (y1 - y0) * blend
                    mujoco.mj_forward(model, data)
            last = now
            viewer.sync()
            time.sleep(1 / 60)


if __name__ == "__main__":
    main()
