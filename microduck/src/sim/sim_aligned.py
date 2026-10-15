#!/usr/bin/env python3
"""FanGe Microduck sim aligned to this checkout's 19-DoF MJCF.

Upstream `src/sim/sim_main.py` still uses a 14-name constants table
(`right_ankle`, `neck_pitch`, …) that does not exist in `robot.xml`.
This launcher:
- drives every MJCF actuator with BAM (xl330 m6) + PyPI bam 1.0.2 kwargs
- aliases missing code names onto MJCF joints for WalkMove/head
- skips SquatMove (needs placo/pinocchio native libs)
"""
from __future__ import annotations

import os
import sys
import time
from collections import deque
from collections.abc import Callable
from pathlib import Path

ROOT = Path("/mnt/g/Microduck-build-tutorial/microduck")
os.chdir(ROOT)
sys.path.insert(0, str(ROOT / "src"))

import mujoco
import mujoco.viewer
from bam.model import load_model as bam_load_model
from bam.mujoco import MujocoController as BamController

from constants import (
    BAM_MAX_CURRENT,
    BAM_VIN,
    BAM_VIN_MIN,
    BAM_VOLTAGE_DROP_GAIN,
    ID_TO_MOTOR,
    KP_DEFAULT,
    MOTOR_TO_ID,
    NEUTRAL_POSE,
)
from moves.rotate_head import RotateHeadMove
from moves.walk import WalkMove
from sim.mujoco_input import MuJoCoInputSource

# scheduler.py imports SquatMove → placo at module load. Stub placo first.
import types

_placo = types.ModuleType("placo")
sys.modules.setdefault("placo", _placo)
from scheduler import Scheduler  # noqa: E402

# Code name -> MJCF joint/actuator name.
# The armless 14-DoF MJCF (robot_armless_14.xml) names its joints/actuators
# exactly like the constants motors (incl. the 4 head/neck joints), so only
# the ankle alias from the old 19-DoF table is still needed.
ALIAS = {
    "right_ankle": "right_ankle_pitch",
    "left_ankle": "left_ankle_pitch",
}

# Aliases needed only by the legacy 19-DoF MJCF (robot.xml): one head hinge,
# so the four head/neck code names all land on it.
ALIAS_19DOF = {
    "neck_pitch": "head",
    "head_pitch": "head",
    "head_yaw": "head",
    "head_roll": "head",
    "head": "head",
}


def mjcf_name(code_name: str, legacy_19dof: bool = False) -> str:
    if legacy_19dof and code_name in ALIAS_19DOF:
        return ALIAS_19DOF[code_name]
    return ALIAS.get(code_name, code_name)


class _DelayBuffer:
    def __init__(self, initial, n_steps: int) -> None:
        size = max(1, n_steps + 1)
        self._buf: deque = deque([initial] * size, maxlen=size)

    def push_and_read(self, value):
        self._buf.appendleft(value)
        return self._buf[-1]

    def fill(self, value) -> None:
        for i in range(len(self._buf)):
            self._buf[i] = value


class _StubViewer:
    """Headless stand-in for mujoco.viewer.launch_passive (DUCK_SIM_VIEWER=0).

    Lets the aligned controller run without WSLg — used by the automated
    stand-test (scripts/verify_armless_14.py) and CI-style checks.
    """

    def __init__(self, model, data) -> None:
        self.opt = mujoco.MjvOption()
        self._model = model
        self._data = data

    def is_running(self) -> bool:
        return True

    def sync(self) -> None:
        return


class AlignedMuJoCoController:
    def __init__(
        self,
        mjcf_path: str,
        key_callback: Callable | None = None,
        stop_flag_path: str = "/tmp/microduck_scheduler.stop",
        reset_source: MuJoCoInputSource | None = None,
        delay_act_steps: int = 2,
        delay_pos_ticks: int = 0,
        delay_vel_ticks: int = 1,
        delay_gyro_ticks: int = 3,
        delay_quat_ticks: int = 4,
        legacy_19dof: bool = False,
    ) -> None:
        self._stop_flag_path = Path(stop_flag_path)
        self._reset_source = reset_source
        self._legacy_19dof = legacy_19dof
        self._model = mujoco.MjModel.from_xml_path(mjcf_path)
        self._data = mujoco.MjData(self._model)

        self._name_to_actuator_idx: dict[str, int] = {}
        self._name_to_qpos_idx: dict[str, int] = {}
        self._name_to_qvel_idx: dict[str, int] = {}
        missing = []
        for name in MOTOR_TO_ID:
            mj = mjcf_name(name, legacy_19dof=legacy_19dof)
            actuator_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, mj)
            joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, mj)
            if actuator_id < 0 or joint_id < 0:
                missing.append(f"{name}->{mj}")
                continue
            self._name_to_actuator_idx[name] = actuator_id
            self._name_to_qpos_idx[name] = int(self._model.jnt_qposadr[joint_id])
            self._name_to_qvel_idx[name] = int(self._model.jnt_dofadr[joint_id])
        if missing:
            raise ValueError(f"MJCF missing mapped actuators/joints: {missing}")

        # Also index every MJCF actuator so BAM can drive the extra arm DoFs
        self._mj_actuator_names: list[str] = []
        for i in range(self._model.nu):
            n = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if n:
                self._mj_actuator_names.append(n)

        self._steps_per_tick = max(1, round(0.02 / self._model.opt.timestep))
        self._torque_interval = 0.1
        self._last_torque_print = 0.0

        self._apply_neutral()
        mujoco.mj_forward(self._model, self._data)

        self._delay_pos = {
            mid: _DelayBuffer(self._data.qpos[self._name_to_qpos_idx[ID_TO_MOTOR[mid]]], delay_pos_ticks)
            for mid in MOTOR_TO_ID.values()
        }
        self._delay_vel = {mid: _DelayBuffer(0.0, delay_vel_ticks) for mid in MOTOR_TO_ID.values()}
        self._delay_gyro = _DelayBuffer((0.0, 0.0, 0.0), delay_gyro_ticks)
        self._delay_quat = _DelayBuffer((1.0, 0.0, 0.0, 0.0), delay_quat_ticks)
        self._delay_act = {
            mid: _DelayBuffer(self._data.qpos[self._name_to_qpos_idx[ID_TO_MOTOR[mid]]], delay_act_steps)
            for mid in MOTOR_TO_ID.values()
        }

        bam_model = bam_load_model(motor_name="xl330", model="m6")
        bam_model.actuator.kp = KP_DEFAULT
        bam_model.actuator.vin = BAM_VIN
        # PyPI better-actuator-models 1.0.2: vin_drop_resistance, no max_current kwarg
        drop_r = BAM_VOLTAGE_DROP_GAIN if BAM_VOLTAGE_DROP_GAIN else None
        self._bam = BamController(
            bam_model,
            self._mj_actuator_names,
            self._model,
            self._data,
            vin_drop_resistance=drop_r,
            vin_min=BAM_VIN_MIN,
        )
        # PyPI bam 1.0.2 has no reset(); hold current qpos as targets
        for name in self._mj_actuator_names:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self._bam.set_q_target(name, float(self._data.qpos[int(self._model.jnt_qposadr[jid])]))
        self._max_current = BAM_MAX_CURRENT

        print("MJCF nq", self._model.nq, "nu", self._model.nu, "dt", self._model.opt.timestep)
        print("mapped code motors", list(self._name_to_actuator_idx))
        print("bam actuators", self._mj_actuator_names)

        if os.environ.get("DUCK_SIM_VIEWER", "1") == "0":
            self._viewer = _StubViewer(self._model, self._data)
        else:
            self._viewer = mujoco.viewer.launch_passive(self._model, self._data, key_callback=key_callback)

    def _apply_neutral(self) -> None:
        self._data.qpos[:] = 0.0
        self._data.qvel[:] = 0.0
        self._data.ctrl[:] = 0.0
        self._data.qpos[2] = 0.165
        for name, angle in NEUTRAL_POSE.items():
            if name in self._name_to_qpos_idx:
                self._data.qpos[self._name_to_qpos_idx[name]] = angle

    @property
    def viewer_opt(self):
        return self._viewer.opt

    def set_kp(self, kp: float) -> None:
        self._bam.model.actuator.kp = kp

    def sync_read_kp(self, ids: list[int]) -> list[int]:
        kp = int(self._bam.model.actuator.kp)
        return [kp] * len(ids)

    def sync_write_kp(self, ids: list[int], gains: list[int]) -> None:
        self._bam.model.actuator.kp = gains[0]

    def sync_write_torque_enable(self, ids: list[int], values: list[bool]) -> None:
        return

    def sync_write_status_return_level(self, ids: list[int], levels: list[int]) -> None:
        return

    def sync_write_goal_position(self, ids: list[int], positions: list[float]) -> None:
        if not self._viewer.is_running():
            self._stop_flag_path.write_text("stop\n", encoding="ascii")
            return

        cmd = dict(zip(ids, positions))
        for _ in range(self._steps_per_tick):
            for motor_id, pos in cmd.items():
                delayed_pos = self._delay_act[motor_id].push_and_read(pos)
                code = ID_TO_MOTOR[motor_id]
                self._bam.set_q_target(mjcf_name(code, legacy_19dof=self._legacy_19dof), delayed_pos)
            self._bam.update()
            # clip current if bam exposes torque in ctrl
            kt = getattr(self._bam.model, "kt", None)
            kt_v = getattr(kt, "value", None) if kt is not None else None
            if kt_v:
                cap = self._max_current * float(kt_v)
                self._data.ctrl[:] = [
                    max(-cap, min(cap, float(x))) for x in self._data.ctrl
                ]
            mujoco.mj_step(self._model, self._data)

        if self._reset_source is not None and self._reset_source.consume_reset():
            self.reset()
            return

        if self._reset_source is not None and self._reset_source.show_torque:
            now = time.monotonic()
            if now - self._last_torque_print >= self._torque_interval:
                total = float(sum(abs(f) for f in self._data.actuator_force))
                print(f"Torque sum: {total:.3f} Nm", flush=True)
                self._last_torque_print = now

        self._viewer.sync()

    def sync_read_present_position(self, ids: list[int]) -> list[float]:
        return [
            self._delay_pos[mid].push_and_read(self._data.qpos[self._name_to_qpos_idx[ID_TO_MOTOR[mid]]])
            for mid in ids
        ]

    def read_present_position(self, motor_id: int) -> float:
        name = ID_TO_MOTOR[motor_id]
        return float(
            self._delay_pos[motor_id].push_and_read(self._data.qpos[self._name_to_qpos_idx[name]])
        )

    def sync_read_present_velocity(self, ids: list[int]) -> list[float]:
        return [
            self._delay_vel[mid].push_and_read(self._data.qvel[self._name_to_qvel_idx[ID_TO_MOTOR[mid]]])
            for mid in ids
        ]

    def read_present_velocity(self, motor_id: int) -> float:
        name = ID_TO_MOTOR[motor_id]
        return float(
            self._delay_vel[motor_id].push_and_read(self._data.qvel[self._name_to_qvel_idx[name]])
        )

    def sync_read_present_current(self, ids: list[int]) -> list[float]:
        kt = getattr(self._bam.model, "kt", None)
        kt_v = float(getattr(kt, "value", 0.366) or 0.366)
        return [
            float(self._data.ctrl[self._name_to_actuator_idx[ID_TO_MOTOR[mid]]] / kt_v)
            for mid in ids
        ]

    def sync_read_present_input_voltage(self, ids: list[int]) -> list[float]:
        return [80.0] * len(ids)

    def read_present_input_voltage(self, motor_id: int) -> float:
        return 80.0

    def read_acc(self) -> tuple[float, float, float]:
        sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, "orientation")
        if sid < 0:
            return 0.0, 0.0, -1.0
        adr = self._model.sensor_adr[sid]
        w, x, y, z = self._data.sensordata[adr : adr + 4]
        gx = 2 * (x * z - w * y)
        gy = 2 * (y * z + w * x)
        gz = w * w - x * x - y * y + z * z
        return float(gx), float(gy), float(-gz)

    def read_gyro(self) -> tuple[float, float, float]:
        sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, "angular-velocity")
        if sid < 0:
            current = (0.0, 0.0, 0.0)
        else:
            adr = self._model.sensor_adr[sid]
            gx, gy, gz = self._data.sensordata[adr : adr + 3]
            current = (float(gx), float(gy), float(gz))
        return self._delay_gyro.push_and_read(current)

    def read_quat(self, dt: float) -> tuple[float, float, float, float]:
        sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, "orientation")
        if sid < 0:
            current = (1.0, 0.0, 0.0, 0.0)
        else:
            adr = self._model.sensor_adr[sid]
            w, x, y, z = self._data.sensordata[adr : adr + 4]
            current = (float(w), float(x), float(y), float(z))
        return self._delay_quat.push_and_read(current)

    def reset(self) -> None:
        self._apply_neutral()
        mujoco.mj_forward(self._model, self._data)
        for name in self._mj_actuator_names:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self._bam.set_q_target(name, float(self._data.qpos[int(self._model.jnt_qposadr[jid])]))
        for mid in MOTOR_TO_ID.values():
            self._delay_act[mid].fill(self._data.qpos[self._name_to_qpos_idx[ID_TO_MOTOR[mid]]])
        self._viewer.sync()


def main() -> None:
    input_source = MuJoCoInputSource(move_keys={"h": "head", "v": "walk"})
    controller = AlignedMuJoCoController(
        # Armless 14-DoF MJCF (ticket #3); override with MJCF_ALIGNED=<path> to
        # load e.g. the legacy 19-DoF robot.xml for comparison runs.
        mjcf_path=os.environ.get(
            "MJCF_ALIGNED", "src/model/mjcf/scene_armless_14.xml"
        ),
        key_callback=input_source.key_callback,
        reset_source=input_source,
    )
    input_source.set_viewer_opt(controller.viewer_opt)
    print("Walk ONNX:", (ROOT / "src/agents/walk.onnx").exists())
    print("Keys in MuJoCo window: [v] walk  [h] head  arrows vx/vtheta  [x] zero  [r] reset  [q] quit")
    scheduler = Scheduler(
        frequency_hz=50.0,
        controller=controller,
        input_source=input_source,
        moves={
            "head": RotateHeadMove(),
            "walk": WalkMove(controller=controller),
        },
    )
    scheduler.run()


if __name__ == "__main__":
    main()
