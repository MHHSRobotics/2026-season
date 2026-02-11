"""MuJoCo simulator for FRC swerve drive robot."""

import mujoco
import mujoco.viewer
import numpy as np
from pathlib import Path

from wpimath.geometry import Pose3d, Quaternion, Rotation3d, Translation3d

from .nt_interface import SwerveInputs, SwerveOutputs


# Motor constants for voltage -> torque conversion
# Kraken X60 (FOC): stall torque 9.37 Nm, free speed 5800 RPM, stall current 483 A
# With MK4i L2 gearing (6.75:1 for drive, 150/7:1 for steer)
DRIVE_GEAR_RATIO = 6.75
STEER_GEAR_RATIO = 150.0 / 7.0  # ~21.43:1

# Kraken X60 motor constants (FOC)
MOTOR_STALL_TORQUE = 9.37  # Nm
MOTOR_FREE_SPEED = 607.4  # rad/s (5800 RPM)
MOTOR_RESISTANCE = 12.0 / 483.0  # V/A (12V / stall current)
MOTOR_STALL_CURRENT = 483.0  # A
MOTOR_CURRENT_LIMIT = 80.0   # A (TalonFX default stator current limit)

# Wheel radius for velocity conversion
WHEEL_RADIUS = 0.05  # meters (4" diameter)


def voltage_to_torque(voltage: float, angular_velocity: float,
                      gear_ratio: float) -> float:
    """Convert motor voltage to output torque using DC motor model.

    Uses the standard DC motor equation:
    torque = (V - omega * Kv) / R * Kt

    Where the motor is geared down by gear_ratio.
    """
    # Motor angular velocity (before gearbox)
    motor_omega = angular_velocity * gear_ratio

    # Back-EMF voltage
    kv = 12.0 / MOTOR_FREE_SPEED  # V/(rad/s)
    back_emf = motor_omega * kv

    # Motor torque
    kt = MOTOR_STALL_TORQUE / (12.0 / MOTOR_RESISTANCE)  # Nm/A
    current = (voltage - back_emf) / MOTOR_RESISTANCE
    motor_torque = current * kt

    # Output torque after gearbox
    output_torque = motor_torque * gear_ratio

    return output_torque


class SwerveSimulator:
    """MuJoCo-based swerve drive simulator."""

    def __init__(self, model_path: str | Path):
        """Initialize the simulator.

        Args:
            model_path: Path to the MuJoCo XML model file
        """
        self.model_path = Path(model_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)

        # Cache actuator and sensor indices
        self._cache_indices()

        # Viewer (created on demand)
        self._viewer = None
        self._voltages = None

    def _cache_indices(self) -> None:
        """Cache actuator and sensor indices for fast access."""
        # Actuator indices
        self.actuators = {
            "fl_steer": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "fl_steer_motor"),
            "fl_drive": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "fl_drive_motor"),
            "fr_steer": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "fr_steer_motor"),
            "fr_drive": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "fr_drive_motor"),
            "bl_steer": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "bl_steer_motor"),
            "bl_drive": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "bl_drive_motor"),
            "br_steer": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "br_steer_motor"),
            "br_drive": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "br_drive_motor"),
        }

        # Sensor indices (using named sensors)
        self.sensors = {}
        for name in ["fl_steer_pos", "fr_steer_pos", "bl_steer_pos", "br_steer_pos",
                     "fl_drive_pos", "fr_drive_pos", "bl_drive_pos", "br_drive_pos",
                     "fl_steer_vel", "fr_steer_vel", "bl_steer_vel", "br_steer_vel",
                     "fl_drive_vel", "fr_drive_vel", "bl_drive_vel", "br_drive_vel",
                     "chassis_gyro", "chassis_accel"]:
            self.sensors[name] = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)

        # Robot chassis freejoint qpos address
        chassis_jnt = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_joint")
        self._chassis_qpos_adr = self.model.jnt_qposadr[chassis_jnt]

        # Fuel ball freejoint qpos addresses (each freejoint has 7 qpos: xyz + quat)
        self._fuel_qpos_adrs = []
        for i in range(48):
            prefix = "blue" if i < 24 else "red"
            idx = i if i < 24 else i - 24
            jnt_name = f"{prefix}_fuel_{idx}_jnt"
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jnt_name)
            if jnt_id >= 0:
                self._fuel_qpos_adrs.append(self.model.jnt_qposadr[jnt_id])

        # Joint indices for velocity lookup
        self.joints = {
            "fl_steer": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "fl_steer"),
            "fl_drive": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "fl_drive"),
            "fr_steer": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "fr_steer"),
            "fr_drive": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "fr_drive"),
            "bl_steer": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "bl_steer"),
            "bl_drive": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "bl_drive"),
            "br_steer": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "br_steer"),
            "br_drive": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "br_drive"),
        }

    def set_voltages(self, inputs: SwerveInputs) -> None:
        """Store motor voltages for use during physics stepping."""
        self._voltages = inputs

    def _apply_motor_model(self) -> None:
        """Recompute torques from stored voltages and current joint velocities."""
        if self._voltages is None:
            return

        def get_qvel(joint_name: str) -> float:
            joint_id = self.joints[joint_name]
            return self.data.qvel[self.model.jnt_dofadr[joint_id]]

        v = self._voltages
        for prefix, module in [("fl", v.fl), ("fr", v.fr), ("bl", v.bl), ("br", v.br)]:
            steer_vel = get_qvel(f"{prefix}_steer")
            drive_vel = get_qvel(f"{prefix}_drive")
            self.data.ctrl[self.actuators[f"{prefix}_steer"]] = voltage_to_torque(
                module.steer_voltage, steer_vel, STEER_GEAR_RATIO)
            self.data.ctrl[self.actuators[f"{prefix}_drive"]] = voltage_to_torque(
                module.drive_voltage, drive_vel, DRIVE_GEAR_RATIO)

    def get_outputs(self) -> SwerveOutputs:
        """Read sensor data from simulation."""
        def get_sensor(name: str, dim: int = 1) -> float | np.ndarray:
            sensor_id = self.sensors[name]
            adr = self.model.sensor_adr[sensor_id]
            if dim == 1:
                return float(self.data.sensordata[adr])
            return self.data.sensordata[adr:adr + dim].copy()

        # Get gyro data (angular velocity in body frame)
        gyro = get_sensor("chassis_gyro", 3)

        # Robot pose from freejoint
        a = self._chassis_qpos_adr
        pos = self.data.qpos[a:a + 3]
        quat = self.data.qpos[a + 3:a + 7]  # MuJoCo quaternion: [w, x, y, z]
        pose = Pose3d(
            Translation3d(float(pos[0]), float(pos[1]), float(pos[2])),
            Rotation3d(Quaternion(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))),
        )

        # Fuel ball poses
        fuel_poses = []
        for adr in self._fuel_qpos_adrs:
            fp = self.data.qpos[adr:adr + 3]
            fq = self.data.qpos[adr + 3:adr + 7]
            fuel_poses.append(Pose3d(
                Translation3d(float(fp[0]), float(fp[1]), float(fp[2])),
                Rotation3d(Quaternion(float(fq[0]), float(fq[1]), float(fq[2]), float(fq[3]))),
            ))

        return SwerveOutputs(
            fl_steer_angle=get_sensor("fl_steer_pos"),
            fr_steer_angle=get_sensor("fr_steer_pos"),
            bl_steer_angle=get_sensor("bl_steer_pos"),
            br_steer_angle=get_sensor("br_steer_pos"),

            fl_drive_pos=get_sensor("fl_drive_pos"),
            fr_drive_pos=get_sensor("fr_drive_pos"),
            bl_drive_pos=get_sensor("bl_drive_pos"),
            br_drive_pos=get_sensor("br_drive_pos"),

            fl_steer_vel=get_sensor("fl_steer_vel"),
            fr_steer_vel=get_sensor("fr_steer_vel"),
            bl_steer_vel=get_sensor("bl_steer_vel"),
            br_steer_vel=get_sensor("br_steer_vel"),

            fl_drive_vel=get_sensor("fl_drive_vel"),
            fr_drive_vel=get_sensor("fr_drive_vel"),
            bl_drive_vel=get_sensor("bl_drive_vel"),
            br_drive_vel=get_sensor("br_drive_vel"),

            # Gyro z-axis is yaw rate in body frame
            gyro_yaw=gyro[2],
            gyro_pitch=gyro[1],
            gyro_roll=gyro[0],

            pose=pose,
            fuel_poses=fuel_poses,
        )

    def step(self, n_steps: int = 1) -> None:
        """Advance simulation by n physics steps.

        Recomputes motor torques from stored voltages each step
        so back-EMF tracks the changing joint velocities.
        """
        for _ in range(n_steps):
            self._apply_motor_model()
            mujoco.mj_step(self.model, self.data)

    def get_timestep(self) -> float:
        """Get the simulation timestep in seconds."""
        return self.model.opt.timestep

    def reset(self) -> None:
        """Reset simulation to initial state."""
        mujoco.mj_resetData(self.model, self.data)

    def launch_viewer(self) -> mujoco.viewer.Handle:
        """Launch the interactive MuJoCo viewer."""
        self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        return self._viewer

    def sync_viewer(self) -> None:
        """Sync the viewer with simulation state."""
        if self._viewer is not None:
            self._viewer.sync()

    def viewer_is_running(self) -> bool:
        """Check if viewer window is still open."""
        return self._viewer is not None and self._viewer.is_running()
