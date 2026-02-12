"""MuJoCo simulator for FRC swerve drive robot."""

import math

import mujoco
import mujoco.viewer
import numpy as np
from pathlib import Path

from wpimath.geometry import Pose3d, Quaternion, Rotation3d, Translation3d

from .nt_interface import SwerveInputs, SwerveOutputs, SteerPIDGains, DrivePIDGains


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
    Stator current is clamped to MOTOR_CURRENT_LIMIT (TalonFX default).
    """
    # Motor angular velocity (before gearbox)
    motor_omega = angular_velocity * gear_ratio

    # Back-EMF voltage
    kv = 12.0 / MOTOR_FREE_SPEED  # V/(rad/s)
    back_emf = motor_omega * kv

    # Motor current with stator current limiting (matches TalonFX firmware)
    kt = MOTOR_STALL_TORQUE / (12.0 / MOTOR_RESISTANCE)  # Nm/A
    current = (voltage - back_emf) / MOTOR_RESISTANCE
    current = max(-MOTOR_CURRENT_LIMIT, min(MOTOR_CURRENT_LIMIT, current))
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

        # Steer PID state (one integrator per module)
        self._steer_pid_integral = {"fl": 0.0, "fr": 0.0, "bl": 0.0, "br": 0.0}
        self._steer_pid_gains = SteerPIDGains()

        # Drive velocity PID state (one integrator per module)
        self._drive_pid_integral = {"fl": 0.0, "fr": 0.0, "bl": 0.0, "br": 0.0}
        self._drive_pid_gains = DrivePIDGains()

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
        # Neutral zone fuel (24x15 grid = 360 balls)
        for i in range(360):
            jnt_name = f"neutral_fuel_{i}_jnt"
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jnt_name)
            if jnt_id >= 0:
                self._fuel_qpos_adrs.append(self.model.jnt_qposadr[jnt_id])

        # Cache fuel ball dof start addresses for velocity damping
        self._fuel_dof_starts = []
        all_fuel_jnt_ids = []
        for i in range(48):
            prefix = "blue" if i < 24 else "red"
            idx = i if i < 24 else i - 24
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}_fuel_{idx}_jnt")
            if jnt_id >= 0:
                all_fuel_jnt_ids.append(jnt_id)
        for i in range(360):
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"neutral_fuel_{i}_jnt")
            if jnt_id >= 0:
                all_fuel_jnt_ids.append(jnt_id)
        for jnt_id in all_fuel_jnt_ids:
            self._fuel_dof_starts.append(self.model.jnt_dofadr[jnt_id])

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
        """Store motor voltages and PID gains for use during physics stepping."""
        self._voltages = inputs
        if inputs.steer_pid is not None:
            self._steer_pid_gains = inputs.steer_pid
        if inputs.drive_pid is not None:
            self._drive_pid_gains = inputs.drive_pid

    def _compute_steer_voltage(self, prefix: str, setpoint: float,
                               angle: float, velocity: float, dt: float) -> float:
        """Run PID on steer angle error with proper angle wrapping.

        Returns voltage in [-12, 12].
        """
        gains = self._steer_pid_gains
        # Shortest-path angle error (wraps around ±pi)
        error = math.atan2(math.sin(setpoint - angle), math.cos(setpoint - angle))

        # Integrate with anti-windup clamp
        self._steer_pid_integral[prefix] += error * dt
        max_integral = 2.0  # prevent windup
        self._steer_pid_integral[prefix] = max(-max_integral,
            min(max_integral, self._steer_pid_integral[prefix]))

        # PID output (derivative on measurement to avoid setpoint kick)
        voltage = (gains.kP * error
                   + gains.kI * self._steer_pid_integral[prefix]
                   - gains.kD * velocity)

        return max(-12.0, min(12.0, voltage))

    def _compute_drive_voltage(self, prefix: str, setpoint: float,
                               velocity: float, dt: float) -> float:
        """Run PID + feedforward on drive velocity error.

        Args:
            setpoint: target wheel velocity in rad/s
            velocity: current wheel velocity in rad/s

        Returns voltage in [-12, 12].
        """
        gains = self._drive_pid_gains
        error = setpoint - velocity

        # Integrate with anti-windup clamp
        self._drive_pid_integral[prefix] += error * dt
        max_integral = 4.0
        self._drive_pid_integral[prefix] = max(-max_integral,
            min(max_integral, self._drive_pid_integral[prefix]))

        # Feedforward + PID
        ff = 0.0
        if setpoint != 0.0:
            ff = gains.kS * math.copysign(1.0, setpoint) + gains.kV * setpoint
        voltage = (ff
                   + gains.kP * error
                   + gains.kI * self._drive_pid_integral[prefix]
                   - gains.kD * velocity)

        return max(-12.0, min(12.0, voltage))

    def _apply_motor_model(self, dt: float) -> None:
        """Recompute torques from stored voltages and current joint velocities.

        Steer and drive torques use sim-side PID when gains are set,
        otherwise fall back to WPILib voltages directly.
        """
        if self._voltages is None:
            return

        def get_qvel(joint_name: str) -> float:
            joint_id = self.joints[joint_name]
            return self.data.qvel[self.model.jnt_dofadr[joint_id]]

        def get_qpos(joint_name: str) -> float:
            joint_id = self.joints[joint_name]
            return self.data.qpos[self.model.jnt_qposadr[joint_id]]

        v = self._voltages
        steer_gains = self._steer_pid_gains
        drive_gains = self._drive_pid_gains
        use_steer_pid = steer_gains.kP != 0.0 or steer_gains.kI != 0.0 or steer_gains.kD != 0.0
        use_drive_pid = (drive_gains.kP != 0.0 or drive_gains.kI != 0.0
                         or drive_gains.kD != 0.0 or drive_gains.kV != 0.0)

        for prefix, module in [("fl", v.fl), ("fr", v.fr), ("bl", v.bl), ("br", v.br)]:
            steer_vel = get_qvel(f"{prefix}_steer")
            drive_vel = get_qvel(f"{prefix}_drive")

            # Steer: sim-side PID or WPILib voltage
            if use_steer_pid:
                steer_angle = get_qpos(f"{prefix}_steer")
                steer_voltage = self._compute_steer_voltage(
                    prefix, module.steer_setpoint, steer_angle, steer_vel, dt)
            else:
                steer_voltage = module.steer_voltage

            # Drive: sim-side PID+FF or WPILib voltage
            if use_drive_pid:
                drive_voltage = self._compute_drive_voltage(
                    prefix, module.drive_velocity_setpoint, drive_vel, dt)
            else:
                drive_voltage = module.drive_voltage

            self.data.ctrl[self.actuators[f"{prefix}_steer"]] = voltage_to_torque(
                steer_voltage, steer_vel, STEER_GEAR_RATIO)
            self.data.ctrl[self.actuators[f"{prefix}_drive"]] = voltage_to_torque(
                drive_voltage, drive_vel, DRIVE_GEAR_RATIO)

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

        # Fuel ball poses (disabled - too expensive for 408 balls at 200Hz)
        fuel_poses = []

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
        # Damping factor per step: exponential decay approximating rolling friction
        # At 2ms timestep, 0.998 per step ≈ halving speed every ~0.69s
        fuel_damping = 0.998
        fuel_radius = 0.075
        dt = self.model.opt.timestep
        for _ in range(n_steps):
            self._apply_motor_model(dt)
            mujoco.mj_step(self.model, self.data)
            # Apply velocity damping to linear DOFs only (indices 0-2)
            # Leave angular DOFs (3-5) undamped so balls roll naturally
            for qpos_adr, dof_adr in zip(self._fuel_qpos_adrs, self._fuel_dof_starts):
                z = self.data.qpos[qpos_adr + 2]  # z position
                if z < fuel_radius + 0.01:  # within 1cm of ground contact
                    self.data.qvel[dof_adr:dof_adr + 3] *= fuel_damping

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
