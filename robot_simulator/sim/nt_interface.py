"""NetworkTables interface for communicating with WPILib simulation."""

import ntcore
from dataclasses import dataclass, field
from typing import List
from wpimath.geometry import Pose3d, Quaternion, Rotation3d, Translation3d


@dataclass
class SwerveModuleState:
    """State for a single swerve module."""
    steer_voltage: float = 0.0
    drive_voltage: float = 0.0


@dataclass
class SwerveInputs:
    """All motor voltage inputs from WPILib."""
    fl: SwerveModuleState
    fr: SwerveModuleState
    bl: SwerveModuleState
    br: SwerveModuleState


@dataclass
class SwerveOutputs:
    """All sensor outputs to send back to WPILib."""
    # Steering angles (radians)
    fl_steer_angle: float = 0.0
    fr_steer_angle: float = 0.0
    bl_steer_angle: float = 0.0
    br_steer_angle: float = 0.0

    # Drive positions (radians)
    fl_drive_pos: float = 0.0
    fr_drive_pos: float = 0.0
    bl_drive_pos: float = 0.0
    br_drive_pos: float = 0.0

    # Steer velocities (rad/s)
    fl_steer_vel: float = 0.0
    fr_steer_vel: float = 0.0
    bl_steer_vel: float = 0.0
    br_steer_vel: float = 0.0

    # Drive velocities (rad/s)
    fl_drive_vel: float = 0.0
    fr_drive_vel: float = 0.0
    bl_drive_vel: float = 0.0
    br_drive_vel: float = 0.0

    # IMU data
    gyro_yaw: float = 0.0
    gyro_pitch: float = 0.0
    gyro_roll: float = 0.0

    # Robot pose
    pose: Pose3d = None

    # Fuel ball poses
    fuel_poses: List[Pose3d] = field(default_factory=list)


# AdvantageKit NT path names for each module
_MODULE_NAMES = {
    "fl": "FrontLeft",
    "fr": "FrontRight",
    "bl": "BackLeft",
    "br": "BackRight",
}


class NetworkTablesInterface:
    """Handles NetworkTables communication with WPILib simulation."""

    def __init__(self, server: str = "localhost", port: int = 5810):
        """Initialize NetworkTables connection.

        Args:
            server: NetworkTables server address (usually localhost for sim)
            port: NetworkTables port (5810 is WPILib sim default)
        """
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.setServer(server, port)
        self.inst.startClient4("MuJoCo Simulator")

        # Publish/subscribe at 20ms to match WPILib loop rate
        pub_opts = ntcore.PubSubOptions(periodic=0.02)

        swerve = self.inst.getTable("AdvantageKit/Swerve")

        # Input subscribers (voltages from WPILib)
        # Path: AdvantageKit/Swerve/{Module}/Drive/AppliedVoltage
        # Path: AdvantageKit/Swerve/{Module}/Steer/AppliedVoltage
        self._drive_voltage = {}
        self._steer_voltage = {}
        for key, name in _MODULE_NAMES.items():
            mod = swerve.getSubTable(name)
            self._drive_voltage[key] = mod.getDoubleTopic("Drive/AppliedVoltage").subscribe(0.0, pub_opts)
            self._steer_voltage[key] = mod.getDoubleTopic("Steer/AppliedVoltage").subscribe(0.0, pub_opts)

        # Output publishers (sensor data to WPILib)
        # Path: MuJoCo/Swerve/{Module}/Drive/Position
        # Path: MuJoCo/Swerve/{Module}/Drive/Velocity
        # Path: MuJoCo/Swerve/{Module}/Steer/Angle
        sim_swerve = self.inst.getTable("MuJoCo/Swerve")
        self._drive_pos = {}
        self._drive_vel = {}
        self._steer_angle = {}
        self._steer_vel = {}
        for key, name in _MODULE_NAMES.items():
            mod = sim_swerve.getSubTable(name)
            self._drive_pos[key] = mod.getDoubleTopic("Drive/Position").publish(pub_opts)
            self._drive_vel[key] = mod.getDoubleTopic("Drive/Velocity").publish(pub_opts)
            self._steer_angle[key] = mod.getDoubleTopic("Steer/Angle").publish(pub_opts)
            self._steer_vel[key] = mod.getDoubleTopic("Steer/Velocity").publish(pub_opts)

        # IMU outputs
        imu = self.inst.getTable("MuJoCo/Gyro")
        self._gyro_yaw = imu.getDoubleTopic("Yaw").publish(pub_opts)
        self._gyro_pitch = imu.getDoubleTopic("Pitch").publish(pub_opts)
        self._gyro_roll = imu.getDoubleTopic("Roll").publish(pub_opts)

        # Robot pose output (struct-encoded Pose3d)
        self._pose = self.inst.getStructTopic("MuJoCo/Swerve/Pose", Pose3d).publish(pub_opts)

        # Fuel ball poses (struct array of Pose3d) - slower rate to reduce AScope load
        fuel_opts = ntcore.PubSubOptions(periodic=0.05)
        self._fuel_poses = self.inst.getStructArrayTopic("MuJoCo/Fuel", Pose3d).publish(fuel_opts)

        # Debug: tick counter to verify publish rate in AdvantageScope
        # self._tick = self.inst.getTable("MuJoCo").getDoubleTopic("Tick").publish(pub_opts)
        # self._tick_count = 0

    def get_inputs(self) -> SwerveInputs:
        """Read motor voltages from NetworkTables."""
        def module(key: str) -> SwerveModuleState:
            return SwerveModuleState(
                steer_voltage=self._steer_voltage[key].get(),
                drive_voltage=self._drive_voltage[key].get(),
            )

        return SwerveInputs(
            fl=module("fl"),
            fr=module("fr"),
            bl=module("bl"),
            br=module("br"),
        )

    def set_outputs(self, outputs: SwerveOutputs) -> None:
        """Write sensor data to NetworkTables."""
        self._steer_angle["fl"].set(outputs.fl_steer_angle)
        self._steer_angle["fr"].set(outputs.fr_steer_angle)
        self._steer_angle["bl"].set(outputs.bl_steer_angle)
        self._steer_angle["br"].set(outputs.br_steer_angle)

        self._steer_vel["fl"].set(outputs.fl_steer_vel)
        self._steer_vel["fr"].set(outputs.fr_steer_vel)
        self._steer_vel["bl"].set(outputs.bl_steer_vel)
        self._steer_vel["br"].set(outputs.br_steer_vel)

        self._drive_pos["fl"].set(outputs.fl_drive_pos)
        self._drive_pos["fr"].set(outputs.fr_drive_pos)
        self._drive_pos["bl"].set(outputs.bl_drive_pos)
        self._drive_pos["br"].set(outputs.br_drive_pos)

        self._drive_vel["fl"].set(outputs.fl_drive_vel)
        self._drive_vel["fr"].set(outputs.fr_drive_vel)
        self._drive_vel["bl"].set(outputs.bl_drive_vel)
        self._drive_vel["br"].set(outputs.br_drive_vel)

        self._gyro_yaw.set(outputs.gyro_yaw)
        self._gyro_pitch.set(outputs.gyro_pitch)
        self._gyro_roll.set(outputs.gyro_roll)

        if outputs.pose is not None:
            self._pose.set(outputs.pose)

        if outputs.fuel_poses:
            self._fuel_poses.set(outputs.fuel_poses)
            pass

        # self._tick_count += 1
        # self._tick.set(self._tick_count)

        self.inst.flush()

    def is_connected(self) -> bool:
        """Check if connected to NetworkTables server."""
        return self.inst.isConnected()

    def close(self) -> None:
        """Clean up NetworkTables connection."""
        self.inst.stopClient()
