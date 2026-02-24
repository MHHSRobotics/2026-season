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
    steer_setpoint: float = 0.0  # radians, for sim-side PID
    drive_velocity_setpoint: float = 0.0  # rad/s at wheel, for sim-side PID


@dataclass
class SteerPIDGains:
    """PID gains for sim-side steer control."""
    kP: float = 0.0
    kI: float = 0.0
    kD: float = 0.0


@dataclass
class DrivePIDGains:
    """PID + feedforward gains for sim-side drive velocity control."""
    kP: float = 0.0
    kI: float = 0.0
    kD: float = 0.0
    kS: float = 0.0  # static friction feedforward (V)
    kV: float = 0.0  # velocity feedforward (V/(rad/s))


@dataclass
class MechanismInputs:
    """Voltage inputs for intake, hopper, and shooter mechanisms."""
    intake_hinge_voltage: float = 0.0
    intake_roller_voltage: float = 0.0
    hopper_roller_voltage: float = 0.0
    shooter_feed_voltage: float = 0.0
    shooter_flywheel_voltage: float = 0.0


@dataclass
class SwerveInputs:
    """All motor voltage inputs from WPILib."""
    fl: SwerveModuleState
    fr: SwerveModuleState
    bl: SwerveModuleState
    br: SwerveModuleState
    steer_pid: SteerPIDGains = None
    drive_pid: DrivePIDGains = None
    mechanisms: MechanismInputs = None


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

    # Shooting stats
    shots_fired: int = 0
    balls_scored: int = 0
    balls_missed: int = 0

    # Mechanism feedback
    intake_hinge_pos: float = 0.0
    intake_hinge_vel: float = 0.0
    intake_roller_pos: float = 0.0
    intake_roller_vel: float = 0.0
    hopper_roller_pos: float = 0.0
    hopper_roller_vel: float = 0.0
    shooter_feed_pos: float = 0.0
    shooter_feed_vel: float = 0.0
    shooter_flywheel_pos: float = 0.0
    shooter_flywheel_vel: float = 0.0


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
        self._steer_setpoint = {}
        self._drive_velocity_setpoint = {}
        for key, name in _MODULE_NAMES.items():
            mod = swerve.getSubTable(name)
            self._drive_voltage[key] = mod.getDoubleTopic("Drive/AppliedVoltage").subscribe(0.0, pub_opts)
            self._steer_voltage[key] = mod.getDoubleTopic("Steer/AppliedVoltage").subscribe(0.0, pub_opts)
            self._steer_setpoint[key] = mod.getDoubleTopic("Setpoint").subscribe(0.0, pub_opts)
            self._drive_velocity_setpoint[key] = mod.getDoubleTopic("Drive/Setpoint").subscribe(0.0, pub_opts)

        # Mechanism voltage subscribers
        ak = self.inst.getTable("AdvantageKit")
        self._intake_hinge_voltage = ak.getDoubleTopic("Intake/Hinge/AppliedVoltage").subscribe(0.0, pub_opts)
        self._intake_roller_voltage = ak.getDoubleTopic("Intake/Flywheel/AppliedVoltage").subscribe(0.0, pub_opts)
        self._hopper_roller_voltage = ak.getDoubleTopic("Hopper/Motor/AppliedVoltage").subscribe(0.0, pub_opts)
        self._shooter_feed_voltage = ak.getDoubleTopic("Shooter/Feed/AppliedVoltage").subscribe(0.0, pub_opts)
        self._shooter_flywheel_voltage = ak.getDoubleTopic("Shooter/Flywheel/AppliedVoltage").subscribe(0.0, pub_opts)

        # PID gains from Swerve/ table (NOT AdvantageKit/Swerve/)
        pid_table = self.inst.getTable("Swerve")
        self._steer_kP = pid_table.getDoubleTopic("SteerKP").subscribe(0.0, pub_opts)
        self._steer_kI = pid_table.getDoubleTopic("SteerKI").subscribe(0.0, pub_opts)
        self._steer_kD = pid_table.getDoubleTopic("SteerKD").subscribe(0.0, pub_opts)
        self._drive_kP = pid_table.getDoubleTopic("DriveKP").subscribe(0.0, pub_opts)
        self._drive_kI = pid_table.getDoubleTopic("DriveKI").subscribe(0.0, pub_opts)
        self._drive_kD = pid_table.getDoubleTopic("DriveKD").subscribe(0.0, pub_opts)
        self._drive_kS = pid_table.getDoubleTopic("DriveKS").subscribe(0.0, pub_opts)
        self._drive_kV = pid_table.getDoubleTopic("DriveKV").subscribe(0.0, pub_opts)

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

        # Mechanism output publishers
        sim_mech = self.inst.getTable("MuJoCo")
        self._intake_hinge_pos_pub = sim_mech.getDoubleTopic("Intake/Hinge/Position").publish(pub_opts)
        self._intake_hinge_vel_pub = sim_mech.getDoubleTopic("Intake/Hinge/Velocity").publish(pub_opts)
        self._intake_roller_pos_pub = sim_mech.getDoubleTopic("Intake/Flywheel/Position").publish(pub_opts)
        self._intake_roller_vel_pub = sim_mech.getDoubleTopic("Intake/Flywheel/Velocity").publish(pub_opts)
        self._hopper_roller_pos_pub = sim_mech.getDoubleTopic("Hopper/Roller/Position").publish(pub_opts)
        self._hopper_roller_vel_pub = sim_mech.getDoubleTopic("Hopper/Roller/Velocity").publish(pub_opts)
        self._shooter_feed_pos_pub = sim_mech.getDoubleTopic("Shooter/Feed/Position").publish(pub_opts)
        self._shooter_feed_vel_pub = sim_mech.getDoubleTopic("Shooter/Feed/Velocity").publish(pub_opts)
        self._shooter_flywheel_pos_pub = sim_mech.getDoubleTopic("Shooter/Flywheel/Position").publish(pub_opts)
        self._shooter_flywheel_vel_pub = sim_mech.getDoubleTopic("Shooter/Flywheel/Velocity").publish(pub_opts)

        # Robot pose output (struct-encoded Pose3d)
        self._pose = self.inst.getStructTopic("MuJoCo/Swerve/Pose", Pose3d).publish(pub_opts)

        # Shooting stats
        shooter_stats = self.inst.getTable("MuJoCo/Shooter")
        self._shots_fired_pub = shooter_stats.getIntegerTopic("ShotsFired").publish(pub_opts)
        self._balls_scored_pub = shooter_stats.getIntegerTopic("BallsScored").publish(pub_opts)
        self._balls_missed_pub = shooter_stats.getIntegerTopic("BallsMissed").publish(pub_opts)
        self._accuracy_pub = shooter_stats.getDoubleTopic("Accuracy").publish(pub_opts)

        # Fuel ball poses (struct array of Pose3d) - disabled to reduce NT load
        # fuel_opts = ntcore.PubSubOptions(periodic=0.05)
        # self._fuel_poses = self.inst.getStructArrayTopic("MuJoCo/Fuel", Pose3d).publish(fuel_opts)

        # LED color subscriber (integer array [R, G, B], 0-255)
        self._led_color = ak.getIntegerArrayTopic("RealOutputs/LED/Color").subscribe([0, 0, 0], pub_opts)

        # Debug: tick counter to verify publish rate in AdvantageScope
        # self._tick = self.inst.getTable("MuJoCo").getDoubleTopic("Tick").publish(pub_opts)
        # self._tick_count = 0

    def get_inputs(self) -> SwerveInputs:
        """Read motor voltages and setpoints from NetworkTables."""
        def module(key: str) -> SwerveModuleState:
            return SwerveModuleState(
                steer_voltage=self._steer_voltage[key].get(),
                drive_voltage=self._drive_voltage[key].get(),
                steer_setpoint=self._steer_setpoint[key].get(),
                drive_velocity_setpoint=self._drive_velocity_setpoint[key].get(),
            )

        return SwerveInputs(
            fl=module("fl"),
            fr=module("fr"),
            bl=module("bl"),
            br=module("br"),
            mechanisms=MechanismInputs(
                intake_hinge_voltage=self._intake_hinge_voltage.get(),
                intake_roller_voltage=self._intake_roller_voltage.get(),
                hopper_roller_voltage=self._hopper_roller_voltage.get(),
                shooter_feed_voltage=self._shooter_feed_voltage.get(),
                shooter_flywheel_voltage=self._shooter_flywheel_voltage.get(),
            ),
            steer_pid=SteerPIDGains(
                kP=self._steer_kP.get(),
                kI=self._steer_kI.get(),
                kD=self._steer_kD.get(),
            ),
            drive_pid=DrivePIDGains(
                kP=self._drive_kP.get(),
                kI=self._drive_kI.get(),
                kD=self._drive_kD.get(),
                kS=self._drive_kS.get(),
                kV=self._drive_kV.get(),
            ),
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

        self._intake_hinge_pos_pub.set(outputs.intake_hinge_pos)
        self._intake_hinge_vel_pub.set(outputs.intake_hinge_vel)
        self._intake_roller_pos_pub.set(outputs.intake_roller_pos)
        self._intake_roller_vel_pub.set(outputs.intake_roller_vel)
        self._hopper_roller_pos_pub.set(outputs.hopper_roller_pos)
        self._hopper_roller_vel_pub.set(outputs.hopper_roller_vel)
        self._shooter_feed_pos_pub.set(outputs.shooter_feed_pos)
        self._shooter_feed_vel_pub.set(outputs.shooter_feed_vel)
        self._shooter_flywheel_pos_pub.set(outputs.shooter_flywheel_pos)
        self._shooter_flywheel_vel_pub.set(outputs.shooter_flywheel_vel)

        if outputs.pose is not None:
            self._pose.set(outputs.pose)

        # Shooting stats
        self._shots_fired_pub.set(outputs.shots_fired)
        self._balls_scored_pub.set(outputs.balls_scored)
        self._balls_missed_pub.set(outputs.balls_missed)
        accuracy = outputs.balls_scored / outputs.shots_fired if outputs.shots_fired > 0 else 0.0
        self._accuracy_pub.set(accuracy)

        # if outputs.fuel_poses:
        #     self._fuel_poses.set(outputs.fuel_poses)

        # self._tick_count += 1
        # self._tick.set(self._tick_count)

        self.inst.flush()

    def get_led_color(self) -> tuple:
        """Get LED color as (r, g, b) floats in [0, 1]."""
        rgb = self._led_color.get()
        if len(rgb) >= 3:
            return (rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0)
        return (0.0, 0.0, 0.0)

    def is_connected(self) -> bool:
        """Check if connected to NetworkTables server."""
        return self.inst.isConnected()

    def close(self) -> None:
        """Clean up NetworkTables connection."""
        self.inst.stopClient()
