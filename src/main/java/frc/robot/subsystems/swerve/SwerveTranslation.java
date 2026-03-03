package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Pseudo-subsystem that acts as a scheduler token for translation commands.
// Commands that require this subsystem will properly conflict with each other,
// while allowing independent rotation commands to run simultaneously.
public class SwerveTranslation extends SubsystemBase {}
