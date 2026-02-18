import frc.robot.io.BitIO;
import frc.robot.io.MotorIO;
import frc.robot.subsystems.intake.Intake;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

public class IntakeTest {
    Intake intake;

    MotorIO mockIntakeMotor = mock();
    MotorIO mockHingeMotor = mock();
    BitIO mockRightSwitch = mock();
    BitIO mockLeftSwitch = mock();

    @BeforeEach
    void setup() {
        intake = new Intake(mockIntakeMotor, mockHingeMotor, mockRightSwitch, mockLeftSwitch);
    }

    @Test
    void intakeSetsPositiveSpeed() {
        intake.intake();
        verify(mockIntakeMotor).setDutyCycle(Intake.Constants.defaultSpeed);
    }

    @Test
    void outtakeSetsNegativeSpeed() {
        intake.outtake();
        verify(mockIntakeMotor).setDutyCycle(-Intake.Constants.defaultSpeed);
    }

    @Test
    void intakeStopSetsZero() {
        intake.intakeStop();
        verify(mockIntakeMotor).setDutyCycle(0);
    }

    @Test
    void hingeStopSetsZero() {
        intake.hingeStop();
        verify(mockHingeMotor).setDutyCycle(0);
    }

    @Test
    void setIntakeSpeedPassesThrough() {
        intake.setIntakeSpeed(0.8);
        verify(mockIntakeMotor).setDutyCycle(0.8);
    }

    @Test
    void setHingeSpeedPassesThrough() {
        intake.setHingeSpeed(0.3);
        verify(mockHingeMotor).setDutyCycle(0.3);
    }

    @Test
    void setHingeGoalUpSetsIntakeUpTrue() {
        intake.setHingeGoal(Intake.Constants.hingeUp);
        assertTrue(intake.getIntakePos());
    }

    @Test
    void setHingeGoalDownSetsIntakeUpFalse() {
        intake.setHingeGoal(Intake.Constants.hingeDown);
        assertFalse(intake.getIntakePos());
    }

    @Test
    void switchPosSetsDownWhenUp() {
        // Intake starts up (true)
        assertTrue(intake.getIntakePos());
        intake.switchPos();
        // After switching, should be down
        assertFalse(intake.getIntakePos());
    }

    @Test
    void switchPosSetsUpWhenDown() {
        // Move down first
        intake.setHingeGoal(Intake.Constants.hingeDown);
        assertFalse(intake.getIntakePos());
        // Switch should move up
        intake.switchPos();
        assertTrue(intake.getIntakePos());
    }
}
