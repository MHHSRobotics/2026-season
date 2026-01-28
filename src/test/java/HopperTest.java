import frc.robot.io.MotorIO;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.RollerState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

public class HopperTest {
    Hopper hopper;

    MotorIO mockRollerMotor = mock();

    @BeforeEach
    void setup() {
        hopper = new Hopper(mockRollerMotor);
    }

    @Test
    void forwardTest() {
        hopper.setRollerForward();
        assertEquals(hopper.getRollerState(), RollerState.ROLLER_FORWARD);
        verify(mockRollerMotor).setDutyCycle(1);
    }

    @Test
    void reverseTest() {
        hopper.setRollerReverse();
        assertEquals(hopper.getRollerState(), RollerState.ROLLER_REVERSE);
        verify(mockRollerMotor).setDutyCycle(-1);
    }

    @Test
    void stopTest() {
        hopper.setRollerStop();
        assertEquals(hopper.getRollerState(), RollerState.ROLLER_STOP);
        verify(mockRollerMotor).setDutyCycle(0);
    }
}
