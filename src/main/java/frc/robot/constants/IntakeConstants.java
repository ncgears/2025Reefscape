
package frc.robot.constants;

import com.ctre.phoenix6.signals.NeutralModeValue;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants for the Intake Subsystem
 */
public class IntakeConstants {
    //Controller Setup
    public static final String canBus = "rio";
    public static final boolean debugDashboard = false; //enable debugging dashboard
    public static final int kMotorID = ID.Talon.intake;
    public static final boolean kIsInverted = true;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    public static final double kSpeed = 0.75;
}
