
package frc.robot.constants;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants for the Autonomous subsystem
 */
public class AutonConstants {
    public static final boolean isDisabled = false; //Disable autonomous
    public static final boolean kUseChoreo = true; //true uses choreo
    public static final boolean kUseTracking = true; //enable target tracking during auton pathing
    public static final double kMaxSpeedMetersPerSecond = 4.77;
    public static final double kMaxAccelMetersPerSecondSquared = 1.0;
    public static final double kMaxOmega = 2.0 * Math.PI; //(kMaxSpeedMetersPerSecond / Math.hypot(0.584 / 2.0, 0.66 / 2.0));
    public static final double kPTranslationController = 5; //0.85;
    public static final double kPThetaController = 5; //0.8;
}
