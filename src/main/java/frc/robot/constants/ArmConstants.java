
package frc.robot.constants;

import com.ctre.phoenix6.signals.NeutralModeValue;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants for the Arm Subsystem
 */
public class ArmConstants {
    //Controller Setup
    public static final String canBus = "rio";
    public static final boolean debugDashboard = false; //enable debugging dashboard
    public static final int kCANcoderID = ID.CANcoder.arm;
    public static final boolean kUseCANcoder = true;
    public static final double kMagnetOffset = -0.098876953125; //Adjust magnet to sensor offset for CANcoder
    public static final int kMotorID = ID.TalonFX.arm;
    public static final boolean kIsInverted = true;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    public static final double kStowPosition = 0;
    public static final double kGearRatio = 49.08163; //9.816326; // 14t:26t -> 14t:74t
    public static final double kPositionThreshold = 0.04; //close enough to target position
    //PID Control
    public static final double kS = 0.15; // add kS to overcome static friction: adjust first to start moving
    public static final double kV = 0.0; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
    public static final double kA = 0.0; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
    public static final double kP = 24.0; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
    public static final double kI = 0.0; // no integral
    public static final double kD = 0.0; // 0.1 = velocity error of 1rps results in 0.1v output
    public static final double kMotionMagicCruise = 30; // Motor Max / Gear Ratio
    public static final double kMotionMagicAccel = 80; // Acceleration: Cruise / Accel = time to cruise
    public static final double kMotionMagicJerk = 1600; //0=disabled; 10-20x accel for smooth; lower for smoother motion at the cost of time: accel / jerk = jerk time
    //Current Limiting
    public static final boolean kCurrentLimitEnable = false;
    public static final double kCurrentLimitAmps = 30.0;
    public static final double kCurrentLimitThresholdAmps = 60.0;
    public static final double kCurrentLimitThresholdSecs = 0.3;
    //Positions
    public class Positions {
        public static final double kIntake = 0.0;
        public static final double kTrapBalance = 0.2751465; //position for balancing for trap climb
        public static final double kTrapClimb = 0.0932617; //position to prepare for trap climbing
        public static final double kAmp = 0.381; //amp scoring position
        public static final double kTrap = 0.345; //position to put note in trap
    }
    public static final boolean kSoftForwardLimitEnable = true;
    public static final double kSoftForwardLimit = 0.4;
    public static final boolean kSoftReverseLimitEnable = true;
    public static final double kSoftReverseLimit = -0.01;
}
