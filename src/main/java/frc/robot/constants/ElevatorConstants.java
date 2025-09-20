
package frc.robot.constants;

import frc.robot.utils.PIDGains;
import com.ctre.phoenix6.signals.NeutralModeValue;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants for the Climber Subsystem
 */
public class ElevatorConstants {

    //Controller Setup
    public static final String canBus = "rio";
    public static final boolean debugDashboard = false; //enable debugging dashboard
    public static final boolean isDisabled = false; //disable climber default command
    public static final int kCANcoderID = ID.CANcoder.elevator;
    public static final boolean kUseCANcoder = true;
    public static final boolean kSensorInverted = true;
    public static final double kMagnetOffset = 0.125732; //Adjust magnet to sensor offset for CANcoder
    public static final int kMotorID = ID.TalonFX.elevator;
    public static final boolean kIsInverted = true;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    public static final double kPositionTolerance = 0.05; //how close to be considered "at target"
    public static final double kMaxSpeed = 0.90;

    public static final double kGearRatio = 16; // 16:1 gearbox
    public static final double kSensorGearRatio = 1.0; // no gearing between sensor and spool -- this is between sensor and spool
    //PID Control
    public static final double kS = 0.39; // add kS to overcome static friction: adjust first to start moving
    public static final double kV = 1.25; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
    public static final double kA = 0.0; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
    public static final double kP = 35.0; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
    public static final double kI = 0.0; // no integral
    public static final double kD = 0.0; // 0.1 = velocity error of 1rps results in 0.1v output
    public static final double kG = 1.00; // increase until start to move up

    /** finding kS and kG
    * up = Increase output to find when it just starts moving up (from stalled) .5
    * dn = Decrease output to find when it just starts moving down (from stalled) .2
    * kG = (up + dn) / 2
    * kS = (up - dn) / 2
    */

    public static final double kMotionMagicCruise = 4000; // Motor Max / Gear Ratio
    public static final double kMotionMagicAccel = 300; // Acceleration: Cruise / Accel = time to cruise
    public static final double kMotionMagicJerk = 2000; //0=disabled; 10-20x accel for smooth; lower for smoother motion at the cost of time: accel / jerk = jerk time
    //Current Limiting
    public static final boolean kCurrentLimitEnable = false; // TODO: Test current limits
    public static final double kCurrentLimitAmps = 30.0;
    public static final double kCurrentLimitThresholdAmps = 60.0;
    public static final double kCurrentLimitThresholdSecs = 0.3;
    public class Positions {
        private static final double kScoreDelta = 1.0; //0.75;
        public static final double kFwdLimit = 5.85; //Forward imit
        public static final double kRevLimit = -0.01; //Reverse Limit
        public static final double kStow = 0.0; //all the way down
        public static final double kFloor = 0.0;
        public static final double kBarge = 5.68; //all the way up
        public static final double kProcessor = kStow; //all the way down
        public static final double kL4 = 4.87; //Elevator up to L4
        public static final double kL4Score = kL4 - 1.0; //Elevator up to L4 Score
        public static final double kAlgaeHigh = 2.73; //Elevator between L3 and L4
        public static final double kL3 = 2.85; //Elevator up to L3
        public static final double kL3Score = kL3 - kScoreDelta; //Elevator up to L3 Score
        public static final double kLineup = 1.25; //Elevator height for lining up
        public static final double kAlgaeLow = 1.45; //Elevator between L2 and L3
        public static final double kL2 = 1.51; //Elevator up to L2
        public static final double kL2Score = kL2 - kScoreDelta; //Elevator up to L2 Score
        public static final double kL1 = 0.0; //Elevator up to L1
        public static final double kHP = 0.24; //all the way down
    }
    public static final boolean kSoftForwardLimitEnable = true;
    public static final double kSoftForwardLimit = Positions.kFwdLimit;
    public static final boolean kSoftReverseLimitEnable = true;
    public static final double kSoftReverseLimit = Positions.kRevLimit;
}
