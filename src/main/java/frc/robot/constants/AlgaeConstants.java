
package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants for the Intake Subsystem
 */
public class AlgaeConstants {
    //Controller Setup
    public static final String canBus = "rio";
    public static final boolean debugDashboard = false; //enable debugging dashboard
    public static final boolean isDisabled = false; //disable algae system
    public static final int kCANcoderID = ID.CANcoder.algae;
    public static final boolean kUseCANcoder = true;
    public static final double kMagnetOffset = -0.6903906; //Adjust magnet to sensor offset for CANcoder
    public static final double kGearRatio = 1.0; // this is between rotor and sensor
    public static final double kSensorGearRatio = 1.0; // no gearing between sensor and spool -- this is between sensor and spool
    public class wrist {
        public static final int kMotorID = ID.TalonFX.algae_wrist;
        public static final boolean kIsInverted = true;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final double kSpeed = 0.75;
    }
    public class left {
        public static final int kMotorID = ID.TalonFXS.algae_left;
        public static final boolean kIsInverted = false;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    }
    public class right {
        public static final int kMotorID = ID.TalonFXS.algae_right;
        public static final boolean kIsInverted = true;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    }

    //PID Control
    public static final double kS = 0.22; // add kS to overcome static friction: adjust first to start moving
    public static final double kV = 0.0; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
    public static final double kA = 0.0; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
    public static final double kP = 32.0; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
    public static final double kI = 0.01; // no integral
    public static final double kD = 0.0; // 0.1 = velocity error of 1rps results in 0.1v output
    public static final double kMotionMagicCruise = 30; // Motor Max / Gear Ratio
    public static final double kMotionMagicAccel = 200; // Acceleration: Cruise / Accel = time to cruise
    public static final double kMotionMagicJerk = 0; //0=disabled; 10-20x accel for smooth; lower for smoother motion at the cost of time: accel / jerk = jerk time
    //Current Limiting
    public static final boolean kCurrentLimitEnable = false; // TODO: Test current limits
    public static final double kCurrentLimitAmps = 30.0;
    public static final double kCurrentLimitThresholdAmps = 30.0;
    public static final double kCurrentLimitThresholdSecs = 0.3;
    public class Positions {
        public static final double kFwdLimit = 0.5; //Forward imit
        public static final double kRevLimit = -0.01; //Reverse Limit
        public static final double kStow = 0.0; //all the way in
    }
    public static final boolean kSoftForwardLimitEnable = true;
    public static final double kSoftForwardLimit = Positions.kFwdLimit;
    public static final boolean kSoftReverseLimitEnable = true;
    public static final double kSoftReverseLimit = Positions.kRevLimit;
}
