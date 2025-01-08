
package frc.robot.constants;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants for the Shooter Subsystem
 */
public class ShooterConstants {
    //Controller Setup
    public static final String canBus = "rio";
    public static final boolean debugDashboard = true; //enable debugging dashboard
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast; 
    public static final boolean isInverted = false;
    public static final double kGearRatio = 0.66667; //36:24 pulley //Adjust for gearing on output of Falcon
    public static final double kMaxRPS = GlobalConstants.kFalconMaxRPS / kGearRatio; //The Maximum free speed of the shooter
    public static final double kSpeedToleranceOptimal = 2.0; //How close in RPS is considered at speed
    public static final double kSpeedToleranceAcceptable = 5.0; //How close in RPS is considered close enough to shoot
    //PID Control
    //these are based on SysId characterization with 1:1 pulleys on 3/17/2024
    // public static final double kS = 0.1367; // add kS to overcome static friction: adjust first to start moving
    // public static final double kV = 0.12189; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
    // public static final double kA = 0.0080711; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
    // public static final double kP = 0.18331; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
    //these are based on SysId characterization with 1.5:1 pulleys on 3/21/2024
    public static final double kS = 0.091751; // add kS to overcome static friction: adjust first to start moving
    public static final double kV = 0.085922; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
    public static final double kA = 0.010311; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
    public static final double kP = 0.10799; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
    public static final double kI = 0.0; // no integral
    public static final double kD = 0.0; // 0.1 = velocity error of 1rps results in 0.1v output
    public static final double kMotionMagicCruise = 40; // Motor Max / Gear Ratio
    public static final double kMotionMagicAccel = 400; // Acceleration: Cruise / Accel = time to cruise
    public static final double kMotionMagicJerk = 7000; //0=disabled; 10-20x accel for smooth; lower for smoother motion at the cost of time: accel / jerk = jerk time
    //Current Limiting
    public static final double kPeakFwdVoltage = 12.0;
    public static final double kPeakRevVoltage = -12.0;
    public static final boolean kCurrentLimitEnable = false;
    public static final double kCurrentLimitAmps = 10.0;
    public static final double kCurrentLimitThresholdAmps = 15.0;
    public static final double kCurrentLimitThresholdSecs = 0.3;
    //Ramping (0.0 by default)
    public static final double kOpenLoopRamp = 0.0;
    public static final double kClosedLoopRamp = 0.0;
    public static final Transform3d kRobotToShooter = new Transform3d(
        new Translation3d(0.0,0.0,0.32), //x,y,z location of shooter on robot in meters
        new Rotation3d(Math.PI,0,0) //yaw,pitch,roll of shooter on robot in radians
    );
    public static final class Top {
        public static final int kMotorID = ID.TalonFX.shootertop;
        public static final boolean kIsInverted = false;
    }
    public static final class Bottom {
        public static final int kMotorID = ID.TalonFX.shooterbottom;
        public static final boolean kIsInverted = true;
    }
}
