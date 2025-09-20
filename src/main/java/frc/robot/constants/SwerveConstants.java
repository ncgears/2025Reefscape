package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveConstants {
    public static final LinearVelocity kMaxSpeedMetersPerSecond = MetersPerSecond.of(4.77); //kSpeedAt12Volts desired top speed (was 10.64)
    public static final double kMaxAngularRate = 0.75; //3/4 of rotation per second max angular velocity
    public static final double kWheelDiamInches = 4.0; //Wheel diameter in inches
    public static final int kPigeonId = ID.Pigeon2.gyro; 
    public static final String kCANbus = "drivetrain"; //canbus name for drivetrain systems
    public static final boolean kInvertLeftSide = false;
    public static final boolean kInvertRightSide = true;
    public static final double kAlignStrafeSpeed = 0.5; //when using pov alignment buttons, this is the requested strafe speed percentage
    public static final boolean debugDashboard = false;
    public class steer {
        public static final double kP = 40.739;
        public static final double kI = 0.0;
        public static final double kD = 0.43733;
        public static final double kS = 0.14259;
        public static final double kV = 1.2168;
        public static final double kA = 1.021733;

        // public static final double kP = 40; //59.165;
        // public static final double kI = 0.0;
        // public static final double kD = 3.5; //11.312;
        // public static final double kS = 0.34135;
        // public static final double kV = 1.004;
        // public static final double kA = 1.2282;
        public static final int kStatorCurrentLimit = 60;
        public static final double kGearRatio = 10.3846154; //Final gear ratio from rotor to mechanism (sensor)
    }
    public class drive {
        public static final double kP = 0.067; //0.1
        public static final double kI = 0.0; //0.0
        public static final double kD = 0.0; //0.0
        public static final double kS = 0.099689; //0.2
        public static final double kV = 0.12558; //0.12
        // public static final double kA = 0.0022959;
        public static final int kSlipCurrent = 120; //Current at which the wheels begin to slip
        public static final double kGearRatio = 6.0; //Final gear ratio from rotor to mechanism (wheel)
    }
    public class modules {
        public class FrontLeft {
            public static final int kDriveMotorId = ID.TalonFX.swerve_fl_drive;
            public static final int kSteerMotorId = ID.TalonFX.swerve_fl_turn;
            public static final int kEncoderId = ID.CANcoder.swerve_fl_cc;
            public static final Angle kEncoderOffset = Rotations.of(0.454590);
            public static final boolean kSteerMotorInverted = true;
            public static final boolean kEncoderInverted = true;
            public static final Distance kXPos = Inches.of(11.5); //forward+ from center
            public static final Distance kYPos = Inches.of(11.5); //left+ from center
        }
        public class FrontRight {
            public static final int kDriveMotorId = ID.TalonFX.swerve_fr_drive;
            public static final int kSteerMotorId = ID.TalonFX.swerve_fr_turn;
            public static final int kEncoderId = ID.CANcoder.swerve_fr_cc;
            public static final Angle kEncoderOffset = Rotations.of(0.334229);
            public static final boolean kSteerMotorInverted = true;
            public static final boolean kEncoderInverted = true;
            public static final Distance kXPos = Inches.of(11.5); //forward+ from center
            public static final Distance kYPos = Inches.of(-11.5); //left+ from center
        }
        public class BackLeft {
            public static final int kDriveMotorId = ID.TalonFX.swerve_bl_drive;
            public static final int kSteerMotorId = ID.TalonFX.swerve_bl_turn;
            public static final int kEncoderId = ID.CANcoder.swerve_bl_cc;
            public static final Angle kEncoderOffset = Rotations.of(0.153564);
            public static final boolean kSteerMotorInverted = true;
            public static final boolean kEncoderInverted = true;
            public static final Distance kXPos = Inches.of(-11.5); //forward+ from center
            public static final Distance kYPos = Inches.of(11.5); //left+ from center
        }
        public class BackRight {
            public static final int kDriveMotorId = ID.TalonFX.swerve_br_drive;
            public static final int kSteerMotorId = ID.TalonFX.swerve_br_turn;
            public static final int kEncoderId = ID.CANcoder.swerve_br_cc;
            public static final Angle kEncoderOffset = Rotations.of(-0.454102);
            public static final boolean kSteerMotorInverted = true;
            public static final boolean kEncoderInverted = true;
            public static final Distance kXPos = Inches.of(-11.5); //forward+ from center
            public static final Distance kYPos = Inches.of(-11.5); //left+ from center
        }
    }
}
