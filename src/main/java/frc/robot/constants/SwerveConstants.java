package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class SwerveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.77; //kSpeedAt12Volts desired top speed (was 10.64)
    public static final double kMaxAngularRate = 0.75; //3/4 of rotation per second max angular velocity
    public static final double kWheelDiamInches = 4.0; //Wheel diameter in inches
    public static final String kCANbus = "drivetrain"; //canbus name for drivetrain systems
    public class steer {
        public static final double kP = 21.739;
        public static final double kI = 0.0;
        public static final double kD = 0.43733;
        public static final double kS = 0.14259;
        public static final double kV = 1.2168;
        public static final double kA = 0.021733;
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
            public static final int kDriveMotorId = 7;
            public static final int kSteerMotorId = 3;
            public static final int kEncoderId = 3;
            public static final Angle kEncoderOffset = Rotations.of(0.176513671875);
            public static final boolean kSteerMotorInverted = true;
            public static final Distance kXPos = Inches.of(11.5); //forward+ from center
            public static final Distance kYPos = Inches.of(11.5); //left+ from center
        }
        public class FrontRight {
            public static final int kDriveMotorId = 8;
            public static final int kSteerMotorId = 4;
            public static final int kEncoderId = 4;
            public static final Angle kEncoderOffset = Rotations.of(0.02783203125);
            public static final boolean kSteerMotorInverted = true;
            public static final Distance kXPos = Inches.of(11.5); //forward+ from center
            public static final Distance kYPos = Inches.of(-11.5); //left+ from center
        }
        public class BackLeft {
            public static final int kDriveMotorId = 6;
            public static final int kSteerMotorId = 2;
            public static final int kEncoderId = 2;
            public static final Angle kEncoderOffset = Rotations.of(-0.241943359375);
            public static final boolean kSteerMotorInverted = true;
            public static final Distance kXPos = Inches.of(-11.5); //forward+ from center
            public static final Distance kYPos = Inches.of(11.5); //left+ from center
        }
        public class BackRight {
            public static final int kDriveMotorId = 5;
            public static final int kSteerMotorId = 1;
            public static final int kEncoderId = 1;
            public static final Angle kEncoderOffset = Rotations.of(0.4580078125);
            public static final boolean kSteerMotorInverted = true;
            public static final Distance kXPos = Inches.of(-11.5); //forward+ from center
            public static final Distance kYPos = Inches.of(-11.5); //left+ from center
        }
    }
}
