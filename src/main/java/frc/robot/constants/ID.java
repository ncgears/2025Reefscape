
package frc.robot.constants;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * The ID Class defines the hardware/canbus IDs of things
 */
public class ID {
    /**
     * IDs of RoboRio Digital IO
     */
    public static final class DIO {
        //public static int some_named_dio = 0;
        public static int climber_cageSwitch1 = 0;
        public static int climber_cageSwitch2 = 1;
        public static int climber_climbSwitch = 2;
    }
    /**
     * IDs of RoboRio Analog IO
     */
    public static final class Analog {
        //public static int some_named_aio = 0;
    }
    /**
     * IDs of RoboRio PWM
     */
    public static final class PWM {
    }
    /**
     * IDs of RoboRio Relays
     */
    public static final class Relay {
    }
    /**
     * IDs of Talons
     */
    public static final class Talon {
    }
    /**
     * IDs of Gyros
     */
    public static final class Pigeon2 {
        public static int gyro = 0;
    }
    /**
     * IDs of Krakens/Falcons
     */
    public static final class TalonFX {
        public static int swerve_fl_drive = 8;
        public static int swerve_fr_drive = 5;
        public static int swerve_bl_drive = 7;
        public static int swerve_br_drive = 6;
        public static int swerve_fl_turn = 1;
        public static int swerve_fr_turn = 3;
        public static int swerve_bl_turn = 4;
        public static int swerve_br_turn = 2;
        public static int climber = 9;
        public static int coral = 10;
        public static int algae_wrist = 11;
        public static int elevator = 12;
    }
    /**
     * IDs of TalonFXS
     */
    public static final class TalonFXS {
        public static int algae_left = 34;
        public static int algae_right = 33;
    }
    /**
     * IDs of CANdles
     */
    public static final class CANdle {
        public static int candle1 = 17;
        public static int candle2 = 18;
    }
    /**
     * IDs of CANcoders
     */
    public static final class CANcoder {
        public static int swerve_fl_cc = 3;
        public static int swerve_fr_cc = 4;
        public static int swerve_bl_cc = 2;
        public static int swerve_br_cc = 1;
        public static int coral = 23; //not in use
        public static int algae = 25;
        public static int climber = 0; //not in use
        public static int elevator = 22;
    }
}
