
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
        public static int indexer_beambreak = 0;
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
        public static int climber_leftServo = 0;
        public static int climber_rightServo = 1;
        public static int climber_ratchetServo = 2;
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
        public static int swerve_fl_turn = 1;
        public static int swerve_fr_turn = 4;
        public static int swerve_bl_turn = 2;
        public static int swerve_br_turn = 3;
        public static int intake = 10;
        public static int indexer = 11;
    }
    /**
     * IDs of Krakens/Falcons
     */
    public static final class TalonFX {
        public static int swerve_fl_drive = 5;
        public static int swerve_fr_drive = 8;
        public static int swerve_br_drive = 7;
        public static int swerve_bl_drive = 6;
        public static int shootertop = 12;
        public static int shooterbottom = 13;
        public static int aimer = 14;
        public static int arm = 15;
        public static int climber = 16;
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
        public static int aimer = 20;
        public static int climber = 21;
        public static int arm = 22;
    }
}
