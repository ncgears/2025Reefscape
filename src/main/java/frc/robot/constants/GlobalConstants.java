
package frc.robot.constants;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants that are Global for the robot
 */
public class GlobalConstants {
    //Global Constants
    public static final int kFalconMaxRPS = 6350 / 60;
    public static final int kKrakenMaxRPS = 5800 / 60;
    public static final boolean CAMERA_ENABLED = false; //set to false if UsbCamera is removed
    public static final boolean SWERVE_SENSOR_NONCONTINUOUS = false;
    public static final int kTimeoutMs = 30; //Timeout for reporting in DS if action fails, set to 0 to skip confirmation
    public static final int kPidIndex = 0;  //Talon PID index for primary loop
    public static final int kPidProfileSlotIndex = 0; //PID Profile gains slot
    //2024 robot is 28x31 (frame perim), wheelbase
    // public static final int ROBOT_WIDTH = 28; //Width of the robot frame (from the pivot of the wheels)
    // public static final int ROBOT_LENGTH = 28; //Length of the robot frame (from the pivot of the wheels)
    public static final int kWheelbaseWidth = 23; //from pivot to pivot of swerve module side to side
    public static final int kWheelbaseLength = 23; //from pivot to pivot of the swerve module front to back
    public static final int kFrameWidth = 28; //outside frame perimeter side to side
    public static final int kFrameLength = 30; //outside frame perimeter front to back
    public static final int kBumperWidth = kFrameWidth + 7; //outside of bumpers side to side //.89m
    public static final int kBumperLength = kFrameLength + 7; //outside of bumpers front to back //.965m
    public static final boolean DEBUG_ENABLED_DEFAULT = true; //Default starting state of debug mode
    public static final int DEBUG_RECURRING_TICKS = 100; //Periodic cycles for recubring debug messages
    public static final int DASH_RECURRING_TICKS = 50; //Periodic cycles for dashboard updates
    public final static boolean tuningMode = true; //Enable tunable numbers
}
