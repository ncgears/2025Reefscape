package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.*; 

public class NCDebug {
    //Helpers for Debugging
    public static final class Debug {
        private static boolean debugEnabled = GlobalConstants.DEBUG_ENABLED_DEFAULT;
        /**
         * This function takes a string and outputs it to the console when the debugging is enabled
         * @param message String to print to console
         */
        public static final void debug(String message) {
            if (debugEnabled) {
                System.out.println(message);
            }
        }
        public static final Command debugC(String message) {
          return new InstantCommand(() -> debug(message));
        }

        /**
         * This function toggles the debugging output to console. In a future version, each press will increase the debug level
         * through a set list of severity levels.
         */
        public final static void toggleDebug() {
            debugEnabled = !debugEnabled;
            System.out.println("Debugging Output=" + debugEnabled);
        }
    }
    //General Helpers
    public static final class General {
        public final static double roundDouble(double val, int decimals) {
            return Math.round(val * Math.pow(10,decimals)) / Math.pow(10,decimals);
            // final DecimalFormat df = new DecimalFormat(format);
            // return df.format(val);
        }
    }
}