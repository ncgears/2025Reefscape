/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot.utils;

public class PIDGains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;
    public final double kNeutralDeadband;
    public final double kCruise;
    public final double kAccel;
    public final int kSCurve;
	
    /**
     * Creates a new constants object for initializing a TalonSRX Motor Controller
     * @param kP - (double) Proportional constant
     * @param kI - (double) Integral constant
     * @param kD - (double) Derivative constant
     * @param kF - (double) Feed Forward constant
     * @param kIZone - (int) Integral Zone constant
     * @param PeakOutput - (double) Maximum output of controller (usually 1.0)
     * @param NeutralDeadband - (double) Neutral Deadband (0.04 default)
     * @param kCruise - (double) Cruise constant for motionmagic
     * @param kAccel - (double) Accelleration constant for motionmagic
     * @param kSCurve - (int) SCurve Strength constant for motionmagic [0 none - 8 strong]
     */
	public PIDGains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput, double _kNeutralDeadband, double _kCruise, double _kAccel, int _kSCurve){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
        kNeutralDeadband = _kNeutralDeadband;
        kCruise = _kCruise;
        kAccel = _kAccel;
        kSCurve = _kSCurve;
	}
}