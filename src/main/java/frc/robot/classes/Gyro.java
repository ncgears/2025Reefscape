package frc.robot.classes;

import java.util.Map;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.*; 
import frc.robot.utils.NCDebug;
import frc.robot.RobotContainer;

public class Gyro implements Sendable {
	private static Gyro instance;
	private static double m_yawOffset = 0.0;

	private static Pigeon2 m_gyro = new Pigeon2(GyroConstants.kPigeonID,GyroConstants.kCANbus);

	/**
	 * Returns the instance of the Gyro subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return GyroSubSystem instance
	 */
    public static Gyro getInstance() {
		if (instance == null)
			instance = new Gyro();
		return instance;
	}

	/**
	 * Initializes the GyroSubsystem class, performs setup steps, etc.
	 */
    public Gyro() {
    }

	public void buildDashboards() {
    if(true) { //false to disable gyro on system dashboard
			ShuffleboardTab systemTab = Shuffleboard.getTab("System");
			systemTab.add("Gyro", this)
				.withSize(4, 4)
				.withPosition(0, 0)  
				.withProperties(Map.of("counter_clockwise_positive",true));
    }
	  if(GyroConstants.debugDashboard) {
			ShuffleboardTab debugTab = Shuffleboard.getTab("DBG:Gyro");
			debugTab.add("Value", this)
				.withSize(5, 4)
				.withPosition(0, 0)  
				.withProperties(Map.of("counter_clockwise_positive",true));
			debugTab.addNumber("Pitch", this::getPitch)
				.withSize(5, 2)
				.withPosition(0, 4)
				.withWidget("Text Display");
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Gyro");
		builder.setActuator(false);
		builder.addDoubleProperty("Value", () -> getYaw().getDegrees(), null);
	}

    public void zeroYaw() {
		// m_gyro.reset();
		m_gyro.setYaw(0,0.1);
		m_gyro.getYaw().waitForUpdate(0.1);
		m_yawOffset = 0.0;
		NCDebug.Debug.debug("Gyro: Reset Gyro");
	}
    public void zeroYaw(double offset) {
		zeroYaw();
		setYawOffset(offset);
	}

	public void setYawOffset(double offset) {
		m_yawOffset = offset;
		NCDebug.Debug.debug("Gyro: Yaw offset set to "+offset+" degrees");
	}

   	/**
     * Returns the yaw/heading of the robot.
     * @return the robot's yaw as a Rotation2d
     */
	public Rotation2d getYaw() {
		// return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
		double yaw = m_gyro.getYaw().getValueAsDouble() - m_yawOffset; //subtract the offset
		yaw += (yaw < 0) ? 360.0 : 0; //make it positive
		yaw *= (GyroConstants.kGyroReversed) ? -1.0 : 1.0; //invert to CCW Positive
		return Rotation2d.fromDegrees(yaw);
	}

   	/**
     * Returns the pitch of the robot.
     * @return the robot's pitch as a double in degrees
     */
	public double getPitch() {
		return m_gyro.getPitch().getValueAsDouble();
	}

   	/**
     * Returns the roll of the robot.
     * @return the robot's roll as a double in degrees
     */
	public double getRoll() {
		return m_gyro.getRoll().getValueAsDouble();
	}
}
