package frc.robot.classes;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.*; 
import frc.robot.utils.NCDebug;
import frc.robot.RobotContainer;

public class Gyro implements Sendable {
	private static Gyro instance;
	private static double m_yawOffset = 0.0;

	private static AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    int m_simgyro = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_simgyro,"Yaw"));
    SimDouble pitch = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_simgyro,"Pitch"));

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
		m_gyro.zeroYaw();
		m_yawOffset = 0.0;
		Helpers.Debug.debug("Gyro: Reset Gyro");
	}
    public void zeroYaw(double offset) {
		zeroYaw();
		setYawOffset(offset);
	}

	public Command zeroYawFromPoseCommand() {
		var angle = RobotContainer.pose.getPose().getRotation().getDegrees();
		return new InstantCommand(() -> zeroYaw(angle)).ignoringDisable(true);
	}


	public void setYawOffset(double offset) {
		m_yawOffset = offset;
		Helpers.Debug.debug("Gyro: Yaw offset set to "+offset+" degrees");
	}

   	/**
     * Returns the yaw/heading of the robot.
     * @return the robot's yaw as a Rotation2d
     */
	public Rotation2d getYaw() {
		double yaw = m_gyro.getYaw() - m_yawOffset; //subtract the offset
		yaw += (yaw < 0) ? 360.0 : 0; //make it positive
		yaw *= (GyroConstants.kGyroReversed) ? -1.0 : 1.0; //invert to CCW Positive
		return Rotation2d.fromDegrees(yaw);
	}

   	/**
     * Returns the pitch of the robot.
     * @return the robot's pitch as a double in degrees
     */
	public double getPitch() {
		return m_gyro.getPitch();
	}

   	/**
     * Returns the roll of the robot.
     * @return the robot's roll as a double in degrees
     */
	public double getRoll() {
		return m_gyro.getRoll();
	}
}
