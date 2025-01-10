package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.constants.*; 
import frc.team1918.robot.Helpers;
import frc.team1918.robot.Robot;
import frc.team1918.robot.RobotContainer;
import frc.team1918.robot.classes.NCPose.Targets;
import frc.team1918.robot.modules.SwerveModule;
import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * This subsystem handles managing the Drivetrain.
 * It is responsible for controlling the swerve modules based on input from the auton system or the driver.
 */
public class DriveSubsystem extends SubsystemBase {
	private static DriveSubsystem instance;
	public Field2d field = new Field2d();

	//initialize 4 swerve modules
	private static SwerveModule m_frontLeft = new SwerveModule("FL", SwerveConstants.FL.constants); // Front Left
	private static SwerveModule m_frontRight = new SwerveModule("FR", SwerveConstants.FR.constants); // Front Right
	private static SwerveModule m_backLeft = new SwerveModule("RL", SwerveConstants.BL.constants); // Rear Left
	private static SwerveModule m_backRight = new SwerveModule("RR", SwerveConstants.BR.constants); // Rear Right
	private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};

	private int m_simgyro = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_simgyro,"Yaw"));
    SimDouble pitch = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_simgyro,"Pitch"));
	private double sim_last_time = Timer.getFPGATimestamp();

	private double target_heading = 0.0;
	private boolean heading_locked = false;
	private boolean m_suppressVision = false;

	public static DriveSubsystem getInstance() {
		if (instance == null)
			instance = new DriveSubsystem();
		return instance;
	}

	public DriveSubsystem() { //initialize the class
		AutoBuilder.configureHolonomic(
		this::getPose, // Robot pose supplier
		this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
		this::getSpeeds,// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
		this::autonDriveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
		new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
			new PIDConstants(AutonConstants.kPTranslationController, 0.0, 0.0), // Translation PID constants
			new PIDConstants(AutonConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
			AutonConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
			Units.inchesToMeters(Math.max(GlobalConstants.kWheelbaseLength / 2, GlobalConstants.kWheelbaseWidth / 2)), // Drive base radius in meters. Distance from robot center to furthest module. 
			new ReplanningConfig() // Default path replanning config. See the API for the options here
		),
		() -> { return RobotContainer.isAllianceRed(); },
		this // Reference to this subsystem to set requirements
		);

		PPHolonomicDriveController.setRotationTargetOverride(this::getAutoTrackedTarget);

		init();
		createDashboards();
	}
	
	public Optional<Rotation2d> getAutoTrackedTarget() {
		if(AutonConstants.kUseTracking && RobotContainer.pose.getTracking()) {
			Rotation2d hdg = Rotation2d.fromDegrees(RobotContainer.pose.getBearingOfTarget(Targets.SPEAKER));
			Helpers.Debug.debug("Tracking: auton heading "+hdg.getDegrees());
			return Optional.of(hdg);
		} else {
			return Optional.empty();
		}
	}

	// @SuppressWarnings("unused")
	@Override
	public void periodic() {
		if(Robot.isSimulation()) updateSim();
		RobotContainer.pose.updatePose();
		autoSuppressVision(); //automatically suppress vision addition based on speed (if enabled)
		if(!m_suppressVision) RobotContainer.pose.correctPoseWithVision();
		field.setRobotPose(RobotContainer.pose.getPose());
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("SwerveDrive");
		builder.setActuator(true);
		//builder.setSafeState(this::disable); //function for safe state to make sure things dont move
		builder.addDoubleProperty("Front Left Angle", () -> Helpers.General.roundDouble(m_frontLeft.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Front Left Velocity", () -> Helpers.General.roundDouble(m_frontLeft.getVelocity(),3), null);

		builder.addDoubleProperty("Front Right Angle", () -> Helpers.General.roundDouble(m_frontRight.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Front Right Velocity", () -> Helpers.General.roundDouble(m_frontRight.getVelocity(),3), null);

		builder.addDoubleProperty("Back Left Angle", () -> Helpers.General.roundDouble(m_backLeft.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Back Left Velocity", () -> Helpers.General.roundDouble(m_backLeft.getVelocity(),3), null);

		builder.addDoubleProperty("Back Right Angle", () -> Helpers.General.roundDouble(m_backRight.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Back Right Velocity", () -> Helpers.General.roundDouble(m_backRight.getVelocity(),3), null);

		// builder.addDoubleProperty("Robot Angle", () -> RobotContainer.pose.getPose().getRotation().getDegrees(), null);
		builder.addDoubleProperty("Robot Angle", () -> RobotContainer.gyro.getYaw().getDegrees(), null);
	}

	public void createDashboards() {
		ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
		driverTab.add("Swerve Drive", this)
			.withSize(4, 4)
			.withPosition(20, 5)
			.withProperties(Map.of("show_robot_rotation","true"));
		// driverTab.add("Field", getField())
		// 	.withSize(12,7)
		// 	.withPosition(8,0)
		// 	.withWidget("Field")
		// 	.withProperties(Map.of("field_game","Crescendo","robot_width",Units.inchesToMeters(Global.kBumperWidth),"robot_length",Units.inchesToMeters(Global.kBumperLength)));

		ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
		swerveTab.add("Swerve Drive", this)
			.withSize(6, 6)
			.withPosition(0, 0)
			.withProperties(Map.of("show_robot_rotation","true"));
		swerveTab.addNumber("FL Angle", () -> Helpers.General.roundDouble(m_frontLeft.getAngle().getDegrees(),2))
			.withSize(2, 2)
			.withPosition(6, 0);
		swerveTab.addNumber("FR Angle", () -> Helpers.General.roundDouble(m_frontRight.getAngle().getDegrees(),2))
			.withSize(2, 2)
			.withPosition(12, 0);
		swerveTab.addNumber("BL Angle", () -> Helpers.General.roundDouble(m_backLeft.getAngle().getDegrees(),2))
			.withSize(2, 2)
			.withPosition(6, 4);
		swerveTab.addNumber("BR Angle", () -> Helpers.General.roundDouble(m_backRight.getAngle().getDegrees(),2))
			.withSize(2, 2)
			.withPosition(12, 4);
		swerveTab.addNumber("FL Speed", () -> Helpers.General.roundDouble(m_frontLeft.getVelocity(),3))
			.withSize(2, 2)
			.withPosition(8, 1);
		swerveTab.addNumber("FR Speed", () -> Helpers.General.roundDouble(m_frontRight.getVelocity(),3))
			.withSize(2, 2)
			.withPosition(10, 1);
		swerveTab.addNumber("BL Speed", () -> Helpers.General.roundDouble(m_backLeft.getVelocity(),3))
			.withSize(2, 2)
			.withPosition(8, 3);
		swerveTab.addNumber("BR Speed", () -> Helpers.General.roundDouble(m_backRight.getVelocity(),3))
			.withSize(2, 2)
			.withPosition(10, 3);
		// swerveTab.add("Field", getField())
		// 	.withSize(6,4)
		// 	.withPosition(0,6)
		// 	.withWidget("Field")
		// 	.withProperties(Map.of("field_game","Crescendo","robot_width",Units.inchesToMeters(Global.kBumperWidth),"robot_length",Units.inchesToMeters(Global.kBumperLength)));

		ShuffleboardLayout thetaList = swerveTab.getLayout("theta Controller", BuiltInLayouts.kList)
			.withSize(4,4)
			.withPosition(6,6)
			.withProperties(Map.of("Label position","LEFT"));
		thetaList.addString("Heading Lock", this::getHeadingLockedColor)
			.withWidget("Single Color View");
		thetaList.addNumber("Target Heading", () -> Helpers.General.roundDouble(getTargetHeading(),4));
		thetaList.addNumber("Current Heading", () -> Helpers.General.roundDouble(getHeading().getDegrees(),4));
		thetaList.addNumber("Heading Error", () -> Helpers.General.roundDouble(getHeadingError(),4));

		ShuffleboardTab systemTab = Shuffleboard.getTab("System");
		systemTab.add("Field", getField())
			.withSize(8,4)
			.withPosition(0,4)
			.withWidget("Field")
			.withProperties(Map.of("field_game","Crescendo","robot_width",Units.inchesToMeters(GlobalConstants.kBumperWidth),"robot_length",Units.inchesToMeters(GlobalConstants.kBumperLength)));
		ShuffleboardLayout systemThetaList = systemTab.getLayout("theta Controller", BuiltInLayouts.kList)
			.withSize(4,4)
			.withPosition(16,6)
			.withProperties(Map.of("Label position","LEFT"));
		systemThetaList.addString("Heading Lock", this::getHeadingLockedColor)
			.withWidget("Single Color View");
		systemThetaList.addNumber("Target Heading", () -> Helpers.General.roundDouble(getTargetHeading(),4));
		systemThetaList.addNumber("Current Heading", () -> Helpers.General.roundDouble(getHeading().getDegrees(),4));
		systemThetaList.addNumber("Heading Error", () -> Helpers.General.roundDouble(getHeadingError(),4));

		if(SwerveConstants.debugDashboard) {
		}

		if(VisionConstants.debugDashboard) {
			ShuffleboardTab debugTab = Shuffleboard.getTab("DBG:Vision");
			debugTab.addBoolean("Suppressed", this::isVisionSuppressed)
			  .withSize(2, 2)
			  .withWidget("Boolean Box")
			  .withPosition(0, 2);  
		}
	}

	public void init() {
		brake(false);
		homeSwerves();
		Helpers.Debug.debug("Pose: Initialized");
	}

	public Field2d getField() {
		return field;
	}

	public Pose2d getPose() {
		return RobotContainer.pose.getPose();
	}

	public void resetPose(Pose2d pose) {
		RobotContainer.pose.resetPose(pose);
	}

	public Rotation2d getHeading() {
		return RobotContainer.gyro.getYaw();
		// return RobotContainer.pose.getPose().getRotation();
	}

	public double getHeadingError() {
		double desired_heading = (isTrackingTarget()) ? Rotation2d.fromDegrees(getTrackingTargetHeading()).rotateBy(new Rotation2d(Math.PI)).getDegrees() : target_heading;
		// double desired_heading = target_heading;
		double error = desired_heading - getHeading().getDegrees();
		return error;
	}

	public void lockHeading() {
		target_heading = RobotContainer.gyro.getYaw().getDegrees();
		heading_locked = true;
	}

	public void unlockHeading() {
		heading_locked = false;
	}

	public void setSuppressVision(boolean suppress) { 
		m_suppressVision = suppress; 
		Helpers.Debug.debug((m_suppressVision) ? "Drive: Vision Suppressed" : "Drive: Vision Unsuppressed");
	}

	public Command suppressVisionC() {
		return runOnce(() -> setSuppressVision(true));
	}

	public Command unsuppressVisionC() {
		return runOnce(() -> setSuppressVision(false));
	}

	public void autoSuppressVision() {
		if(VisionConstants.kUseAutoSuppress) {
			ChassisSpeeds speeds = getSpeeds();
			//if the speed is over threshold, suppress vision measurements from being added to pose
			m_suppressVision = (
				Math.sqrt(
					Math.pow(speeds.vxMetersPerSecond,2) + 
					Math.pow(speeds.vyMetersPerSecond,2)
				) >= VisionConstants.kAutosuppressSpeedMetersPerSecond);
		}
	}
	public boolean isVisionSuppressed() { return m_suppressVision; }

	public boolean getHeadingLocked() { return heading_locked; }
	public String getHeadingLockedColor() {
		return (heading_locked) ?
			(isTrackingTarget()) ? DashboardConstants.Colors.ORANGE : DashboardConstants.Colors.GREEN
			: DashboardConstants.Colors.RED;
	}
	public double getTargetHeading() { return (isTrackingTarget()) ? getTrackingTargetHeading() : target_heading; }
	public boolean isTrackingTarget() { return RobotContainer.pose.getTracking(); }
	public double getTrackingTargetHeading() { 
		// return Rotation2d.fromDegrees(RobotContainer.pose.getTrackingTargetBearing()).rotateBy(new Rotation2d(Math.PI)).getDegrees(); 
		return Rotation2d.fromDegrees(RobotContainer.pose.getTrackingTargetBearing()).getDegrees(); 
	}

	public void updateSim() {
		for (SwerveModule module: modules) {
			double ts = Timer.getFPGATimestamp();
			TalonFXSimState driveMotor = module.getDriveMotor().getSimState();
			driveMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
			module.SimDriveMotor.setInputVoltage(addFriction(driveMotor.getMotorVoltage(), 0.25));
			module.SimDriveMotor.update(ts - sim_last_time);
			driveMotor.setRawRotorPosition(module.SimDriveMotor.getAngularPositionRotations() * SwerveConstants.kRotationsPerWheelRotation);
			driveMotor.setRotorVelocity(module.SimDriveMotor.getAngularVelocityRPM() / 60 * SwerveConstants.kRotationsPerWheelRotation);
			sim_last_time = ts;
		}
	}

	/**
     * Applies the effects of friction to dampen the motor voltage.
     *
     * @param motorVoltage Voltage output by the motor
     * @param frictionVoltage Voltage required to overcome friction
     * @return Friction-dampened motor voltage
     */
    protected double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }

	/**
	 * Method to drive the robot using percentages of max speeds (from -1.0 to 1.0)
	 * @param xPercent Speed of the robot in the x direction (forward).
	 * @param yPercent Speed of the robot in the y direction (sideways).
	 * @param rotPercent Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 */
	public void drivePercentage(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
		double invert = (RobotContainer.isAllianceRed()) ? -1 : 1; //invert if red
		double xSpeed = invert * -xPercent * DriveTrainConstants.kMaxMetersPerSecond; //positive is away
		double ySpeed = invert * yPercent * DriveTrainConstants.kMaxMetersPerSecond; //positive is left
		double rot = rotPercent * DriveTrainConstants.kMaxRotationRadiansPerSecond;
		drive(xSpeed, ySpeed, rot, fieldRelative);
	}

	/**
	 * Method to drive the robot using calculated speed info.
	 * @param xSpeed Speed of the robot in the x direction (forward).
	 * @param ySpeed Speed of the robot in the y direction (sideways).
	 * @param rot Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 */
	// @SuppressWarnings("unused")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		ChassisSpeeds speeds = ChassisSpeeds.discretize(
			(fieldRelative) 
				? ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, RobotContainer.gyro.getYaw()) 
				: new ChassisSpeeds(xSpeed, ySpeed, rot),
			Robot.kDefaultPeriod);
			driveRelative(speeds);
	}
	/**
	 * Method to drive the robot using calculated speed info.
	 * @param speeds ChassisSpeeds object representing the x,y and rotational speed of the robot
	 */
	@SuppressWarnings("unused")
	public void driveRelative(ChassisSpeeds speeds) {
		if (DriveTrainConstants.useBrakeWhenStopped && (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0)) {
			brake(false);
		}
		SwerveModuleState[] swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
		if(!SwerveConstants.FL.isDisabled) m_frontLeft.setDesiredState(swerveModuleStates[0]);
		if(!SwerveConstants.FR.isDisabled) m_frontRight.setDesiredState(swerveModuleStates[1]);
		if(!SwerveConstants.BL.isDisabled) m_backLeft.setDesiredState(swerveModuleStates[2]);
		if(!SwerveConstants.BR.isDisabled) m_backRight.setDesiredState(swerveModuleStates[3]);
	}

	public void autonDriveRelative(ChassisSpeeds speeds) {
		speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond); //correct path planner inversions
		// Helpers.Debug.debug("Auton Driving: X: "+ speeds.vxMetersPerSecond + " Y: "+ speeds.vyMetersPerSecond + " Omega: " + speeds.omegaRadiansPerSecond);
		SwerveModuleState[] swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);
	}

	//Stops all modules
	public void brake(boolean withDefensiveLock) {
		if(withDefensiveLock) {
			m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)));
			m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315.0)));
			m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135.0)));
			m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(225.0)));
		} else {
			for (SwerveModule module: modules) {
				module.setDesiredState(new SwerveModuleState(0, module.getState().angle));
			}
		}
	}

	public SwerveModulePosition[] getSwerveModulePositions() {
		// SwerveModulePosition[] positions = new SwerveModulePosition[4];
		// for (SwerveModule module: modules) {
		// 	positions[module.ID]=module.getPosition();
		// }
		// return positions;
		return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
		};
	}

	public ChassisSpeeds getSpeeds() {
		return SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
	}

	public SwerveModuleState[] getModuleStates() {
		// SwerveModuleState[] states = new SwerveModuleState[4];
		// for (SwerveModule module: modules) {
		// 	states[module.ID] = module.getState();
		// }
		// return states;
		return new SwerveModuleState[] {
			m_frontLeft.getState(),
			m_frontRight.getState(),
			m_backLeft.getState(),
			m_backRight.getState()
		};
	}

	/**
	 * Sets the swerve ModuleStates.
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_backLeft.setDesiredState(desiredStates[2]);
		m_backRight.setDesiredState(desiredStates[3]);
	}

	/** Resets the drive encoders to read a position of 0. */
	public void resetEncoders() {
		for (SwerveModule module: modules) {
			module.resetEncoders();
		}
	}

	/** Resets the drive distances to read 0. */
	public void resetDistances() {
		for (SwerveModule module: modules) {
			module.resetDistance();
		}
	}

	/** Moves the swerve modules to their 0 position (in current loop). */
	public void homeSwerves() {
		for (SwerveModule module: modules) {
			module.homeSwerve();
		}
	}

	public boolean swervesAtHome() {
		boolean home = true;
		for (SwerveModule module: modules) {
			home &= module.getTurnError() <= SwerveConstants.kDefaultModuleTurnAllowableError;
		}
		return home;
	}

	//#region MOTOR CONTROLLER STUFF
	public void setAllDriveBrakeMode(boolean b) {
		for (SwerveModule module: modules) {
			module.setBrakeMode("drive",b);
		}
	}

	public void setAllTurnBrakeMode(boolean b) {
		for (SwerveModule module: modules) {
			module.setBrakeMode("turn", b);
		}
	}
	//#endregion MOTOR CONTROLLER STUFF

	public TalonFX[] getMotors() {
	    ArrayList<TalonFX> motors = new ArrayList<>();
		for (SwerveModule module: modules) {
			motors.add(module.getDriveMotor());
		}
		return motors.toArray(new TalonFX[motors.size()]);
	}

	// PathPlanner

	// follow named path
	public Command followPathCommand(String pathName) {
		PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
		return AutoBuilder.followPath(path);
	}
}