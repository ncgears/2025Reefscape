package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.NCDebug;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
@SuppressWarnings({"unused"})
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    public Field2d field = new Field2d();
	
    private boolean m_suppressFrontVision = false;
    private boolean m_suppressBackVision = false;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following 
     * These are used for PathPlanner
    */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /** Swerve request to apply during field-centric path following 
     * These are used for Choreo
    */
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController m_pathXController = new PIDController(7,0, 0);
    private final PIDController m_pathYController = new PIDController(7, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(8, 0, 0);

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        init();
        createDashboards();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        init();
        createDashboards();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        init();
        createDashboards();
    }

    public void init() {
        NCDebug.Debug.debug("Drivetrain: Initialized");
    }

   	public void createDashboards() {
		ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
		// driverTab.add("Swerve Drive", this)
		// 	.withSize(4, 4)
		// 	.withPosition(20, 5)
		// 	.withProperties(Map.of("show_robot_rotation","true"));
		// driverTab.add("Field", getField())
		// 	.withSize(17,9)
		// 	.withPosition(8,0)
		// 	.withWidget("Field")
		// 	.withProperties(Map.of("field_game","Crescendo","robot_width",Units.inchesToMeters(Global.kBumperWidth),"robot_length",Units.inchesToMeters(Global.kBumperLength)));

		// ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
		// // swerveTab.add("Swerve Drive", null)
		// // 	.withSize(6, 6)
		// // 	.withPosition(0, 0)
		// // 	.withProperties(Map.of("show_robot_rotation","true"));
		// swerveTab.addNumber("FL Angle", () -> NCDebug.General.roundDouble(getState().ModuleStates[0].angle.getDegrees(),2))
		// 	.withSize(2, 2)
		// 	.withPosition(6, 0);
		// swerveTab.addNumber("FR Angle", () -> NCDebug.General.roundDouble(getState().ModuleStates[1].angle.getDegrees(),2))
		// 	.withSize(2, 2)
		// 	.withPosition(12, 0);
		// swerveTab.addNumber("BL Angle", () -> NCDebug.General.roundDouble(getState().ModuleStates[2].angle.getDegrees(),2))
		// 	.withSize(2, 2)
		// 	.withPosition(6, 4);
		// swerveTab.addNumber("BR Angle", () -> NCDebug.General.roundDouble(getState().ModuleStates[3].angle.getDegrees(),2))
		// 	.withSize(2, 2)
		// 	.withPosition(12, 4);
		// swerveTab.addNumber("FL Speed", () -> NCDebug.General.roundDouble(getState().ModuleStates[0].speedMetersPerSecond,3))
		// 	.withSize(2, 2)
		// 	.withPosition(8, 1);
		// swerveTab.addNumber("FR Speed", () -> NCDebug.General.roundDouble(getState().ModuleStates[1].speedMetersPerSecond,3))
		// 	.withSize(2, 2)
		// 	.withPosition(10, 1);
		// swerveTab.addNumber("BL Speed", () -> NCDebug.General.roundDouble(getState().ModuleStates[2].speedMetersPerSecond,3))
		// 	.withSize(2, 2)
		// 	.withPosition(8, 3);
		// swerveTab.addNumber("BR Speed", () -> NCDebug.General.roundDouble(getState().ModuleStates[3].speedMetersPerSecond,3))
		// 	.withSize(2, 2)
		// 	.withPosition(10, 3);
		// // swerveTab.add("Field", getField())
		// // 	.withSize(6,4)
		// // 	.withPosition(0,6)
		// // 	.withWidget("Field")
		// // 	.withProperties(Map.of("field_game","Crescendo","robot_width",Units.inchesToMeters(Global.kBumperWidth),"robot_length",Units.inchesToMeters(Global.kBumperLength)));

		// // ShuffleboardLayout thetaList = swerveTab.getLayout("theta Controller", BuiltInLayouts.kList)
		// // 	.withSize(4,4)
		// // 	.withPosition(6,6)
		// // 	.withProperties(Map.of("Label position","LEFT"));
		// // thetaList.addString("Heading Lock", this::getHeadingLockedColor)
		// // 	.withWidget("Single Color View");
		// // thetaList.addNumber("Target Heading", () -> NCDebug.General.roundDouble(getTargetHeading(),4));
		// // thetaList.addNumber("Current Heading", () -> NCDebug.General.roundDouble(getHeading().getDegrees(),4));
		// // thetaList.addNumber("Heading Error", () -> NCDebug.General.roundDouble(getHeadingError(),4));

		ShuffleboardTab systemTab = Shuffleboard.getTab("System");
		systemTab.add("Field", getField())
			.withSize(4,10)
			.withPosition(4,0)
			.withWidget("Field")
			.withProperties(Map.of(
                "field_game","Reefscape",
                "robot_width",Units.inchesToMeters(GlobalConstants.kBumperWidth),
                "robot_length",Units.inchesToMeters(GlobalConstants.kBumperLength),
                "robot_color","0xff0000ff",
                "field_rotation",RobotContainer.isAllianceRed()?90.0:270.0
            ));
		ShuffleboardLayout systemThetaList = systemTab.getLayout("theta Controller", BuiltInLayouts.kList)
			.withSize(4,5)
			.withPosition(0,4)
			.withProperties(Map.of("Label position","LEFT"));
		systemThetaList.addString("Heading Lock", this::getHeadingLockedColor)
			.withWidget("Single Color View");
		systemThetaList.addNumber("Target Heading", () -> NCDebug.General.roundDouble(getTargetHeading(),4));
		systemThetaList.addNumber("Current Heading", () -> NCDebug.General.roundDouble(getBotHeading().getDegrees(),4));
		systemThetaList.addNumber("Heading Error", () -> NCDebug.General.roundDouble(getHeadingError().getDegrees(),4));

		if(SwerveConstants.debugDashboard) {
		}

	}

  public Pose2d getBotPose() {
    return getState().Pose;
  }
  public Rotation2d getBotHeading() {
    return getBotPose().getRotation();
  }
  public Rotation2d getHeadingError() {
    if(!getHeadingLocked()) return Rotation2d.kZero;
    return getBotHeading().minus(RobotContainer.m_targetDirection);
  }

	public void setSuppressFrontVision(boolean suppress) { 
		m_suppressFrontVision = suppress; 
		NCDebug.Debug.debug((m_suppressFrontVision) ? "Drive: Front Vision Suppressed" : "Drive: Front Vision Unsuppressed");
	}
	public Command suppressFrontVisionC() {
		return runOnce(() -> setSuppressFrontVision(true));
	}
	public Command unsuppressFrontVisionC() {
		return runOnce(() -> setSuppressFrontVision(false));
	}
	public void setSuppressBackVision(boolean suppress) { 
		m_suppressBackVision = suppress; 
		NCDebug.Debug.debug((m_suppressBackVision) ? "Drive: Back Vision Suppressed" : "Drive: Back Vision Unsuppressed");
	}
	public Command suppressBackVisionC() {
		return runOnce(() -> setSuppressBackVision(true));
	}
	public Command unsuppressBackVisionC() {
		return runOnce(() -> setSuppressBackVision(false));
	}
	public void autoSuppressVision() {
		if(VisionConstants.kUseAutoSuppress) {
			// ChassisSpeeds speeds = getState().ChassisSpeeds;
			// //if the speed is over threshold, suppress vision measurements from being added to pose
			// m_suppressFrontVision = (
			// 	Math.sqrt(
			// 		Math.pow(speeds.vxMetersPerSecond,2) + 
			// 		Math.pow(speeds.vyMetersPerSecond,2)
			// 	) >= VisionConstants.kAutosuppressSpeedMetersPerSecond);
		}
	}
	public boolean isFrontVisionSuppressed() { return m_suppressFrontVision; }
	public boolean isBackVisionSuppressed() { return m_suppressBackVision; }

  public boolean getHeadingLocked() { return RobotContainer.m_targetLock; }
	public String getHeadingLockedColor() {
		return (getHeadingLocked()) ?	DashboardConstants.Colors.GREEN	: DashboardConstants.Colors.RED;
	}
	public double getTargetHeading() { return RobotContainer.m_targetDirection.getDegrees(); }
	public boolean isTrackingTarget() { return RobotContainer.targeting.getTracking(); }
	public double getTrackingTargetHeading() { 
		return Rotation2d.fromDegrees(RobotContainer.targeting.getTrackingTargetBearing()).getDegrees(); 
	}

	public Field2d getField() {
		return field;
	}

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this,
            trajLogger
        );
    }    

    /**
     * Returns an array of TalonFX motors available for the orchestra
     * @return Array TalonFX[] of TalonFX devices
     */
    public TalonFX[] getMotors() {
	    ArrayList<TalonFX> motors = new ArrayList<>();
		for (var module: getModules()) {
			motors.add(module.getDriveMotor());
            motors.add(module.getSteerMotor());
		}
		return motors.toArray(new TalonFX[motors.size()]);
	}

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

	public SwerveModulePosition[] getSwerveModulePositions() {
		// SwerveModulePosition[] positions = new SwerveModulePosition[4];
		// for (SwerveModule module: modules) {
		// 	positions[module.ID]=module.getPosition();
		// }
		// return positions;
        return getState().ModulePositions;
	}

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        if(!m_suppressFrontVision) {
          RobotContainer.vision.correctPoseWithVision();
        }
        // if(!m_suppressBackVision) {
        //   RobotContainer.vision.correctPoseWithVision();
        // }
        field.setRobotPose(this.getState().Pose);
    }

    public Command resetGyroC() {
      return runOnce(() -> {
        seedFieldCentric();
        NCDebug.Debug.debug("Drive: Reset Gyro");
      });
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
