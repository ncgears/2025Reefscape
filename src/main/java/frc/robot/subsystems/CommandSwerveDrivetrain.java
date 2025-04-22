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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        publishData();
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
        publishData();
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
        publishData();
    }

    public void init() {
        m_suppressFrontVision = false;
        m_suppressBackVision = false;
        NCDebug.Debug.debug("Drivetrain: Initialized");
    }

    public void publishData() {
      SmartDashboard.putData("Swerve Drive", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
          builder.setSmartDashboardType("SwerveDrive");
          builder.addDoubleProperty("Front Left Angle", () -> getState().ModuleStates[0].angle.getRadians(), null);
          builder.addDoubleProperty("Front Left Velocity", () -> getState().ModuleStates[0].speedMetersPerSecond, null);
          builder.addDoubleProperty("Front Right Angle", () -> getState().ModuleStates[1].angle.getRadians(), null);
          builder.addDoubleProperty("Front Right Velocity", () -> getState().ModuleStates[1].speedMetersPerSecond, null);
          builder.addDoubleProperty("Back Left Angle", () -> getState().ModuleStates[2].angle.getRadians(), null);
          builder.addDoubleProperty("Back Left Velocity", () -> getState().ModuleStates[2].speedMetersPerSecond, null);
          builder.addDoubleProperty("Back Right Angle", () -> getState().ModuleStates[3].angle.getRadians(), null);
          builder.addDoubleProperty("Back Right Velocity", () -> getState().ModuleStates[3].speedMetersPerSecond, null);
          builder.addDoubleProperty("Robot Angle", () -> getBotHeading().getRadians(), null);
        }      
      });
      SmartDashboard.putData("theta Controller", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
          builder.addStringProperty("Heading Lock", () -> getHeadingLockedColor(), null);
          builder.addDoubleProperty("Target Heading", () -> NCDebug.General.roundDouble(getTargetHeading(),4), null);
          builder.addDoubleProperty("Current Heading", () -> NCDebug.General.roundDouble(getBotHeading().getDegrees(),4), null);
          builder.addDoubleProperty("Heading Error", () -> NCDebug.General.roundDouble(getHeadingError().getDegrees(),4), null);
        }      
      });
      SmartDashboard.putData("Field", field);
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

  public boolean getHeadingLocked() { return (RobotContainer.m_targetLock != null) ? RobotContainer.m_targetLock : false; }
	public String getHeadingLockedColor() {
		return (getHeadingLocked()) ?	DashboardConstants.Colors.GREEN	: DashboardConstants.Colors.RED;
	}
	public double getTargetHeading() { return (RobotContainer.m_targetDirection != null) ? RobotContainer.m_targetDirection.getDegrees() : 0; }
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
