package frc.robot.classes;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.*; 
import frc.robot.utils.NCDebug;
import frc.robot.RobotContainer;

/**
 * The NCPose class handles getting and managing the different poses and calculations of them.
 * It is responsible for maintaining the robot poses and exposing methods to calculate items related to the poses
 */
public class Targeting {
	private static Targeting instance;

	/**
	 * Targets represents different locations on the field that we might be interested in tracking
	 */
	private static final double m_fieldLength = VisionConstants.kTagLayout.getFieldLength();
    public enum Targets { //based on blue origin 0,0 (blue driver station, right corner)
      HP_RIGHT(0.851154,0.65532,1.4859,54),
      HP_LEFT(0.851154,7.39648,1.4859,306),
      REEF_FRONT_RIGHT_R(4.2158,3.2234,0,240),
      REEF_FRONT_RIGHT_C(4.073906,3.306318,0.308102,240),
      REEF_FRONT_RIGHT_L(3.9312,3.3877,0,240),
      REEF_FRONT_CENTER_R(3.6576,3.8616,0,180),
      REEF_FRONT_CENTER_C(3.6576,4.0259,0.308102,180),
      REEF_FRONT_CENTER_L(3.6576,4.1902,0,180),
      REEF_FRONT_LEFT_R(3.9312,4.664,0,120),
      REEF_FRONT_LEFT_C(4.073906,4.745482,0.308102,120),
      REEF_FRONT_LEFT_L(4.2158,4.8283,0,120),
      REEF_BACK_RIGHT_R(4.7629,3.2234,0,300),
      REEF_BACK_RIGHT_C(4.90474,3.306318,0.308102,300),
      REEF_BACK_RIGHT_L(5.0475,3.3877,0,300),
      REEF_BACK_CENTER_R(5.321046,3.8616,0,0),
      REEF_BACK_CENTER_C(5.321046,4.0259,0.308102,0),
      REEF_BACK_CENTER_L(5.321046,4.1902,0,0),
      REEF_BACK_LEFT_R(5.0475,4.664,0,60),
      REEF_BACK_LEFT_C(4.90474,4.745482,0.308102,60),
      REEF_BACK_LEFT_L(4.7629,4.8283,0,60),
      PROCESSOR(5.969,-0.00381,1.30175,90),
      SPIKE_LEFT(1.2192,5.8547,0,-1),
      SPIKE_CENTER(1.2192,4.0259,0,-1),
      SPIKE_RIGHT(1.2192,2.1971,0,-1),
      BARGE_CAGE_RIGHT(8.7741,5.0784,0,-1),
      BARGE_CAGE_CENTER(8.7741,6.169,0,-1),
      BARGE_CAGE_LEFT(8.7741,7.2596,0,-1);
      private final double x,y,z,angle;
      Targets(double x, double y, double z, double angle) { this.x=x; this.y=y; this.z=z; this.angle=angle; }
      public Rotation2d getRawAngle() { return Rotation2d.fromDegrees(this.angle); }
      public Rotation2d getAngle(boolean redOrigin) { return (redOrigin) ? getRawAngle() : getRawAngle().rotateBy(Rotation2d.k180deg); }
      public Rotation2d getMirrorAngle(boolean redOrigin) { return (redOrigin) ? getRawAngle().unaryMinus() : getRawAngle().unaryMinus().rotateBy(Rotation2d.k180deg); }
      public Pose3d getPose() { return new Pose3d(
        new Translation3d(this.x, this.y, this.z),
  			new Rotation3d()
          // new Rotation3d(0,0,Math.toRadians(this.angle))
          ); 
      }
      public Pose3d getMirrorPose() { return new Pose3d(
        new Translation3d(m_fieldLength - this.x, this.y, this.z),
			  new Rotation3d()
          // new Rotation3d(0,0,Math.PI - Math.toRadians(this.angle))
        );
      }
    }
	/** State represents different tracking system states */
    public enum State {
        READY(DashboardConstants.Colors.GREEN),
        TRACKING(DashboardConstants.Colors.ORANGE),
        ERROR(DashboardConstants.Colors.RED),
        STOP(DashboardConstants.Colors.BLACK);
        private final String color;
        State(String color) { this.color = color; }
        public String getColor() { return this.color; }
    }
    private State m_trackingState = State.STOP; //current Tracking state
    private Targets m_trackingTarget = Targets.HP_LEFT; //current Tracking target
    // private Pose3d m_shooterPose = new Pose3d();
    // private boolean m_adjustUp = false;
    public final Trigger isTracking = new Trigger(() -> { return (m_trackingState==State.READY || m_trackingState==State.TRACKING); });
    public final Trigger isReady = new Trigger(() -> { return (m_trackingState==State.READY); });
    public final PIDController thetaController = new PIDController(ThetaConstants.kP, ThetaConstants.kI, ThetaConstants.kP, ThetaConstants.kIZone);
  
    public Targeting() {
		init();
		createDashboards();
    }

	public void init() {
		m_trackingState = State.STOP;
		m_trackingTarget = Targets.HP_LEFT;
		resetPose(
			(RobotContainer.isAllianceRed()) //more realistic starting position, center on black line
				? new Pose2d(m_fieldLength - 7.2439,4.0082,Rotation2d.k180deg) 
				: new Pose2d(7.2439,4.0082,Rotation2d.kZero)
		);
		NCDebug.Debug.debug("Pose: Initialized");
	}

    /**
	 * Returns the instance of the class.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return instance of this class
	 */
    public static Targeting getInstance() {
		if (instance == null)
			instance = new Targeting();
		return instance;
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 * @return The pose.
	 */
	public Supplier<Pose2d> getPose() {
		return () -> RobotContainer.drivetrain.getState().Pose;
	}

  public Rotation2d getRobotHeading() {
    return getPose().get().getRotation().rotateBy(new Rotation2d(Math.PI));
  }

	public double getBearingOfTarget(Targets target) {
		return 0.0;
		//RobotContainer.drivetrain.getTargetHeading();
	}

	public double getDistanceOfTarget(Targets target) {
		return 0.0;
	}

	public Rotation2d getAngleOfTarget(Targets target) {
    boolean red = RobotContainer.isAllianceRed();
    Rotation2d perspective = (red) ? Rotation2d.k180deg : Rotation2d.kZero;
    return (red) ? target.getAngle(red).minus(perspective) : target.getMirrorAngle(red).unaryMinus().minus(perspective);
    // return target.getRawAngle();
	}



	// /**
    //  * Reset the estimated pose of the swerve drive on the field.
    //  *
	//  * @param heading Heading to reset robot to (for configuring a yaw offset)
    //  * @param pose New robot pose.
    //  */
	// public void resetPose(double heading, Pose2d pose) {
	// 	poseEstimator.resetPosition(RobotContainer.gyro.getYaw(), RobotContainer.drivetrain.getSwerveModulePositions(), pose);
	// }
	/**
     * Reset the estimated pose of the swerve drive on the field.
     *
     * @param pose New robot pose.
     */
	public void resetPose(Pose2d pose) {
		RobotContainer.drivetrain.resetPose(pose);
	}

	////#region "Tracking"
	/** Creates the dashboard for the tracking system */
	public void createDashboards() {
		// if(true) { //false to disable tracking dashboard
		// 	ShuffleboardTab systemTab = Shuffleboard.getTab("System");
		// 	systemTab.addNumber("Bot Pose Hdg", () -> NCDebug.General.roundDouble(getPose().get().getRotation().getDegrees(),2))
		// 		.withSize(4,2)
		// 		.withPosition(0,4);
		// 	// systemTab.addNumber("Shooter Hdg", () -> NCDebug.General.roundDouble(getPose().get().rotateBy(new Rotation2d(Math.PI)).getRotation().getDegrees(),2))
		// 	// 	.withSize(4,2)
		// 	// 	.withPosition(4,2);
		// 	ShuffleboardLayout trackingList = systemTab.getLayout("Target Tracking", BuiltInLayouts.kList)
		// 		.withSize(4,5)
		// 		.withPosition(12,5)
		// 		.withProperties(Map.of("Label position","LEFT"));
		// 	trackingList.addString("Tracking", this::getTrackingStateColor)
		// 		.withWidget("Single Color View");
		// 	trackingList.addString("State", this::getTrackingStateName)
		// 		.withWidget("Text Display");
		// 	trackingList.addString("Target", this::getTrackingTargetName)
		// 		.withWidget("Text Display");
		// 	trackingList.addNumber("Bearing", this::getTrackingTargetBearing);
		// 	trackingList.addNumber("Distance", this::getTrackingTargetDistance);
		// 	trackingList.addNumber("Angle", this::getTrackingTargetAngle);
		// }
	}
	/** Determines if the robot should be tracking a target
	 * @return boolean indicating if robot is tracking
	 */
	public boolean getTracking() { return m_trackingState != State.STOP; }
	/** Gets the current target tracking state */
	public State getTrackingState() { return m_trackingState; }
	/** Gets the name of the current target tracking state */
	public String getTrackingStateName() { return m_trackingState.toString(); }
	/** Gets the defined color of the current target tracking state */
	public String getTrackingStateColor() { return m_trackingState.getColor(); }
	/** Gets the current tracking target */
	public Targets getTrackingTarget() { return m_trackingTarget; }
	/** Gets the name of the current tracking target */
	public String getTrackingTargetName() { return m_trackingTarget.toString(); }
	/** Gets the relative bearing of the current tracking target */
	public double getTrackingTargetBearing() { return NCDebug.General.roundDouble(getBearingOfTarget(m_trackingTarget),2); }
	/** Gets the relative distance from the shooter to the current tracking target */
	public double getTrackingTargetDistance() { return NCDebug.General.roundDouble(getDistanceOfTarget(m_trackingTarget),2); }
	/** Gets the relative angle from the shooter to the current tracking target */
	public double getTrackingTargetAngle() { return NCDebug.General.roundDouble(getAngleOfTarget(m_trackingTarget).getDegrees(),2); }
	/** Gets the relative angle from the shooter to the current tracking target as rotations from 0.0 */
	public double getTrackingTargetAngleAsRotations() { return getAngleOfTarget(m_trackingTarget).getRotations(); } //.plus(getGravityAdjustmentOfTarget(m_trackingTarget))
	/** Enables target tracking */
	public void trackingStart() {
		m_trackingState = State.TRACKING;
		NCDebug.Debug.debug("Tracking: Start Tracking ("+m_trackingTarget.toString()+")");
    }
	/** Disables target tracking */
    public void trackingStop() {
        m_trackingState = State.STOP;
		// RobotContainer.drivetrain.lockHeading();
		NCDebug.Debug.debug("Tracking: Stop Tracking");
	}
	/** Sets the requested tracking target
	 * @param target Target to track
	 */
	public void setTrackingTarget(Targets target) { 
		m_trackingTarget = target; 
		NCDebug.Debug.debug("Tracking: Set Tracking Target ("+target.toString()+")");
	}
	/** Sets the tracking target to Reef Front CENTER_C (2025 REEFSCAPE) */
	public Command setTrackingRFCC() { return new InstantCommand(() -> setTrackingTarget(Targets.REEF_FRONT_CENTER_C)); }
	/** Sets the tracking target to Reef Front LEFT_C (2025 REEFSCAPE) */
	public Command setTrackingRFLC() { return new InstantCommand(() -> setTrackingTarget(Targets.REEF_FRONT_LEFT_C)); }
	/** Sets the tracking target to Reef Front RIGHT_C (2025 REEFSCAPE) */
	public Command setTrackingRFRC() { return new InstantCommand(() -> setTrackingTarget(Targets.REEF_FRONT_RIGHT_C)); }
	/** Sets the tracking target to Reef Back CENTER_C (2025 REEFSCAPE) */
	public Command setTrackingRBCC() { return new InstantCommand(() -> setTrackingTarget(Targets.REEF_BACK_CENTER_C)); }
	/** Sets the tracking target to Reef Back LEFT_C (2025 REEFSCAPE) */
	public Command setTrackingRBLC() { return new InstantCommand(() -> setTrackingTarget(Targets.REEF_BACK_LEFT_C)); }
	/** Sets the tracking target to Reef Back RIGHT_C (2025 REEFSCAPE) */
	public Command setTrackingRBRC() { return new InstantCommand(() -> setTrackingTarget(Targets.REEF_BACK_RIGHT_C)); }
	/** Sets the tracking target to Reef HP LEFT (2025 REEFSCAPE) */
	public Command setTrackingHPL() { return new InstantCommand(() -> setTrackingTarget(Targets.HP_LEFT)); }
	/** Sets the tracking target to Reef HP RIGHT (2025 REEFSCAPE) */
	public Command setTrackingHPR() { return new InstantCommand(() -> setTrackingTarget(Targets.HP_RIGHT)); }
	public void setTrackingReady(boolean ready) {
		m_trackingState = (ready) ? State.READY : State.TRACKING;
	}
	////#endregion "Tracking"

}
