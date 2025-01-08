
package frc.robot.subsystems;

import java.util.Map;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.*;
import frc.robot.utils.NCDebug;
import frc.robot.RobotContainer;

/**
 * This subsystem handles managing the Aimer.
 * It is responsible for adjusting the vertical aim of the shooter using tracking data from vision.
 */
public class AimerSubsystem extends SubsystemBase {
	private static AimerSubsystem instance;
  //private and public variables defined here
  public enum State {
    READY(DashboardConstants.Colors.GREEN),
    TRACKING(DashboardConstants.Colors.ORANGE),
    ERROR(DashboardConstants.Colors.RED),
    STOP(DashboardConstants.Colors.BLACK);
    private final String color;
    State(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0);
  // private final PositionVoltage m_voltagePosition = new PositionVoltage(0);

  private CANcoder m_encoder;
  private TalonFX m_motor1;
  private State m_curState = State.STOP;
  private Double m_targetPosition = 0.0;
  private boolean m_suppressTracking = true; //do not allow tracking while true
  private DoubleSubscriber new_position_sub;
  private double new_position = 0.0;

  public final Trigger isTracking = new Trigger(() -> { return (m_curState==State.READY || m_curState==State.TRACKING); });
  public final Trigger isReady = new Trigger(() -> { return (m_curState==State.READY); });

  /**
	 * Returns the instance of the AimerSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return AimerSubsystem instance
	 */
  public static AimerSubsystem getInstance() {
		if (instance == null)
			instance = new AimerSubsystem();
		return instance;
	}

  public AimerSubsystem() {
    //initialize values for private and public variables, etc.
    m_encoder = new CANcoder(AimerConstants.kCANcoderID, AimerConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.aimerCCConfig));

    m_motor1 = new TalonFX(AimerConstants.kMotorID, AimerConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.aimerFXConfig));
    // Helpers.Debug.debug("Could not initialize aimer motor, error: " + status1.toString());

    init();
    createDashboards();
  }
  
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    m_curState = State.STOP;
    NCDebug.Debug.debug("Aimer: Initialized");
    setPosition(0.0);
  }

  @Override
  public void periodic() {
    new_position = new_position_sub.get(0.0);
    updateSetpoint();
    updateState();
  }

  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Aimer", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(12, 7);  

    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout aimerList = systemTab.getLayout("Aimer", BuiltInLayouts.kList)
      .withSize(4,5)
      .withPosition(8,3)
      .withProperties(Map.of("Label position","LEFT"));
    aimerList.addString("Status", this::getColor)
      .withWidget("Single Color View");
    aimerList.addString("State", this::getStateName);
    aimerList.addNumber("Target", this::getTargetPosition);
    aimerList.addNumber("Position", () -> NCDebug.General.roundDouble(getPosition(),7));
    aimerList.addNumber("Absolute", () -> NCDebug.General.roundDouble(getPositionAbsolute(),7));
    aimerList.addNumber("Error", () -> NCDebug.General.roundDouble(getPositionError(),7));

		if(AimerConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
      debugTab.addNumber("Aimer Target", this::getNewPosition)
        .withSize(4,2)
        .withPosition(0,0)
        .withWidget("Number Slider")
        .withProperties(Map.of("min_value",0,"max_value",0.125,"divisions",10));

      new_position_sub = NetworkTableInstance.getDefault().getDoubleTopic("/Shuffleboard/Debug/Aimer Target").subscribe(0.0);
    }
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getColor() { return m_curState.getColor(); }
  public double getNewPosition() { return NCDebug.General.roundDouble(new_position,3); }

  public void updateSetpoint() {
    if(!m_suppressTracking) { //allowed to track
      new_position = RobotContainer.pose.getTrackingTargetAngleAsRotations();
      if(new_position != m_targetPosition) setPosition(new_position);
    } else {
      setPosition(m_targetPosition);
    }
  }

  /** Sets the position of the aimer, but is restricted to the safe range defined by
   * Aimer.Positions.kRevLimit and kFwdLimit
   * @param position Position to set aimer in rotations
   */
  public void setPositionRotations(double position) {
    m_targetPosition = MathUtil.clamp(position, AimerConstants.Positions.kRevLimit, AimerConstants.Positions.kFwdLimit);
  }

  public void updateState() {
    if(!m_suppressTracking) m_curState = (atSetpoint()) ? State.READY : State.TRACKING;
  }

  public void setPosition(double position) {
    // m_motor1.setControl(m_voltagePosition.withPosition(position));
    m_targetPosition = position;
    m_motor1.setControl(m_mmVoltage.withPosition(position));
  }

  public void setZero() {
    m_motor1.setPosition(0.0);
  }

  public double getTargetPosition() { return m_motor1.getClosedLoopReference().getValue(); } //m_targetPosition; }
  public double getPositionError() { return m_motor1.getClosedLoopError().getValue(); }
  public boolean atSetpoint() { return (Math.abs(m_motor1.getClosedLoopError().getValue()) <= AimerConstants.kPositionThreshold); }

  public double getPosition() {
    return m_motor1.getPosition().getValue();
  }

  public double getPositionAbsolute() {
    return m_encoder.getPosition().getValue();
  }

  public void aimerAimShort() {
    m_suppressTracking = true;
    m_curState = State.STOP;
    setPositionRotations(AimerConstants.Positions.kShotShort);
    NCDebug.Debug.debug("Aimer: Shot Shot");
  }

  public void aimerAimLong() {
    m_suppressTracking = true;
    m_curState = State.STOP;
    setPositionRotations(AimerConstants.Positions.kShotLong);
    NCDebug.Debug.debug("Aimer: Long Shot");
  }

  public void aimerTrapClimb() {
    m_suppressTracking = true;
    m_curState = State.STOP;
    setPosition(AimerConstants.Positions.kTrapClimb);
    NCDebug.Debug.debug("Aimer: Trap Climb");
  }

  public void aimerStopAndStow() {
    m_suppressTracking = true;
    m_curState = State.STOP;
    NCDebug.Debug.debug("Aimer: Stop and Stow");
    setPosition(AimerConstants.kStowPosition);
  }

  public void aimerStartTracking() {
    m_suppressTracking = false;
    m_curState = State.TRACKING;
    NCDebug.Debug.debug("Aimer: Start Tracking");
  }

}
