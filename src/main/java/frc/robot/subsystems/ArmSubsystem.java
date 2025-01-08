
package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.*;
import frc.robot.utils.NCDebug;
import frc.robot.utils.NCDebug.Debug;
import frc.robot.RobotContainer;

/**
 * This subsystem handles managing the Arm.
 * It is responsible for moving the arm with the indexer for scoring trap and amp notes.
 */
public class ArmSubsystem extends SubsystemBase {
	private static ArmSubsystem instance;
  //private and public variables defined here
  public enum State {
    UP(DashboardConstants.Colors.GREEN),
    DOWN(DashboardConstants.Colors.RED),
    HOLD(DashboardConstants.Colors.ORANGE),
    STOP(DashboardConstants.Colors.BLACK);
    private final String color;
    State(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  public enum Position {
    INTAKE(ArmConstants.Positions.kIntake),
    AMP(ArmConstants.Positions.kAmp),
    TRAP(ArmConstants.Positions.kTrap),
    TRAPBAL(ArmConstants.Positions.kTrapBalance),
    TRAPCLIMB(ArmConstants.Positions.kTrapClimb);
    private final double position;
    Position(double position) { this.position = position; }
    public double getAngularPositionRotations() { return this.position; }
  }

  public final Trigger atTarget = new Trigger(this::atSetpoint);
  public final Trigger atIntake = new Trigger(this::atIntake);
  public final Trigger atTrap = new Trigger(this::atTrap);
  public final Trigger atAmp = new Trigger(this::atAmp);

  private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0);
  // private final PositionVoltage m_voltagePosition = new PositionVoltage(0);
  private CANcoder m_encoder;
  private TalonFX m_motor1;
  private State m_curState = State.STOP;
  private Position m_targetPosition = Position.INTAKE;
  
  /**
	 * Returns the instance of the ArmSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return ArmSubsystem instance
	 */
  public static ArmSubsystem getInstance() {
		if (instance == null)
			instance = new ArmSubsystem();
		return instance;
	}
  
  public ArmSubsystem() {
    //initialize values for private and public variables, etc.
    m_encoder = new CANcoder(ArmConstants.kCANcoderID, ArmConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.armCCConfig));

    m_motor1 = new TalonFX(ArmConstants.kMotorID, ArmConstants.canBus);
    m_motor1.setInverted(ArmConstants.kIsInverted);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.armFXConfig));

    init();
    createDashboards();
  }
  
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    m_curState = State.STOP;
    m_targetPosition = Position.INTAKE;
    NCDebug.Debug.debug("Arm: Initialized");
  }
  
  @Override
  public void periodic() {
    // updateState();
  }

  // @Override
  // public void initSendable(SendableBuilder builder) {
  //   super.initSendable(builder);
  //   builder.setSmartDashboardType("Number Slider");
  //   builder.setActuator(true);
  //   builder.addDoubleProperty("Target Speed", this::getTargetSpeed, this::setSpeedPercent);
  //   builder.addDoubleProperty("Current Speed", this::getSpeedPercent, null);
  // }

  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Arm", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(16, 7);  

    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout armList = systemTab.getLayout("Arm", BuiltInLayouts.kList)
      .withSize(4,6)
      .withPosition(20,4)
      .withProperties(Map.of("Label position","LEFT"));
    // armList.addString("Status", this::getColor)
    //   .withWidget("Single Color View");
    armList.addBoolean("Status", this::atSetpoint);
    // armList.addString("State", this::getStateName);
    armList.addString("Target", this::getTargetPositionName);
    armList.addNumber("Position", () -> NCDebug.General.roundDouble(getPosition(),7));
    armList.addNumber("Absolute", () -> NCDebug.General.roundDouble(getPositionAbsolute(),7));
    armList.addNumber("Error", () -> NCDebug.General.roundDouble(getPositionError(),7));

    if(ArmConstants.debugDashboard) {
    }
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getColor() { return m_curState.getColor(); }
  public String getTargetPositionName() { return m_targetPosition.toString(); }
  public double getTargetPosition() { return m_motor1.getClosedLoopReference().getValue(); }
  public double getPositionError() { return m_motor1.getClosedLoopError().getValue(); }
  public boolean atSetpoint() { return (Math.abs(m_motor1.getClosedLoopError().getValue()) <= ArmConstants.kPositionThreshold); }
  // public boolean atSetpoint() { return MathUtil.isNear(m_motor1.getClosedLoopReference().getValue(), m_motor1.getPosition().getValue(), ArmConstants.kPositionThreshold); }
  public boolean atIntake() { return m_targetPosition==Position.INTAKE && atSetpoint(); }
  public boolean atAmp() { return m_targetPosition==Position.AMP && atSetpoint(); }
  public boolean atTrap() { return m_targetPosition==Position.TRAP && atSetpoint(); }

  public double getPosition() {
    return m_motor1.getPosition().getValue();
  }

  public double getPositionAbsolute() {
    return m_encoder.getPosition().getValue();
  }

  public void setPosition(Position position) {
    m_targetPosition = position;
    m_motor1.setControl(m_mmVoltage.withPosition(position.getAngularPositionRotations()));
  }


  public void armIntake() {
    setPosition(Position.INTAKE);
    NCDebug.Debug.debug("Arm: Intake Position");
  }
  public void armAmp() {
    setPosition(Position.AMP);
    NCDebug.Debug.debug("Arm: Amp Position");
  }
  public void armTrapClimb() {
    setPosition(Position.TRAPCLIMB);
    NCDebug.Debug.debug("Arm: TrapClimb Position");
  }
  public void armTrapBalance() {
    setPosition(Position.TRAPBAL);
    NCDebug.Debug.debug("Arm: TrapBalance Position");
  }
  public void armTrap() {
    setPosition(Position.TRAP);
    NCDebug.Debug.debug("Arm: Trap Position");
  }
  public void armUp() {
    m_curState = State.UP;
    NCDebug.Debug.debug("Arm: Up");
  }
  public void armDown() {
    m_curState = State.DOWN;
    NCDebug.Debug.debug("Arm: Down");
  }
  public void armHold() {
    m_curState = State.HOLD;
    NCDebug.Debug.debug("Arm: Hold");
  }
  public void armStop() {
    m_curState = State.STOP;
    NCDebug.Debug.debug("Arm: Stop");
  }

  public void setCoast() {
    m_motor1.setNeutralMode(NeutralModeValue.Coast);
    NCDebug.Debug.debug("Arm: Switch to Coast");
  }

  public void setBrake() {
    m_motor1.setNeutralMode(NeutralModeValue.Brake);
    NCDebug.Debug.debug("Arm: Switch to Brake");
  }
}
