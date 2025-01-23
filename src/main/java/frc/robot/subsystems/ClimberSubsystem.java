
package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.*; 
import frc.robot.utils.NCDebug;
import frc.robot.RobotContainer;

/**
 * This subsystem handles managing the Climber.
 * It is responsible for extending and retracting the elevator/climber.
 */
public class ClimberSubsystem extends SubsystemBase {
	private static ClimberSubsystem instance;
  //private and public variables defined here

  private DigitalInput m_cageSwitch1 = new DigitalInput(ClimberConstants.kCageSwitch1ID);
  private DigitalInput m_cageSwitch2 = new DigitalInput(ClimberConstants.kCageSwitch2ID);

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
    STOW(ClimberConstants.Positions.kStow),
    DEEPCAPTURE(ClimberConstants.Positions.kDeepCapture),
    DEEPCLIMB(ClimberConstants.Positions.kDeepClimb),
    SHALLOWCAPTURE(ClimberConstants.Positions.kShallowCapture),
    SHALLOWCLIMB(ClimberConstants.Positions.kShallowClimb);
    private final double position;
    Position(double position) { this.position = position; }
    public double getAngularPositionRotations() { return this.position; }
  }
  private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0);
  private final DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();
  private final StaticBrake m_brake = new StaticBrake();
  private CANcoder m_encoder;
  private TalonFX m_motor1;
  private State m_curState = State.STOP;
  private Position m_targetPosition = Position.STOW;
  private boolean m_ratchetLocked = false;

  /**
   * Returns true when the cage switch is engaged
   */
  public final Trigger hasCage = new Trigger(this::hasCage);

  /**
	 * Returns the instance of the ClimberSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return ClimberSubsystem instance
	 */
  public static ClimberSubsystem getInstance() {
		if (instance == null)
			instance = new ClimberSubsystem();
		return instance;
	}
  
  public ClimberSubsystem() {
    //initialize values for private and public variables, etc.
    m_encoder = new CANcoder(ClimberConstants.kCANcoderID, ClimberConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.climberCCConfig));

    m_motor1 = new TalonFX(ClimberConstants.kMotorID, ClimberConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.climberFXConfig));

    init();
    createDashboards();
  }
  
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    m_targetPosition = Position.STOW;
    climberStop();
    NCDebug.Debug.debug("Climber: Initialized");
  }
  
  @Override
  public void periodic() {
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
    driverTab.addString("Climber", this::getStateColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(18, 7);  

    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout climberList = systemTab.getLayout("Climber", BuiltInLayouts.kList)
      .withSize(4,6)
      .withPosition(16,0)
      .withProperties(Map.of("Label position","LEFT"));
    climberList.addString("Status", this::getStateColor)
      .withWidget("Single Color View");
    climberList.addBoolean("Has Note", this::hasCage);
    climberList.addString("State", this::getStateName);
    climberList.addString("Target", this::getTargetPositionName);
    climberList.addNumber("Target Pos", this::getTargetPosition);
    climberList.addNumber("Position", () -> NCDebug.General.roundDouble(getPosition().in(Units.Rotations),7));
    climberList.addNumber("Absolute", () -> NCDebug.General.roundDouble(getPositionAbsolute().in(Units.Rotations),7));
    climberList.addNumber("Error", () -> NCDebug.General.roundDouble(getPositionError(),7));

    if(ClimberConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
			ShuffleboardLayout dbgClimberList = debugTab.getLayout("Climber", BuiltInLayouts.kList)
				.withSize(4,10)
				.withPosition(16,0)
				.withProperties(Map.of("Label position","LEFT"));
      dbgClimberList.addString("Status", this::getStateColor)
        .withWidget("Single Color View");
      dbgClimberList.addBoolean("Has Note", this::hasCage);
      dbgClimberList.addString("State", this::getStateName);
      dbgClimberList.addNumber("Target", this::getTargetPosition);
      dbgClimberList.addNumber("Position", () -> { return getPosition().in(Units.Rotations); });
      dbgClimberList.addNumber("Absolute", () -> { return getPositionAbsolute().in(Units.Rotations); });
      dbgClimberList.addNumber("Error", this::getPositionError);
      dbgClimberList.add("Climber Up", new InstantCommand(this::climberUp))
        .withProperties(Map.of("show_type",false));  
      dbgClimberList.add("Climber Down", new InstantCommand(this::climberDown))
        .withProperties(Map.of("show_type",false));  
      dbgClimberList.add("Climber Hold", new InstantCommand(this::climberHold))
        .withProperties(Map.of("show_type",false));  
      dbgClimberList.add("Climber Stop", new InstantCommand(this::climberStop))
        .withProperties(Map.of("show_type",false));  
    }
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getStateColor() { return m_curState.getColor(); }

  public boolean hasCage() {
    return getCageSwitch1() && getCageSwitch2();
  }

  private boolean getCageSwitch1() {
    return m_cageSwitch1.get();
  }
  private boolean getCageSwitch2() {
    return m_cageSwitch2.get();
  }

  public String getTargetPositionName() { return m_targetPosition.toString(); }
  public double getTargetPosition() { return m_motor1.getClosedLoopReference().getValue(); }
  public double getPositionError() { return m_motor1.getClosedLoopError().getValue(); }

  public Angle getPosition() {
    return m_motor1.getPosition().getValue();
  }

  public Angle getPositionAbsolute() {
    return m_encoder.getPosition().getValue();
  }

  public void setPosition(Position position) {
    // if(getPosition() <= position.getAngularPositionRotations()) ratchetFree();
    // if(getPosition() > position.getAngularPositionRotations()) ratchetLock();
    m_motor1.setControl(m_mmVoltage.withPosition(position.getAngularPositionRotations()));
    NCDebug.Debug.debug("Climber: Move to "+position.toString());
  }

  public boolean getForwardLimit() {
    //if using NormallyOpen, this should be ForwardLimitValue.ClosedToGround
    return m_motor1.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  public boolean getReverseLimit() {
    //if using NormallyOpen, this should be ReverseLimitValue.ClosedToGround
    return m_motor1.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  public void climberMove(double power) {
    if(power>0) {
      if(m_curState != State.UP) {
        NCDebug.Debug.debug("Climber: Up ("+power+")");
        m_curState = State.UP;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else if(power<0) {
      if(m_curState != State.DOWN) {
        NCDebug.Debug.debug("Climber: Down ("+power+")");
        m_curState = State.DOWN;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else { //0 power
      if(m_curState != State.HOLD && m_curState != State.STOP) {
        m_motor1.setControl(m_brake);
        m_curState = State.HOLD;
        NCDebug.Debug.debug("Climber: Hold");
      }
    }
  }

  public Command climberMoveC(DoubleSupplier power) {
    return run(() -> climberMove(power.getAsDouble()));
  }

  public void climberDeepCapture() { setPosition(Position.DEEPCAPTURE); }
  public void climberDeepClimb() { setPosition(Position.DEEPCLIMB); }
  public void climberShallowCapture() { setPosition(Position.SHALLOWCAPTURE); }
  public void climberShallowClimb() { setPosition(Position.SHALLOWCLIMB); }

  public void climberUp() {
    m_curState = State.UP;
    m_motor1.setControl(m_DutyCycle);
    NCDebug.Debug.debug("Climber: Up");
  }
  public void climberDown() {
    m_curState = State.DOWN;
    NCDebug.Debug.debug("Climber: Down");
  }
  public void climberHold() {
    m_motor1.setControl(m_brake);
    if(m_curState != State.HOLD) {
      m_curState = State.HOLD;
      NCDebug.Debug.debug("Climber: Hold");
    }
  }
  public void climberStop() {
    m_motor1.setControl(m_neutral);
    if(m_curState != State.HOLD) {
      m_curState = State.STOP;
      NCDebug.Debug.debug("Climber: Stop");
    }
  }

  public void setCoast() {
    m_motor1.setNeutralMode(NeutralModeValue.Coast);
    NCDebug.Debug.debug("Climber: Switch to Coast");
  }

  public void setBrake() {
    m_motor1.setNeutralMode(NeutralModeValue.Brake);
    NCDebug.Debug.debug("Climber: Switch to Brake");
  }

}
