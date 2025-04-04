
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.*;
import frc.robot.utils.NCDebug;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * This subsystem handles managing the Climber.
 * It is responsible for extending and retracting the elevator/climber.
 */
public class ClimberSubsystem extends SubsystemBase {
  private static ClimberSubsystem instance;
  // private and public variables defined here

  private DigitalInput m_cageSwitch1 = new DigitalInput(ClimberConstants.kCageSwitch1ID);
  private DigitalInput m_cageSwitch2 = new DigitalInput(ClimberConstants.kCageSwitch2ID);
  private DigitalInput m_climbSwitch = new DigitalInput(ClimberConstants.kClimbSwitchID);

  public enum State {
    UP(DashboardConstants.Colors.ORANGE),
    DOWN(DashboardConstants.Colors.RED),
    HOLD(DashboardConstants.Colors.GREEN),
    STOP(DashboardConstants.Colors.BLACK);

    private final String color;

    State(String color) {
      this.color = color;
    }

    public String getColor() {
      return this.color;
    }
  }

  private final DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();
  private final StaticBrake m_brake = new StaticBrake();
  // private CANcoder m_encoder;
  private TalonFX m_motor1;
  private State m_curState = State.STOP;

  /**
   * Returns true when the cage switch is engaged
   */
  public final Trigger hasCage = new Trigger(this::getHasCage);
  /**
   * Returns true when the climber has reached its limit
   */
  public final Trigger climbComplete = new Trigger(this::getClimbComplete);

  /**
   * Returns the instance of the ClimberSubsystem subsystem.
   * The purpose of this is to only create an instance if one does not already
   * exist.
   * 
   * @return ClimberSubsystem instance
   */
  public static ClimberSubsystem getInstance() {
    if (instance == null)
      instance = new ClimberSubsystem();
    return instance;
  }

  public ClimberSubsystem() {
    // initialize values for private and public variables, etc.
    // m_encoder = new CANcoder(ClimberConstants.kCANcoderID,
    // ClimberConstants.canBus);
    // RobotContainer.ctreConfigs.retryConfigApply(()->m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.climberCCConfig));

    m_motor1 = new TalonFX(ClimberConstants.kMotorID, ClimberConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.climberFXConfig));

    init();
    createDashboards();
  }

  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    climberStop();
    NCDebug.Debug.debug("Climber: Initialized");
  }

  @Override
  public void periodic() {
  }

  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Climber", this::getStateColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(6, 7);

    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout climberList = systemTab.getLayout("Climber", BuiltInLayouts.kList)
      .withSize(4, 6)
      .withPosition(16, 0)
      .withProperties(Map.of("Label position", "LEFT"));
    climberList.addString("Status", this::getStateColor)
      .withWidget("Single Color View");
    climberList.addBoolean("Has Cage", this::getHasCage);
    climberList.addBoolean("Complete", this::getClimbComplete);
    climberList.addString("State", this::getStateName);
    climberList.addNumber("Position", () -> NCDebug.General.roundDouble(getPosition().in(Units.Rotations), 7));
    climberList.addBoolean("CageSw1", this::getCageSwitch1);
    climberList.addBoolean("CageSw2", this::getCageSwitch2);

    if (ClimberConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
      ShuffleboardLayout dbgClimberList = debugTab.getLayout("Climber", BuiltInLayouts.kList)
        .withSize(4, 11)
        .withPosition(4, 0)
        .withProperties(Map.of("Label position", "LEFT"));
      dbgClimberList.addString("Status", this::getStateColor)
        .withWidget("Single Color View");
      dbgClimberList.addBoolean("Has Cage", this::getHasCage);
      dbgClimberList.addBoolean("Complete", this::getClimbComplete);
      dbgClimberList.addString("State", this::getStateName);
      dbgClimberList.addNumber("Position", () -> {
        return NCDebug.General.roundDouble(getPosition().in(Units.Rotations), 6);
      });
      dbgClimberList.addBoolean("CageSw1", this::getCageSwitch1);
      dbgClimberList.addBoolean("CageSw2", this::getCageSwitch2);
      dbgClimberList.add("Climber Up", new InstantCommand(this::climberUp))
        .withProperties(Map.of("show_type", false));
      dbgClimberList.add("Climber Down", new InstantCommand(this::climberDown))
        .withProperties(Map.of("show_type", false));
      dbgClimberList.add("Climber Hold", new InstantCommand(this::climberHold))
        .withProperties(Map.of("show_type", false));
      dbgClimberList.add("Climber Stop", new InstantCommand(this::climberStop))
        .withProperties(Map.of("show_type", false));
    }
  }

  public State getState() {
    return m_curState;
  }

  public String getStateName() {
    return m_curState.toString();
  }

  public String getStateColor() {
    return m_curState.getColor();
  }

  public boolean getHasCage() {
    return getCageSwitch1() && getCageSwitch2();
  }

  public boolean getClimbComplete() {
    return getClimbSwitch();
  }

  public boolean atLimit() {
    return getClimbComplete();
  }

  private boolean getCageSwitch1() {
    return !m_cageSwitch1.get();
  }

  private boolean getCageSwitch2() {
    return !m_cageSwitch2.get();
  }

  private boolean getClimbSwitch() {
    if(Robot.isSimulation()) {
      return !m_climbSwitch.get();
    }
    return (m_motor1.getForwardLimit().getValue() == ForwardLimitValue.Open);
  }

  public Angle getPosition() {
    return m_motor1.getPosition().getValue();
  }

  public void climberMove(double power) {
    if (power > 0) {
      if (m_curState != State.UP) {
        NCDebug.Debug.debug("Climber: Up (" + power + ")");
        m_curState = State.UP;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else if (power < 0) {
      if (m_curState != State.DOWN) {
        NCDebug.Debug.debug("Climber: Down (" + power + ")");
        m_curState = State.DOWN;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else { // 0 power
      if (m_curState != State.HOLD && m_curState != State.STOP) {
        m_motor1.setControl(m_brake);
        m_curState = State.HOLD;
        NCDebug.Debug.debug("Climber: Hold");
      }
    }
  }

  public Command climberMoveC(DoubleSupplier power) {
    return runOnce(() -> climberMove(power.getAsDouble()));
  }

  public Command climberStopC() {
    return runOnce(() -> climberStop());
  }

  public Command climberHoldC() {
    return runOnce(() -> climberHold());
  }

  public void climberUp() {
    m_curState = State.UP;
    climberMove(ClimberConstants.kClimbPower);
    NCDebug.Debug.debug("Climber: Up");
  }

  public void climberDown() {
    m_curState = State.DOWN;
    climberMove(-ClimberConstants.kClimbPower);
    NCDebug.Debug.debug("Climber: Down");
  }

  public void climberHold() {
    m_motor1.setControl(m_brake);
    if (m_curState != State.HOLD) {
      m_curState = State.HOLD;
      NCDebug.Debug.debug("Climber: Hold");
    }
  }

  public void climberStop() {
    m_motor1.setControl(m_neutral);
    if (m_curState != State.STOP) {
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

  // #region SysID Functions
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      null, // default ramp rate 1V/s
      Volts.of(4), // reduce dynamic step voltage to 4 to prevent brownout
      null, // default timeout 10s
      (state) -> SignalLogger.writeString("SysId_State", state.toString())),
    new SysIdRoutine.Mechanism(
      (volts) -> m_motor1.setControl(m_voltReq.withOutput(volts.in(Volts))),
      null,
      this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command runSysIdCommand() {
    return Commands.sequence(
      sysIdQuasistatic(SysIdRoutine.Direction.kForward).until(this::atLimit),
      sysIdQuasistatic(SysIdRoutine.Direction.kReverse).until(this::atLimit),
      sysIdDynamic(SysIdRoutine.Direction.kForward).until(this::atLimit),
      sysIdDynamic(SysIdRoutine.Direction.kReverse).until(this::atLimit));
  }
  // #endregion

}
