
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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
import frc.robot.RobotContainer;

/**
 * This subsystem handles managing the Elevator.
 * It is responsible for extending and retracting the elevator/Elevator.
 */
public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem instance;

  // private and public variables defined here
  // #region Declarations
  public enum State {
    UP(DashboardConstants.Colors.GREEN),
    DOWN(DashboardConstants.Colors.RED),
    HOLD(DashboardConstants.Colors.ORANGE),
    STOP(DashboardConstants.Colors.BLACK);

    private final String color;

    State(String color) {
      this.color = color;
    }

    public String getColor() {
      return this.color;
    }
  }

  public enum Position {
    STOW(ElevatorConstants.Positions.kStow),
    L1(ElevatorConstants.Positions.kL1),
    L2SCORE(ElevatorConstants.Positions.kL2Score),
    L2(ElevatorConstants.Positions.kL2),
    L3SCORE(ElevatorConstants.Positions.kL3Score),
    L3(ElevatorConstants.Positions.kL3),
    L4SCORE(ElevatorConstants.Positions.kL4Score),
    L4(ElevatorConstants.Positions.kL4),
    BARGE(ElevatorConstants.Positions.kBarge),
    ALGAEHIGH(ElevatorConstants.Positions.kAlgaeHigh),
    ALGAELOW(ElevatorConstants.Positions.kAlgaeLow),
    LINEUP(ElevatorConstants.Positions.kLineup),
    FLOOR(ElevatorConstants.Positions.kFloor),
    HP(ElevatorConstants.Positions.kHP),
    PROC(ElevatorConstants.Positions.kProcessor);

    private final double position;

    Position(double position) {
      this.position = position;
    }

    public double getRotations() {
      return this.position;
    }
  }

  private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0);
  private final PositionVoltage m_posVoltage = new PositionVoltage(0);
  private final DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();
  private final StaticBrake m_brake = new StaticBrake();
  private CANcoder m_encoder;
  private TalonFX m_motor1;
  private State m_curState = State.STOP;
  private Position m_prevPosition = Position.L1;
  private Position m_targetPosition = Position.STOW;
  // #endregion Declarations

  // #region Triggers
  /**
   * Returns true when the elevator has reached its limit
   */
  public final Trigger atTarget = new Trigger(this::isAtTarget);

  // #endregion Triggers

  // #region Setup
  /**
   * Returns the instance of the ElevatorSubsystem subsystem.
   * The purpose of this is to only create an instance if one does not already
   * exist.
   * 
   * @return ElevatorSubsystem instance
   */
  public static ElevatorSubsystem getInstance() {
    if (instance == null)
      instance = new ElevatorSubsystem();
    return instance;
  }

  public ElevatorSubsystem() {
    // initialize values for private and public variables, etc.
    m_encoder = new CANcoder(ElevatorConstants.kCANcoderID, ElevatorConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.elevatorCCConfig));

    m_motor1 = new TalonFX(ElevatorConstants.kMotorID, ElevatorConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.elevatorFXConfig));

    init();
    createDashboards();
  }

  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    m_targetPosition = Position.STOW;
    // ElevatorStop();
    setPosition(m_targetPosition);
    NCDebug.Debug.debug("Elevator: Initialized");
  }

  @Override
  public void periodic() {
  }
  // #endregion Setup

  // #region Dashboard
  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Elevator", this::getStateColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(4, 7);

    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout ElevatorList = systemTab.getLayout("Elevator", BuiltInLayouts.kList)
      .withSize(4, 6)
      .withPosition(12, 0)
      .withProperties(Map.of("Label position", "LEFT"));
    ElevatorList.addString("Status", this::getStateColor)
      .withWidget("Single Color View");
    ElevatorList.addString("State", this::getStateName);
    ElevatorList.addString("Target", this::getTargetPositionName);
    ElevatorList.addNumber("Target Pos", () -> NCDebug.General.roundDouble(getTargetPosition(),6));
    ElevatorList.addNumber("Motor Pos", () -> NCDebug.General.roundDouble(getPosition().in(Units.Rotations), 6));

    if (ElevatorConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
      ShuffleboardLayout dbgElevatorList = debugTab.getLayout("Elevator", BuiltInLayouts.kList)
        .withSize(4, 11)
        .withPosition(16, 0)
        .withProperties(Map.of("Label position", "LEFT"));
      dbgElevatorList.addString("Status", this::getStateColor)
        .withWidget("Single Color View");
      dbgElevatorList.addString("State", this::getStateName);
      dbgElevatorList.addString("Target", this::getTargetPositionName);
      dbgElevatorList.addNumber("Target Pos", this::getTargetPosition);
      dbgElevatorList.addNumber("Position", () -> {
        return NCDebug.General.roundDouble(getPosition().in(Units.Rotations), 6);
      });
      dbgElevatorList.addNumber("Absolute", () -> {
        return NCDebug.General.roundDouble(getPositionAbsolute().in(Units.Rotations), 6);
      });
      dbgElevatorList.addNumber("Error", this::getPositionError);
      dbgElevatorList.add("Elevator Up", new InstantCommand(this::ElevatorUp))
        .withProperties(Map.of("show_type", false));
      dbgElevatorList.add("Elevator Down", new InstantCommand(this::ElevatorDown))
        .withProperties(Map.of("show_type", false));
      dbgElevatorList.add("Elevator Hold", new InstantCommand(this::ElevatorHold))
        .withProperties(Map.of("show_type", false));
      dbgElevatorList.add("Elevator Stop", new InstantCommand(this::ElevatorStop))
        .withProperties(Map.of("show_type", false));
    }
  }
  // #endregion Dashboard

  // #region Getters
  public State getState() {
    return m_curState;
  }

  public String getStateName() {
    return m_curState.toString();
  }

  public String getStateColor() {
    return m_curState.getColor();
  }

  public String getTargetPositionName() {
    return m_targetPosition.toString();
  }

  public double getTargetPosition() {
    return m_motor1.getClosedLoopReference().getValue();
  }

  public double getPositionError() {
    // m_motor1.getClosedLoopError(true)
    // return m_motor1.getClosedLoopError(true).getValue();
    return (getTargetPosition() - getPosition().in(Units.Rotations));
  }

  public Angle getPosition() {
    return m_motor1.getPosition().getValue();
  }

  public Angle getPositionAbsolute() {
    return m_encoder.getPosition().getValue();
  }

  public boolean isAtTarget() {
    return (Math.abs(getPositionError()) <= ElevatorConstants.kPositionTolerance);
  }
  // #endregion Getters

  // #region Setters
  public void setPrevPosition(Position position) {
    switch (position) {
      case L4:
      case L3:
      case L2:
      case L1:
        if(position != m_prevPosition) NCDebug.Debug.debug("Elevator: Saved Last Position " + position.toString());
        m_prevPosition = position;
        break;
      default:
        break;
    }
  }

  public void setPosition(Position position) {
    setPrevPosition(position);
    m_targetPosition = position;
    // //if down, use slot1; if up use slot0
    // int slot = (position.getRotations() > m_motor1.getClosedLoopReference().getValueAsDouble()) ? 1 : 0; 
    // m_motor1.setControl(m_mmVoltage.withPosition(position.getRotations()).withSlot(slot));
    // m_motor1.setControl(m_mmVoltage.withPosition(position.getRotations()));
    gotoTargetPosition();
    NCDebug.Debug.debug("Elevator: Move to " + position.toString());
  }
  // #endregion Setters

  // #region Limits
  public boolean getForwardLimit() {
    // if using NormallyOpen, this should be ForwardLimitValue.ClosedToGround
    // return m_motor1.getForwardLimit().getValue() ==
    // ForwardLimitValue.ClosedToGround;
    return m_motor1.getPosition().getValueAsDouble() >= ElevatorConstants.Positions.kFwdLimit;
  }

  public boolean getReverseLimit() {
    // if using NormallyOpen, this should be ReverseLimitValue.ClosedToGround
    // return m_motor1.getReverseLimit().getValue() ==
    // ReverseLimitValue.ClosedToGround;
    return m_motor1.getPosition().getValueAsDouble() <= ElevatorConstants.Positions.kRevLimit;
  }

  public boolean atLimit() {
    // if Either limit is met
    return getForwardLimit() || getReverseLimit();
  }
  // #endregion Limits

  // #region Controls
  public void ElevatorMove(double power) {
    if (power > 0) {
      if (m_curState != State.UP) {
        NCDebug.Debug.debug("Elevator: Up (" + power + ")");
        m_curState = State.UP;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else if (power < 0) {
      if (m_curState != State.DOWN) {
        NCDebug.Debug.debug("Elevator: Down (" + power + ")");
        m_curState = State.DOWN;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else { // 0 power
      if (m_curState != State.HOLD && m_curState != State.STOP) {
        m_motor1.setControl(m_mmVoltage.withPosition(m_motor1.getPosition().getValueAsDouble())); // maintain this spot
        // m_motor1.setControl(m_brake);
        m_curState = State.HOLD;
        NCDebug.Debug.debug("Elevator: Hold");
      }
    }
  }

  public Command ElevatorMoveC(DoubleSupplier power) {
    return run(() -> ElevatorMove(power.getAsDouble()));
  }

  public Command ElevatorPositionC(Position position) {
    return runOnce(
        () -> setPosition(position));
  }
  // effectively, this is runOnce().andThen(idle()), allowing us to interrupt using .until(::atTarget) trigger
  // public Command ElevatorPositionC(Position position) {
  //   return runAndWait(
  //       () -> setPosition(position));
  // }

  public Command ScoreC() {
    return runOnce(() -> {
      switch (m_targetPosition) {
        case L4:
          setPosition(Position.L4SCORE);
          break;
        case L3:
          setPosition(Position.L3SCORE);
          break;
        case L2:
          setPosition(Position.L2SCORE);
          break;
        default:
          NCDebug.Debug.debug("Elevator: Not in a scoring configuration from " + m_targetPosition.toString());
      }
    });
  }

  public Command LastPositionC() {
    return runOnce(
      () -> setPosition(m_prevPosition)
    );
  }

  public Command ElevatorStopC() {
    return runOnce(
      () -> ElevatorStop()
    );
  }

  public Command ElevatorZeroC() {
    return runOnce(() -> {
      m_motor1.setPosition(0);
      m_encoder.setPosition(0);
      NCDebug.Debug.debug("Elevator: Zero Motor and Encoder");
    });
  }

  public void gotoTargetPosition() {
    // m_motor1.setControl(m_mmVoltage.withPosition(m_targetPosition.getRotations()));
    m_motor1.setControl(m_posVoltage.withPosition(m_targetPosition.getRotations()));
  }

  public void ElevatorUp() {
    m_curState = State.UP;
    m_motor1.setControl(m_DutyCycle);
    NCDebug.Debug.debug("Elevator: Up");
  }

  public void ElevatorDown() {
    m_curState = State.DOWN;
    NCDebug.Debug.debug("Elevator: Down");
  }

  public void ElevatorHold() {
    m_motor1.setControl(m_brake);
    if (m_curState != State.HOLD) {
      m_curState = State.HOLD;
      NCDebug.Debug.debug("Elevator: Hold");
    }
  }

  public void ElevatorStop() {
    m_motor1.setControl(m_neutral);
    if (m_curState != State.STOP) {
      m_curState = State.STOP;
      NCDebug.Debug.debug("Elevator: Stop");
    }
  }

  public void setCoast() {
    m_motor1.setNeutralMode(NeutralModeValue.Coast);
    NCDebug.Debug.debug("Elevator: Switch to Coast");
  }

  public void setBrake() {
    m_motor1.setNeutralMode(NeutralModeValue.Brake);
    NCDebug.Debug.debug("Elevator: Switch to Brake");
  }
  // #endregion Controls

  // #region SysID Functions
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      Volts.per(Units.Second).of(0.75), // default ramp rate 1V/s
      Volts.of(1.25), // reduce dynamic step voltage to 4 to prevent brownout
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
  // #endregion SysID Functions

  // #region Example for 2054
  private enum elevatorPositions {
    FLOOR, L1, L2, L3, L4
  };

  private enum wheelPositions {
    TRANSIT, CORAL, ALGAE
  };

  private void moveWheel(wheelPositions targetPos) {
    // move the wheel to targetPos
  }

  public Command moveWheelCommand(wheelPositions targetPos) {
    // move the wheel
    return run(() -> moveWheel(targetPos));
  }

  public boolean wheelAtPosition() {
    // this should ask the wheel subsystem if the wheel is at the target
    return true;
  }

  private void moveElevator(elevatorPositions targetPos) {
    // move the elevator
  }

  public Command moveElevatorCommand(elevatorPositions targetPos) {
    // move the wheel
    return run(() -> moveElevator(targetPos));
  }

  public boolean elevAtPosition() {
    // this should ask the elev subsystem if the elev is at the target
    return true;
  }

  public Command raiseElevatorCommand(elevatorPositions elevTarget, wheelPositions wheelTarget) {
    return Commands.sequence(
      moveWheelCommand(wheelPositions.TRANSIT).until(() -> wheelAtPosition()),
      moveElevatorCommand(elevTarget).until(() -> elevAtPosition()),
      moveWheelCommand(wheelTarget));
  }

  public Command lowerElevatorCommand(elevatorPositions elevTarget, wheelPositions wheelTarget) {
    return Commands.sequence(
      moveWheelCommand(wheelPositions.TRANSIT).until(() -> wheelAtPosition()),
      moveElevatorCommand(elevTarget).until(() -> elevAtPosition()),
      moveWheelCommand(wheelTarget));
  }
  // #endregion Example for 2054

}
