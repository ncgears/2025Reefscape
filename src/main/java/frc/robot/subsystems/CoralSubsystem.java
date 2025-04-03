
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.*;
import frc.robot.utils.NCDebug;
import frc.robot.RobotContainer;

/**
 * This subsystem handles managing the Coral.
 * It is responsible for running the Coral using information from the indexer
 * about whether we have a note.
 */
@SuppressWarnings({ "unused" })
public class CoralSubsystem extends SubsystemBase {
  private static CoralSubsystem instance;

  // private and public variables defined here
  // #region Declarations
  public enum Direction {
    OUT(DashboardConstants.Colors.GREEN),
    IN(DashboardConstants.Colors.RED),
    HOLD(DashboardConstants.Colors.ORANGE),
    STOP(DashboardConstants.Colors.BLACK);

    private final String color;

    Direction(String color) {
      this.color = color;
    }

    public String getColor() {
      return this.color;
    }
  }

  private Direction m_curDirection = Direction.STOP;

  public enum Position {
    OUT(CoralConstants.Positions.kOut, DashboardConstants.Colors.GREEN),
    IN(CoralConstants.Positions.kIn, DashboardConstants.Colors.RED),
    STOW(CoralConstants.Positions.kStow, DashboardConstants.Colors.BLACK),
    SCORE(CoralConstants.Positions.kScore, DashboardConstants.Colors.ORANGE);

    private final double position;
    private final String color;

    Position(double position, String color) {
      this.position = position;
      this.color = color;
    }

    public double getRotations() {
      return this.position;
    }

    public String getColor() {
      return this.color;
    }
  }

  private Position m_targetPosition = Position.STOW;

  private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0);
  private final PositionVoltage m_posVoltage = new PositionVoltage(0);
  private final PositionDutyCycle m_posDutyCycle = new PositionDutyCycle(0);
  private final DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();
  private final StaticBrake m_brake = new StaticBrake();

  private final CANcoder m_encoder;
  private TalonFX m_motor1;
  private final LinearFilter curSpikeFilter = LinearFilter.highPass(0.1, 0.02);
  private static final double curSpikeLimit = CoralConstants.kCurrentSpikeLimit;
  // #endregion Declarations

  // #region Triggers
  private final Trigger curSpikeTrigger = new Trigger(
    () -> curSpikeFilter.calculate(getStatorCurrent()) > curSpikeLimit).debounce(0.15);
  public final Trigger isRunning = new Trigger(() -> {
    return (m_curDirection != Direction.STOP);
  });
  // #endregion Triggers

  // #region Setup
  /**
   * Returns the instance of the CoralSubsystem subsystem.
   * The purpose of this is to only create an instance if one does not already
   * exist.
   * 
   * @return CoralSubsystem instance
   */
  public static CoralSubsystem getInstance() {
    if (instance == null)
      instance = new CoralSubsystem();
    return instance;
  }

  public CoralSubsystem() {
    // m_motor1 = new TalonFXS(CoralConstants.kMotorID,CoralConstants.kCANBus);
    // TalonFXSConfigurator m_config = m_motor1.getConfigurator();
    // TalonFXSConfiguration m_fxsConfigs = new TalonFXSConfiguration();
    // m_fxsConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    // m_config.apply(m_fxsConfigs);

    //initialize values for private and public variables, etc.
    m_encoder = new CANcoder(CoralConstants.kCANcoderID, CoralConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.coralCCConfig));

    m_motor1 = new TalonFX(CoralConstants.kMotorID, CoralConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.coralFXConfig));

    init();
    createDashboards();
  }

  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    // resetMotorPosC();
    coralStop();
    m_curDirection = Direction.STOP;
    NCDebug.Debug.debug("Coral: Initialized");
  }

  @Override
  public void periodic() {
  }
  // #endregion Setup

  // #region Dashboard
  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Coral", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(0, 7);
    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout CoralList = systemTab.getLayout("Coral", BuiltInLayouts.kList)
      .withSize(4, 4)
      .withPosition(8, 4)
      .withProperties(Map.of("Label position", "LEFT"));
    CoralList.addString("Status", this::getColor)
      .withWidget("Single Color View");
    CoralList.addString("Direction", this::getDirectionName);
    CoralList.addString("Target", this::getTargetPositionName);
    CoralList.addNumber("Target Pos", this::getTargetPosition);
    CoralList.addNumber("Motor Pos", () -> { return NCDebug.General.roundDouble(getMotorPosition().in(Units.Rotations),6); });

    if (CoralConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
      ShuffleboardLayout dbgCoralList = debugTab.getLayout("Coral", BuiltInLayouts.kList)
        .withSize(4, 11)
        .withPosition(8, 0)
        .withProperties(Map.of("Label position", "LEFT"));
      dbgCoralList.addString("Status", this::getColor)
        .withWidget("Single Color View");
      dbgCoralList.addString("Direction", this::getDirectionName);
      dbgCoralList.addString("Target", this::getTargetPositionName);
      dbgCoralList.addNumber("Target Pos", this::getTargetPosition);
      dbgCoralList.addNumber("Motor Pos", () -> { return NCDebug.General.roundDouble(getMotorPosition().in(Units.Rotations),6); });
      dbgCoralList.addNumber("Absolute", () -> { return NCDebug.General.roundDouble(getPositionAbsolute().in(Units.Rotations),6); });
      dbgCoralList.add("Coral In", CoralPositionC(Position.IN))
        .withProperties(Map.of("show_type", false));
      dbgCoralList.add("Coral Out", CoralPositionC(Position.OUT))
        .withProperties(Map.of("show_type", false));
      dbgCoralList.add("Coral Score", CoralPositionC(Position.SCORE))
        .withProperties(Map.of("show_type", false));
      dbgCoralList.add("Coral Stop", CoralStopC())
        .withProperties(Map.of("show_type", false));
      dbgCoralList.add("Pos Reset", resetMotorPosC())
        .withProperties(Map.of("show_type", false));
    }
  }
  // #endregion Dashboard

  // #region Getters
  public Direction getDirection() {
    return m_curDirection;
  }

  public String getDirectionName() {
    return m_curDirection.toString();
  }

  public String getColor() {
    return m_curDirection.getColor();
  }

  public double getTargetPosition() {
    return NCDebug.General.roundDouble(m_motor1.getClosedLoopReference().getValue(),6);
  }

  public String getTargetPositionName() {
    return m_targetPosition.toString();
  }

  public double getPositionError() {
    return m_motor1.getClosedLoopError().getValue();
  }

  private double getStatorCurrent() {
    return m_motor1.getStatorCurrent().getValueAsDouble();
  }

  public Angle getMotorPosition() {
    return m_motor1.getPosition().getValue();
  }

  public Angle getPositionAbsolute() {
    return m_encoder.getPosition().getValue();
  }

  public TalonFX[] getMotors() {
    TalonFX[] motors = { m_motor1 };
    return motors;
  }
  // #endregion Getters

  // #region Setters
  public void setPosition(Position position) {
    m_targetPosition = position;
    //if forward, use slot1; if reverse use slot0
    int slot = (position.getRotations() > m_motor1.getClosedLoopReference().getValueAsDouble()) ? 1 : 0; 
    //figure out if forward or reverse
    m_motor1.setControl(m_posDutyCycle.withPosition(position.getRotations()).withSlot(slot));
    NCDebug.Debug.debug("Coral: Move to " + position.toString());
  }

  public Command resetMotorPosC() {
    return runOnce(() -> {
      m_motor1.setPosition(Position.IN.getRotations());
      NCDebug.Debug.debug("Coral: Reset relative encoder to " + Position.IN.getRotations());
    }).ignoringDisable(true);
  }
  // #endregion Setters

  // #region Limits
  public boolean getForwardLimit() {
    return m_motor1.getPosition().getValueAsDouble() >= CoralConstants.Positions.kFwdLimit;
  }

  public boolean getReverseLimit() {
    return m_motor1.getPosition().getValueAsDouble() <= CoralConstants.Positions.kRevLimit;
  }

  public boolean atLimit() {
    return getForwardLimit() || getReverseLimit();
  }
  // #endregion Limits

  // #region Controls
  public void coralStop() {
    m_motor1.setControl(m_neutral);
    if (m_curDirection != Direction.HOLD) {
      m_curDirection = Direction.STOP;
      NCDebug.Debug.debug("Coral: Stop");
    }
  }

  public Command CoralHomeC() {
    return runOnce(() -> {
      m_motor1.setControl(m_DutyCycle.withOutput(-CoralConstants.kZeroPower));
    });
  }
  
  public Command CoralZeroC() {
    return runOnce(() -> {
      m_motor1.setPosition(0);
      m_encoder.setPosition(0);
      NCDebug.Debug.debug("Coral: Zero Motor and Encoder");
    });
  }

  public Command CoralPositionC(Position position) {
    return runOnce(
      () -> setPosition(position)
    );
  }

  public Command CoralStopC() {
    return runOnce(
      () -> coralStop()
    );
  }
  // #endregion Controls

  // #region SysID Functions
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      Volts.per(Units.Second).of(0.75), // default ramp rate 1V/s
      Volts.of(1), // reduce dynamic step voltage to 4 to prevent brownout
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
}
