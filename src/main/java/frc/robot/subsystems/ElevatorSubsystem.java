
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
 * This subsystem handles managing the Elevator.
 * It is responsible for extending and retracting the elevator/Elevator.
 */
public class ElevatorSubsystem extends SubsystemBase {
	private static ElevatorSubsystem instance;
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
    STOW(ElevatorConstants.Positions.kStow),
    L1(ElevatorConstants.Positions.kL1),
    L2SCORE(ElevatorConstants.Positions.kL2Score),
    L2(ElevatorConstants.Positions.kL2),
    L3SCORE(ElevatorConstants.Positions.kL3Score),
    L3(ElevatorConstants.Positions.kL3),
    L4SCORE(ElevatorConstants.Positions.kL4Score),
    L4(ElevatorConstants.Positions.kL4);
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
	 * Returns the instance of the ElevatorSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return ElevatorSubsystem instance
	 */
  public static ElevatorSubsystem getInstance() {
		if (instance == null)
			instance = new ElevatorSubsystem();
		return instance;
	}
  
  public ElevatorSubsystem() {
    //initialize values for private and public variables, etc.
    m_encoder = new CANcoder(ElevatorConstants.kCANcoderID, ElevatorConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.elevatorCCConfig));

    m_motor1 = new TalonFX(ElevatorConstants.kMotorID, ElevatorConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.elevatorFXConfig));

    init();
    createDashboards();
  }
  
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    m_targetPosition = Position.STOW;
    ElevatorStop();
    NCDebug.Debug.debug("Elevator: Initialized");
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
    driverTab.addString("Elevator", this::getStateColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(12, 7);  

    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout ElevatorList = systemTab.getLayout("Elevator", BuiltInLayouts.kList)
      .withSize(4,6)
      .withPosition(16,0)
      .withProperties(Map.of("Label position","LEFT"));
    ElevatorList.addString("Status", this::getStateColor)
      .withWidget("Single Color View");
    ElevatorList.addString("State", this::getStateName);
    ElevatorList.addString("Target", this::getTargetPositionName);
    ElevatorList.addNumber("Target Pos", this::getTargetPosition);
    ElevatorList.addNumber("Position", () -> NCDebug.General.roundDouble(getPosition().in(Units.Rotations),7));
    ElevatorList.addNumber("Absolute", () -> NCDebug.General.roundDouble(getPositionAbsolute().in(Units.Rotations),7));
    ElevatorList.addNumber("Error", () -> NCDebug.General.roundDouble(getPositionError(),7));

    if(ElevatorConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
			ShuffleboardLayout dbgElevatorList = debugTab.getLayout("Elevator", BuiltInLayouts.kList)
				.withSize(4,10)
				.withPosition(16,0)
				.withProperties(Map.of("Label position","LEFT"));
      dbgElevatorList.addString("Status", this::getStateColor)
        .withWidget("Single Color View");
      dbgElevatorList.addString("State", this::getStateName);
      dbgElevatorList.addNumber("Target", this::getTargetPosition);
      dbgElevatorList.addNumber("Position", () -> { return getPosition().in(Units.Rotations); });
      dbgElevatorList.addNumber("Absolute", () -> { return getPositionAbsolute().in(Units.Rotations); });
      dbgElevatorList.addNumber("Error", this::getPositionError);
      dbgElevatorList.add("Elevator Up", new InstantCommand(this::ElevatorUp))
        .withProperties(Map.of("show_type",false));  
      dbgElevatorList.add("Elevator Down", new InstantCommand(this::ElevatorDown))
        .withProperties(Map.of("show_type",false));  
      dbgElevatorList.add("Elevator Hold", new InstantCommand(this::ElevatorHold))
        .withProperties(Map.of("show_type",false));  
      dbgElevatorList.add("Elevator Stop", new InstantCommand(this::ElevatorStop))
        .withProperties(Map.of("show_type",false));  
    }
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getStateColor() { return m_curState.getColor(); }

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
    NCDebug.Debug.debug("Elevator: Move to "+position.toString());
  }

  public boolean getForwardLimit() {
    //if using NormallyOpen, this should be ForwardLimitValue.ClosedToGround
    return m_motor1.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  public boolean getReverseLimit() {
    //if using NormallyOpen, this should be ReverseLimitValue.ClosedToGround
    return m_motor1.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  public void ElevatorMove(double power) {
    if(power>0) {
      if(m_curState != State.UP) {
        NCDebug.Debug.debug("Elevator: Up ("+power+")");
        m_curState = State.UP;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else if(power<0) {
      if(m_curState != State.DOWN) {
        NCDebug.Debug.debug("Elevator: Down ("+power+")");
        m_curState = State.DOWN;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else { //0 power
      if(m_curState != State.HOLD && m_curState != State.STOP) {
        m_motor1.setControl(m_brake);
        m_curState = State.HOLD;
        NCDebug.Debug.debug("Elevator: Hold");
      }
    }
  }

  public Command ElevatorMoveC(DoubleSupplier power) {
    return run(() -> ElevatorMove(power.getAsDouble()));
  }

  public void ElevatorL4() { setPosition(Position.L4); }
  public void ElevatorL4Score() { setPosition(Position.L4SCORE); }
  public void ElevatorL3() { setPosition(Position.L3); }
  public void ElevatorL3Score() { setPosition(Position.L3SCORE); }
  public void ElevatorL2() { setPosition(Position.L2); }
  public void ElevatorL2Score() { setPosition(Position.L2SCORE); }
  public void ElevatorL1() { setPosition(Position.L1); }

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
    if(m_curState != State.HOLD) {
      m_curState = State.HOLD;
      NCDebug.Debug.debug("Elevator: Hold");
    }
  }
  public void ElevatorStop() {
    m_motor1.setControl(m_neutral);
    if(m_curState != State.HOLD) {
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

}
