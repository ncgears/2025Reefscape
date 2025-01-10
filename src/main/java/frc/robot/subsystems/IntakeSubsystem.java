
package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.*; 
import frc.robot.utils.NCDebug;
import frc.robot.utils.NCDebug.Debug;
import frc.robot.RobotContainer;

/**
 * This subsystem handles managing the Intake.
 * It is responsible for running the intake using information from the indexer about whether we have a note.
 */
public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem instance;
  //private and public variables defined here
  public enum Direction {
    IN(DashboardConstants.Colors.GREEN),
    FEED(DashboardConstants.Colors.ORANGE),
    OUT(DashboardConstants.Colors.RED),
    STOP(DashboardConstants.Colors.BLACK);
    private final String color;
    Direction(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  private WPI_TalonSRX m_motor1;
  private Direction m_curDirection = Direction.STOP;
 
  public final Trigger isRunning = new Trigger(() -> { return (m_curDirection == Direction.IN);});

  /**
	 * Returns the instance of the IntakeSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return IntakeSubsystem instance
	 */
  public static IntakeSubsystem getInstance() {
		if (instance == null)
			instance = new IntakeSubsystem();
		return instance;
	}
  
  public IntakeSubsystem() {
    // m_motor1 = new TalonFXS(IntakeConstants.kMotorID,IntakeConstants.kCANBus);
    // TalonFXSConfigurator m_config = m_motor1.getConfigurator();
    // TalonFXSConfiguration m_fxsConfigs = new TalonFXSConfiguration();
    // m_fxsConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    // m_config.apply(m_fxsConfigs);

    //initialize values for private and public variables, etc.
    m_motor1 = new WPI_TalonSRX(IntakeConstants.kMotorID);
    m_motor1.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff from carrying over
    m_motor1.set(ControlMode.PercentOutput, 0); //Set controller to disabled
    m_motor1.setNeutralMode(IntakeConstants.kNeutralMode); //Set controller to brake mode  
    m_motor1.setInverted(IntakeConstants.kIsInverted);

    init();
    createDashboards();
  }
    
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    m_motor1.set(ControlMode.PercentOutput,0);
    m_curDirection = Direction.STOP;
    NCDebug.Debug.debug("Intake: Initialized");
  }

  @Override
  public void periodic() {
  }

  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Intake", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(8, 7);  
      ShuffleboardTab systemTab = Shuffleboard.getTab("System");
			ShuffleboardLayout intakeList = systemTab.getLayout("Intake", BuiltInLayouts.kList)
				.withSize(4,2)
				.withPosition(4,0)
				.withProperties(Map.of("Label position","LEFT"));
			intakeList.addString("Status", this::getColor)
				.withWidget("Single Color View");
			intakeList.addString("Direction", this::getDirectionName);

      if(IntakeConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
			ShuffleboardLayout dbgIntakeList = debugTab.getLayout("Intake", BuiltInLayouts.kList)
				.withSize(4,6)
				.withPosition(8,4)
				.withProperties(Map.of("Label position","LEFT"));
			dbgIntakeList.addString("Status", this::getColor)
				.withWidget("Single Color View");
			dbgIntakeList.addString("Direction", this::getDirectionName);
      dbgIntakeList.add("Intake In", new InstantCommand(this::intakeIn))
        .withProperties(Map.of("show_type",false));  
      dbgIntakeList.add("Intake Out", new InstantCommand(this::intakeOut))
        .withProperties(Map.of("show_type",false));  
      dbgIntakeList.add("Intake Stop", new InstantCommand(this::intakeStop))
        .withProperties(Map.of("show_type",false));  
    }
  }

  /**
   * Sets the speed of the Intake
   * @param speed The speed of the Intake in percentage (-1.0 to 1.0)
   */
  public void setSpeedPercent(double speed) {
    m_motor1.set(ControlMode.PercentOutput, speed);
  }

  public Direction getDirection() { return m_curDirection; }
  public String getDirectionName() { return m_curDirection.toString(); }
  public String getColor() { return m_curDirection.getColor(); }

  public void intakeAuto() {
    if(RobotContainer.indexer.hasNote()) {
      intakeOut();
    } else {
      intakeIn();
    }
  }

  public void intakeIn() {
    m_curDirection = Direction.IN;
    NCDebug.Debug.debug("Intake: In");
    setSpeedPercent(IntakeConstants.kSpeed);
  }

  public void intakeFeed() {
    m_curDirection = Direction.FEED;
    NCDebug.Debug.debug("Intake: Feed");
    setSpeedPercent(IntakeConstants.kSpeed/2);
  }

  public void intakeOut() {
    m_curDirection = Direction.OUT;
    NCDebug.Debug.debug("Intake: Out");
    setSpeedPercent(-IntakeConstants.kSpeed);
  }

  public void intakeStop() {
    m_curDirection = Direction.STOP;
    NCDebug.Debug.debug("Intake: Stop");
    setSpeedPercent(0.0);
  }
}
