
package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.*; 
import frc.robot.utils.NCDebug;
import frc.robot.RobotContainer;

/**
 * This subsystem handles managing the Coral.
 * It is responsible for running the Coral using information from the indexer about whether we have a note.
 */
public class CoralSubsystem extends SubsystemBase {
	private static CoralSubsystem instance;
  //private and public variables defined here
  public enum Direction {
    OUT(DashboardConstants.Colors.GREEN),
    IN(DashboardConstants.Colors.RED),
    HOLD(DashboardConstants.Colors.ORANGE),
    STOP(DashboardConstants.Colors.BLACK);
    private final String color;
    Direction(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  private Direction m_curDirection = Direction.STOP;

  private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0);
  private final DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();
  private final StaticBrake m_brake = new StaticBrake();

  private final CANcoder m_encoder;
  private final TalonFX m_motor1;
  private final LinearFilter curSpikeFilter = LinearFilter.highPass(0.1, 0.02);
  private static final double curSpikeLimit = CoralConstants.kCurrentSpikeLimit;

  private final Trigger curSpikeTrigger = new Trigger(() -> curSpikeFilter.calculate(getStatorCurrent()) > curSpikeLimit).debounce(0.15);
  public final Trigger isRunning = new Trigger(() -> { return (m_curDirection != Direction.STOP);});

  /**
	 * Returns the instance of the CoralSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
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
    RobotContainer.ctreConfigs.retryConfigApply(()->m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.coralFXConfig));

    init();
    createDashboards();
  }
    
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    coralStop();
    m_curDirection = Direction.STOP;
    NCDebug.Debug.debug("Coral: Initialized");
  }

  @Override
  public void periodic() {
  }

  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Coral", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(8, 7);  
      ShuffleboardTab systemTab = Shuffleboard.getTab("System");
			ShuffleboardLayout CoralList = systemTab.getLayout("Coral", BuiltInLayouts.kList)
				.withSize(4,2)
				.withPosition(4,0)
				.withProperties(Map.of("Label position","LEFT"));
			CoralList.addString("Status", this::getColor)
				.withWidget("Single Color View");
			CoralList.addString("Direction", this::getDirectionName);

      if(CoralConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
			ShuffleboardLayout dbgCoralList = debugTab.getLayout("Coral", BuiltInLayouts.kList)
				.withSize(4,6)
				.withPosition(8,4)
				.withProperties(Map.of("Label position","LEFT"));
			dbgCoralList.addString("Status", this::getColor)
				.withWidget("Single Color View");
			dbgCoralList.addString("Direction", this::getDirectionName);
      // dbgCoralList.add("Coral In", new InstantCommand(this::CoralIn))
      //   .withProperties(Map.of("show_type",false));  
      // dbgCoralList.add("Coral Out", new InstantCommand(this::CoralOut))
      //   .withProperties(Map.of("show_type",false));  
      // dbgCoralList.add("Coral Stop", new InstantCommand(this::CoralStop))
      //   .withProperties(Map.of("show_type",false));  
    }
  }

  public void coralStop() {
    m_motor1.setControl(m_neutral);
    if(m_curDirection != Direction.HOLD) {
      m_curDirection = Direction.STOP;
      NCDebug.Debug.debug("Climber: Stop");
    }
  }

  public Direction getDirection() { return m_curDirection; }
  public String getDirectionName() { return m_curDirection.toString(); }
  public String getColor() { return m_curDirection.getColor(); }

  private double getStatorCurrent() {
    return m_motor1.getStatorCurrent().getValueAsDouble();
  }
}
