
package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.filter.LinearFilter;
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
 * This subsystem handles managing the Algae.
 * It is responsible for running the Algae using information from the indexer about whether we have a note.
 */
public class AlgaeSubsystem extends SubsystemBase {
	private static AlgaeSubsystem instance;
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
  public enum Position {
    DOWN(DashboardConstants.Colors.GREEN),
    UP(DashboardConstants.Colors.RED),
    STOW(DashboardConstants.Colors.ORANGE);
    private final String color;
    Position(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  private Position m_curPosition = Position.STOW;

  private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0);
  private final DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();
  private final StaticBrake m_brake = new StaticBrake();

  private final CANcoder m_encoder;
  private final TalonFX m_wristmotor1;
  private final TalonFXS m_swizmotor_left, m_swizmotor_right; 

  public final Trigger isRunning = new Trigger(() -> { return (m_curDirection != Direction.STOP);});

  /**
	 * Returns the instance of the AlgaeSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return AlgaeSubsystem instance
	 */
  public static AlgaeSubsystem getInstance() {
		if (instance == null)
			instance = new AlgaeSubsystem();
		return instance;
	}
  
  public AlgaeSubsystem() {
    // m_motor1 = new TalonFXS(AlgaeConstants.kMotorID,AlgaeConstants.kCANBus);
    // TalonFXSConfigurator m_config = m_motor1.getConfigurator();
    // TalonFXSConfiguration m_fxsConfigs = new TalonFXSConfiguration();
    // m_fxsConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    // m_config.apply(m_fxsConfigs);

    //initialize values for private and public variables, etc.
    m_encoder = new CANcoder(AlgaeConstants.kCANcoderID, AlgaeConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.algaeCCConfig));

    m_wristmotor1 = new TalonFX(AlgaeConstants.wrist.kMotorID, AlgaeConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_wristmotor1.getConfigurator().apply(RobotContainer.ctreConfigs.algaewristFXConfig));

    m_swizmotor_left = new TalonFXS(AlgaeConstants.left.kMotorID, AlgaeConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_swizmotor_left.getConfigurator().apply(RobotContainer.ctreConfigs.algaeleftFXSConfig));
    m_swizmotor_right = new TalonFXS(AlgaeConstants.right.kMotorID, AlgaeConstants.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_swizmotor_right.getConfigurator().apply(RobotContainer.ctreConfigs.algaerightFXSConfig));

    init();
    createDashboards();
  }
    
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    AlgaeStop();
    m_curDirection = Direction.STOP;
    NCDebug.Debug.debug("Algae: Initialized");
  }

  @Override
  public void periodic() {
  }

  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Algae", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(10, 7);  
      ShuffleboardTab systemTab = Shuffleboard.getTab("System");
			ShuffleboardLayout AlgaeList = systemTab.getLayout("Algae", BuiltInLayouts.kList)
				.withSize(4,2)
				.withPosition(4,0)
				.withProperties(Map.of("Label position","LEFT"));
			AlgaeList.addString("Status", this::getColor)
				.withWidget("Single Color View");
			AlgaeList.addString("Direction", this::getDirectionName);

      if(AlgaeConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
			ShuffleboardLayout dbgAlgaeList = debugTab.getLayout("Algae", BuiltInLayouts.kList)
				.withSize(4,6)
				.withPosition(8,4)
				.withProperties(Map.of("Label position","LEFT"));
			dbgAlgaeList.addString("Status", this::getColor)
				.withWidget("Single Color View");
			dbgAlgaeList.addString("Direction", this::getDirectionName);
      // dbgAlgaeList.add("Algae In", new InstantCommand(this::AlgaeIn))
      //   .withProperties(Map.of("show_type",false));  
      // dbgAlgaeList.add("Algae Out", new InstantCommand(this::AlgaeOut))
      //   .withProperties(Map.of("show_type",false));  
      // dbgAlgaeList.add("Algae Stop", new InstantCommand(this::AlgaeStop))
      //   .withProperties(Map.of("show_type",false));  
    }
  }

  public void AlgaeStop() {
    m_wristmotor1.setControl(m_neutral);
    if(m_curDirection != Direction.HOLD) {
      m_curDirection = Direction.STOP;
      NCDebug.Debug.debug("Algae: Stop");
    }
  }

  public Direction getDirection() { return m_curDirection; }
  public String getDirectionName() { return m_curDirection.toString(); }
  public String getColor() { return m_curDirection.getColor(); }

  private double getStatorCurrent() {
    return m_wristmotor1.getStatorCurrent().getValueAsDouble();
  }

  public TalonFX[] getMotors() {
    TalonFX[] motors = {m_wristmotor1};
    return motors;
  }
}
