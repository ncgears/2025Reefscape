
package frc.robot.classes;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.*; 
import frc.robot.utils.NCDebug;

/**
 * The Lighting Subsystem handles getting and managing the Lighting from the CANdle.
 * It is responsible for changing lighting colors and animations.
 */
@SuppressWarnings("unused")
public class Lighting {
	private static Lighting instance;
  private final CANdle m_candle1 = new CANdle(LightingConstants.kCandle1ID, LightingConstants.canBus);
  private final CANdle m_candle2 = new CANdle(LightingConstants.kCandle2ID, LightingConstants.canBus);
  private Colors m_currentColor, m_oldColor = Colors.OFF;
  private boolean m_blinking, m_oldBlinking = false;
  private int m_intensity, m_oldIntensity = 100;
  //private and public variables defined here
  public enum Colors {
    OFF(0,0,0),
    NCBLUE(0,119,181),
    NCGREEN(0,255,0),
    AMBER(255,100,0),
    AQUA(50,255,255),
    BLACK(0,0,0),
    BLUE(0,0,255),
    CYAN(0,255,255),
    GOLD(255,222,30),
    GREEN(0,255,0),
    JADE(0,255,40),
    MAGENTA(255,0,20),
    LACE(253,245,230),
    ORANGE(255,40,0),
    PINK(242,90,255),
    PURPLE(180,0,255),
    RED(255,0,0),
    WHITE(255,255,255),
    TEAL(0,255,120),
    YELLOW(255,150,0);
    private final int r, g, b;
    Colors(int r, int g, int b) { this.r = r; this.g = g; this.b = b; }
    public int R() { return this.r; }
    public int G() { return this.g; }
    public int B() { return this.b; }
  }
  /**
	 * Returns the instance of the LightingSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return LightingSubsystem instance
	 */
  public static Lighting getInstance() {
		if (instance == null)
			instance = new Lighting();
		return instance;
	}
  
  public Lighting() {
    //initialize values for private and public variables, etc.
    init();
    publishData();
  }
  
    
  /**
   * The init method resets and operational state of the class
   */
  public void init() {
    m_currentColor = Colors.OFF;
    NCDebug.Debug.debug("Lighting: Initialized");
  }
  
  // #region Dashboard
  public void publishData() {
    SmartDashboard.putData("Lighting", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("LED Color", () -> getColor(), null);
      }      
    });
  }
  // #endregion Dashboard

  public String getColor() {
    if(m_currentColor == null) m_currentColor = Colors.OFF;
    return new Color(m_currentColor.R(), m_currentColor.G(), m_currentColor.B()).toHexString();
  }

  public void setColor(Colors color) {
    if(color != m_currentColor) { //change it
      m_currentColor = color;
      m_candle1.setLEDs(color.R(), color.G(), color.B());
      m_candle2.setLEDs(color.R(), color.G(), color.B());
    }
  }

  public Command setColorCommand(Colors color) {
    return new InstantCommand(() -> setColor(color)).ignoringDisable(true);
  }

  public Command danceParty() {
    return new RepeatCommand(  //Lighting Dance Party!
      setColorCommand(Colors.NCBLUE)
      .andThen(wait(0.5))
      .andThen(setColorCommand(Colors.NCGREEN))
      .andThen(wait(0.5))
      .andThen(setColorCommand(Colors.NCBLUE))
      .andThen(wait(0.15))
      .andThen(setColorCommand(Colors.OFF))
      .andThen(wait(0.15))
      .andThen(setColorCommand(Colors.NCBLUE))
      .andThen(wait(0.15))
      .andThen(setColorCommand(Colors.NCGREEN))
      .andThen(wait(0.25))
    );
  }

  public Command climbStartColor() {
    return setColorCommand(Colors.GOLD);
  }

  public Command climbDoneColor() {
    return new RepeatCommand(  //blinking
      setColorCommand(Colors.OFF)
      .andThen(wait(0.2))
      .andThen(setColorCommand(Colors.GOLD))
      .andThen(wait(0.2))
    );
  }

  private Command wait(double seconds) {
    return new WaitCommand(seconds);
  }

}
