
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.NCDebug;

/**
 * This subsystem handles managing the Template subsystem.
 * It is responsible for doing some stuff.
 */
public class TemplateSubsystem extends SubsystemBase {
	private static TemplateSubsystem instance;
  //#region Declarations
    // Declare public and private variables
  //#endregion Declarations

  //#region Triggers
    // Trigger definitions
  //#endregion Triggers

  //#region Setup
  /**
	 * Returns the instance of the TemplateSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return TemplateSubsystem instance
	 */
  public static TemplateSubsystem getInstance() {
		if (instance == null)
			instance = new TemplateSubsystem();
		return instance;
	}
  
  public TemplateSubsystem() {
    //initialize values for private and public variables, etc.
  	
    init();
  }
  
  /**
   * The init method resets and operational state of the subsystem
   */
  public void init() {
    // set initial stuff, etc.
    NCDebug.Debug.debug("Template: Initialized");
  }
  
  @Override
  public void periodic() {
  }
  //#endregion Setup

  //#region Dashboard
    // Methods for creating and updating dashboards
  //#endregion Dashboard

  //#region Getters
    // Methods for getting data for subsystem
  //#endregion Getters

  //#region Setters
    // Methods for setting data for subsystem
  //#endregion Setters

  //#region Limits
    // Methods for detecting limit conditions
  //#endregion Limits
  
  //#region Controls
    // Methods for controlling the subsystem
  //#endregion Controls

  //#region SysID Functions
    // Routines for characterization
  //#endregion SysID Functions
}
