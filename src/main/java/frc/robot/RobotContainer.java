// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.classes.Gyro;
import frc.robot.classes.Lighting;
import frc.robot.classes.Lighting.Colors;
import frc.robot.classes.NCOrchestra;
import frc.robot.classes.Targeting;
import frc.robot.classes.Vision;
import frc.robot.constants.*;
import frc.robot.utils.CTREConfigs;
import frc.robot.utils.InputAxis;
import frc.robot.utils.NCDebug;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class RobotContainer {
    //#region Declarations
    public static final CTREConfigs ctreConfigs = new CTREConfigs();
    public static final Lighting lighting = Lighting.getInstance();
    public static final Gyro gyro = Gyro.getInstance();
    public static final Vision vision = Vision.getInstance();
    private final NCOrchestra orchestra = NCOrchestra.getInstance();
    // public static final DriveSubsystem drive = DriveSubsystem.getInstance(); //must be after gyro
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); //must be after gyro
    public static final Targeting targeting = Targeting.getInstance(); //must be after drive
    public static final PowerDistribution power = new PowerDistribution(1,ModuleType.kRev);
    public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public static final ClimberSubsystem climber = ClimberSubsystem.getInstance();
    public static final CoralSubsystem coral = CoralSubsystem.getInstance();
    public static final AlgaeSubsystem algae = AlgaeSubsystem.getInstance();
    
    public static Optional<Alliance> m_alliance;

    private AutoFactory autoFactory;
    private AutoRoutines autoRoutines;
        
    private final AutoChooser autoChooser = new AutoChooser();
    //Sendables definitions
    private SendableChooser<Command> m_auto_chooser = new SendableChooser<>();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(SwerveConstants.kMaxAngularRate).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors    
    private final SwerveRequest.RobotCentric robotdrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandStadiaController dj = new CommandStadiaController(OIConstants.JoyDriverID);
    private final CommandStadiaController oj = new CommandStadiaController(OIConstants.JoyOperID);
    private final CommandStadiaController pj = new CommandStadiaController(OIConstants.JoyProgID);
    //#endregion Declarations

    public RobotContainer() {
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        final InputAxis m_fieldX = new InputAxis("Forward", dj::getLeftY)
            .withDeadband(OIConstants.kMinDeadband)
            .withInvert(true)
            .withSquaring(false);
        final InputAxis m_fieldY = new InputAxis("Strafe", dj::getLeftX)
            .withDeadband(OIConstants.kMinDeadband)
            .withInvert(true)
            .withSquaring(false);
        final InputAxis m_rotate = new InputAxis("Rotate", dj::getRightX)
            .withDeadband(OIConstants.kMinDeadband)
            .withInvert(true);
        final InputAxis m_elevatorAxis = new InputAxis("Elevator", oj::getRightY)
            .withDeadband(OIConstants.kMinDeadband)
            .withMultiplier(ElevatorConstants.kMaxSpeed)
            .withSquaring(true)
            .withInvert(true);
        autoFactory = new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            drivetrain::followPath,
            true,
            drivetrain
        );
        autoRoutines = new AutoRoutines(autoFactory);
        
        initOrchestra();
        configureBindings();
        buildDashboards();

        //#region Default Commands
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(m_fieldX.getAsDouble() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(m_fieldY.getAsDouble() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(m_rotate.getAsDouble() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        /**
         * Handle manual control of the elevator
         */
        if(!ElevatorConstants.isDisabled) {
            elevator.setDefaultCommand(elevator.ElevatorMoveC(m_elevatorAxis));
        }
        //#endregion Default Commands
      
    }
    
    private void initOrchestra() {
        /**
         * This adds instruments to the orchestra. 
         * It is recommended there is at least 6-8 TalonFX devices in the orchestra.
         * Falcon500 are much louder than Krakens
         * 
         * This depends on each subsystem having a getMotors() method that returns an array of the TalonFX devices
         */
        if(AudioConstants.isEnabled) {
            ArrayList<TalonFX> instrumentsAll = new ArrayList<>();
            // The following subsystems return an array of motors (TalonFX[])
            for (TalonFX motor: drivetrain.getMotors()) {
                instrumentsAll.add(motor);
            }
            // for (TalonFX motor: shooter.getMotors()) {
            //   instrumentsAll.add(motor);
            // }
            TalonFX[] instruments = instrumentsAll.toArray(new TalonFX[instrumentsAll.size()]);
            orchestra.apply(instruments);
        }
    }

    /**
     * This performs robot reset initializations when the disabled() trigger fires.
     */
    private void resetRobot() {
        lighting.init();
        targeting.init();
        drivetrain.init();
        climber.init();
        coral.init();
        algae.init();
    }

    public void neutralRobot() {
        algae.AlgaeNeutral().ignoringDisable(true);
    }

    // Returns true if the alliance is red, otherwise false (blue)
    public static boolean isAllianceRed() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }
    
    private void configureBindings() {
        //#region RobotMode Triggers
        // bind to the disabled() trigger which happens any time the robot is disabled
        RobotModeTriggers.disabled().onTrue(
            new InstantCommand(this::resetRobot).ignoringDisable(true)
            .alongWith(
                new RepeatCommand(  //Lighting Disco Party!
                    lighting.setColorCommand(Colors.NCBLUE)
                    .andThen(new WaitCommand(0.5))
                    .andThen(lighting.setColorCommand(Colors.NCGREEN))
                    .andThen(new WaitCommand(0.5))
                    .andThen(lighting.setColorCommand(Colors.NCBLUE))
                    .andThen(new WaitCommand(0.15))
                    .andThen(lighting.setColorCommand(Colors.OFF))
                    .andThen(new WaitCommand(0.15))
                    .andThen(lighting.setColorCommand(Colors.NCBLUE))
                    .andThen(new WaitCommand(0.15))
                    .andThen(lighting.setColorCommand(Colors.NCGREEN))
                    .andThen(new WaitCommand(0.25))
                ).until(RobotModeTriggers.disabled().negate())
            )
            .alongWith(elevator.ElevatorStopC().ignoringDisable(true)).ignoringDisable(true)
        );
        // bind to the autonomous() and teleop() trigger which happens any time the robot is enabled in either of those modes
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(
            new InstantCommand(orchestra::stop).ignoringDisable(true)
            .andThen(() -> lighting.setColorCommand(Colors.OFF))
            .andThen(coral.CoralPositionC(CoralSubsystem.Position.SCORE))
            .andThen(wait(0.2))
            .andThen(coral.CoralStopC())
        );
        //#endregion

        //#region Trigger Actions
        if(!ClimberConstants.isDisabled) {
            /**
             * This monitors the hasCage trigger and immediately starts climbing until the climbComplete trigger, then goes to holding mode
             * Once the climbComplete trigger fires, the climber stops after 2 seconds
             */
            climber.hasCage.and(climber.climbComplete.negate()).onTrue(climber.climberMoveC(() -> ClimberConstants.kClimbPower));
            // .onFalse(climber.climberMoveC(() -> 0).andThen(new WaitCommand(2).andThen(climber.climberStopC())));
            // climber.climbComplete.onTrue(climber.climberHoldC().andThen(new WaitCommand(2)).andThen(climber.climberStopC()));
            climber.climbComplete.onTrue(climber.climberStopC());
        }
        //#endregion Trigger Actions

        //#region Driver Joystick
        if(AudioConstants.isEnabled) {
            /** Manage Music - Song list
            *  Brawl-Theme.chrp
            *  Megalovania.chrp
            *  Rickroll.chrp
            *  Still-Alive.chrp
            */
            dj.stadia().onTrue(new InstantCommand(() -> {
                if(orchestra.isPlaying()) {
                    orchestra.stop();
                } else {
                    orchestra.withMusic("Still-Alive.chrp").play();
                }
            }).ignoringDisable(true));
        }

        //POV left and right are robot-centric strafing
        dj.povLeft().whileTrue(drivetrain.applyRequest(() -> 
                robotdrive.withVelocityX(0).withVelocityY(SwerveConstants.kAlignStrafeSpeed)
            ).alongWith(
                new InstantCommand(() -> { NCDebug.Debug.debug("Debug: StrafeLeft"); })
            )
        );
        dj.povRight().whileTrue(drivetrain.applyRequest(() -> 
            robotdrive.withVelocityX(0).withVelocityY(-SwerveConstants.kAlignStrafeSpeed)
            ).alongWith(
                new InstantCommand(() -> { NCDebug.Debug.debug("Debug: StrafeRight"); })
            )
        );

        // dj.frame().onTrue((Commands.runOnce(drivetrain::zeroGyro)));
        // dj.stadia().onTrue(Commands.runOnce(drivetrain::addFakeVisionReading));

        //hold A to apply brake
        dj.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //hold B to point the wheels forward
        dj.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-dj.getLeftY(),-dj.getLeftX()))
        ));

        //target tracking/alignment buttons
        dj.leftBumper().and(dj.leftTrigger().negate()).onTrue(targeting.setTrackingRFLC());
        dj.rightBumper().and(dj.leftTrigger().negate()).onTrue(targeting.setTrackingRFCC());
        dj.rightTrigger().and(dj.leftTrigger().negate()).onTrue(targeting.setTrackingRFRC());
        dj.leftBumper().and(dj.leftTrigger()).onTrue(targeting.setTrackingRBLC());
        dj.rightBumper().and(dj.leftTrigger()).onTrue(targeting.setTrackingRBCC());
        dj.rightTrigger().and(dj.leftTrigger()).onTrue(targeting.setTrackingRBRC());
        
        // reset the field-centric heading on hamburger button press
        dj.hamburger().onTrue(drivetrain.resetGyroC());
        //#endregion Driver Joystick

        //#region Operator Joystick
        // CORAL STUFF
        /** OJ X - L1 Position (currently stow?) */
        oj.x().onTrue(
            coral.CoralPositionC(CoralSubsystem.Position.SCORE)
            .andThen(wait(0.4))
            .andThen(
                elevator.ElevatorPositionC(ElevatorSubsystem.Position.L1)
            )
            // .until(elevator::isAtTarget)
            .andThen(coral.CoralStopC())
        );
        /** OJ A - L2 Scoring Position */
        oj.a().onTrue(
            elevator.ElevatorPositionC(ElevatorSubsystem.Position.L2)
            .andThen(wait(CoralConstants.kWaitDelay))
            .andThen(coral.CoralPositionC(CoralSubsystem.Position.OUT))
        );
        /** OJ B - L3 Scoring Position */
        oj.b().onTrue(
            elevator.ElevatorPositionC(ElevatorSubsystem.Position.L3)
            .andThen(wait(CoralConstants.kWaitDelay))
            .andThen(coral.CoralPositionC(CoralSubsystem.Position.OUT))
        );
        /** OJ Y - L4 Scoring Position */
        oj.y().onTrue(
            elevator.ElevatorPositionC(ElevatorSubsystem.Position.L4)
            .andThen(wait(CoralConstants.kWaitDelay))
            .andThen(coral.CoralPositionC(CoralSubsystem.Position.OUT))
        );
        /** OJ Right Trigger - Score Coral sequence (from L2, L3, and L4) (hold trigger) */
        oj.rightTrigger().onTrue(
            elevator.ScoreC()
            .until(elevator::isAtTarget)
            .andThen(wait(0.2))
            .andThen(coral.CoralPositionC(CoralSubsystem.Position.SCORE))
        ).onFalse(
            wait(0.2)
            .andThen(coral.CoralStopC())
        );
        /** OJ Right Bumper - Return to previous position */
        oj.rightBumper().onTrue(
            elevator.LastPositionC()
            .andThen(wait(CoralConstants.kWaitDelay))
            .andThen(coral.CoralPositionC(CoralSubsystem.Position.OUT))
        );
        /** OJ Left Bumper - Human Player Intake */
        oj.leftBumper().onTrue(
            elevator.ElevatorPositionC(ElevatorSubsystem.Position.HP)
            .andThen(coral.CoralPositionC(CoralSubsystem.Position.OUT))
            .andThen(algae.setAlgaePositionC(AlgaeSubsystem.Position.UP))
        ).onFalse(
            coral.CoralPositionC(CoralSubsystem.Position.SCORE)
            .andThen(wait(0.5))
            .andThen(coral.CoralStopC())
        );

        // CLIMBER STUFF
        // OJ Frame - Manual climb override, hold for at least 1 second
        oj.frame().onTrue(
            wait(1.0).andThen(climber.climberMoveC(() -> ClimberConstants.kClimbPower))) //wait 1 second for manual override
        .onFalse(
          climber.climberStopC()
        );

        // ALGAE STUFF
        //Temp for testing
        // oj.povRight().onTrue(algae.startToroC(false)).onFalse(algae.stopToroC()); //intake
        // oj.povLeft().onTrue(algae.startToroC(true)).onFalse(algae.stopToroC()); //outtake
        // oj.povUp().onTrue(algae.setAlgaePositionC(AlgaeSubsystem.Position.UP)); //wrist up
        // oj.povDown().onTrue(algae.setAlgaePositionC(AlgaeSubsystem.Position.FLOOR)
        //     .andThen(algae.startToroC(false))
        // ).onFalse(algae.stopToroC()); //wrist down

        /** OJ POV Up - Barge scoring position */
        oj.povUp().onTrue(
          elevator.ElevatorPositionC(ElevatorSubsystem.Position.BARGE)
        );
        /** OJ POV Down - Processor scoring position */
        oj.povDown().onTrue(
          elevator.ElevatorPositionC(ElevatorSubsystem.Position.PROC)
          .andThen(wait(0.25))
          .andThen(algae.setAlgaePositionC(AlgaeSubsystem.Position.PROC))
        );
        /** OJ POV Left - Reef High Algae Pickup (do not stop toros, use limits) */
        oj.povLeft().onTrue(
          elevator.ElevatorPositionC(ElevatorSubsystem.Position.ALGAEHIGH)
          .andThen(wait(0.5))
          .andThen(algae.setAlgaePositionC(AlgaeSubsystem.Position.REEF))
          .andThen(algae.startToroC(false))
        );
        /** OJ POV Right - Reef High Algae Pickup (do not stop toros, use limits) */
        oj.povRight().onTrue(
          elevator.ElevatorPositionC(ElevatorSubsystem.Position.ALGAELOW)
          .andThen(wait(0.5))
          .andThen(algae.setAlgaePositionC(AlgaeSubsystem.Position.REEF))
          .andThen(algae.startToroC(false))
        );
        /** OJ L3 - Floor Algae Pickup (do not stop toros, use limits) */
        oj.leftStick().onTrue(
            algae.setAlgaePositionC(AlgaeSubsystem.Position.FLOOR)
            .andThen(algae.startToroC(false))
        ).onFalse(
            algae.setAlgaePositionC(AlgaeSubsystem.Position.UP)
        );
        /** OJ Left Trigger - Outtake Algae then stop toros */
        oj.leftTrigger().onTrue(
          algae.startToroC(true))
        .onFalse(
          algae.stopToroC()
        );
        /** OJ Ellipses - Stop toros */
        oj.ellipses().onTrue(
            algae.stopToroC()
        );
        oj.google().onTrue(
            algae.startToroC(false)
        ).onFalse(
            algae.stopToroC()
        );

        // Other OJ bindings
        // right stick elevator manual (see setup section)
        // stadia - open
        // hamburg - open
        //#endregion Operator Joystick

        //#region Programmer Joystick
        // Run SysId routines when holding ellipses/google and X/Y.
        // Note that each routine should be run exactly once in a single log.
        var m_mechanism = drivetrain; //drivetrain, elevator, coral, algae, climber
        // pj.ellipses().and(pj.a()).whileTrue(m_mechanism.runSysIdCommand());
        
        //seperately, but would need to use logic to see if we are atLimit
        pj.leftBumper().onTrue(Commands.runOnce(SignalLogger::start)); 
        pj.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        pj.ellipses().and(pj.y()).whileTrue(m_mechanism.sysIdDynamic(Direction.kForward));
        pj.ellipses().and(pj.x()).whileTrue(m_mechanism.sysIdDynamic(Direction.kReverse));
        pj.google().and(pj.y()).whileTrue(m_mechanism.sysIdQuasistatic(Direction.kForward));
        pj.google().and(pj.x()).whileTrue(m_mechanism.sysIdQuasistatic(Direction.kReverse));
        //#endregion Programmer Joystick

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    /**
     * Use this to pass the named command to the main Robot class.
     * @return command
     */
    public Command getAutonomousCommand() {
        if(AutonConstants.isDisabled) {
            NCDebug.Debug.debug("Robot: Selected auton routine is "+m_auto_chooser.getSelected().getName());
            return m_auto_chooser.getSelected();
        } else {
            NCDebug.Debug.debug("Robot: Selected auton routine is "+autoChooser.selectedCommand().getName());
            return autoChooser.selectedCommand();
        }
    }

    //#region Dashboard
    //From here down is all used for building the shuffleboard
    public void buildDashboards(){
        //List of Widgets: https://github.com/Gold872/elastic-dashboard/wiki/Widgets-List-&-Properties-Reference
        buildAutonChooser();
        buildDriverTab();
        // buildTab("Swerve");
        // buildTab("System");
        // buildPowerTab();
        // buildDebugTab();
        gyro.buildDashboards();
    }    
    
    public void buildAutonChooser() {
        //This builds the auton chooser, giving driver friendly names to the commands from above
        if(AutonConstants.isDisabled) {
            m_auto_chooser.setDefaultOption("None (Auto Disabled)", Commands.none());
        } else {
            // m_auto_chooser.setDefaultOption("Do Nothing", new cg_autonDoNothing(drive));
            if(AutonConstants.kUseChoreo) {
                autoFactory = drivetrain.createAutoFactory();
                autoRoutines = new AutoRoutines(autoFactory);
                // autoChooser.addRoutine("None (Do Nothing)", autoRoutines::doNothingAuto);
                autoChooser.addRoutine("Simple Forward", autoRoutines::simpleForwardAuto);
                // SmartDashboard.putData("Autonomous Chooser", autoChooser);
            // m_auto_chooser = autoChooser;
            }
        }
    }

    private void buildDriverTab(){
        ShuffleboardTab driverTab = buildTab("Driver");
        // Match Time - Cannot be programmatically placed, but we put it here for informative reasons
        driverTab.add("Match Time", "")
            .withPosition(0,2)
            .withSize(8,3)
            .withProperties(Map.of("time_display_mode","Minutes and Seconds","red_start_time",15,"yellow_start_time",30)) //mode: "Seconds Only" or "Minutes and Seconds"
            .withWidget("Match Time");
        // Auton Chooser
        if(AutonConstants.isDisabled) {
            driverTab.add("Autonomous Chooser", m_auto_chooser)
                .withPosition(0, 5)
                .withSize(8, 2)
                .withProperties(Map.of("sort_options",true))
                .withWidget("ComboBox Chooser");
        } else {
            // SmartDashboard.putData("Autonomous Chooser", autoChooser);
            driverTab.add("Autonomous Chooser", autoChooser)
                .withPosition(0, 5)
                .withSize(8, 2)
                .withProperties(Map.of("sort_options",true))
                .withWidget("ComboBox Chooser");
        }
            // FMS Info - Cannot be programmatically placed, but we put it here for informative reasons
            // driverTab.add("FMS Info", "")
            //   .withPosition(0,5)
            //   .withSize(8,2)
            //   .withWidget("FMSInfo");
            // Alerts
            // driverTab.add("Alerts", "")
            //   .withPosition(8,0)
            //   .withSize(11,7)
            //   .withWidget("Alerts");
        // Camera
        // driverTab.add("Camera", Robot.camera)
        //     .withPosition(18,0)
        //     .withSize(6,5)
        //     // .withProperties(Map.of("Glyph","CAMERA_RETRO","Show Glyph",true,"Show crosshair",true,"Crosshair color","#CCCCCC","Show controls",false))
        //     .withWidget("Camera Stream");
    }

    @SuppressWarnings({"unused"})
    private void buildDebugTab(){
        ShuffleboardTab debugTab = buildTab("Debug");
        debugTab.add("Command Scheduler", CommandScheduler.getInstance())
            .withPosition(0,2);      
    }

    @SuppressWarnings({"unused"})
    private void buildPowerTab(){
        ShuffleboardTab powerTab = buildTab("Power");
        powerTab.add("Power", power)
            .withPosition(0, 0);
            // .withSize(1, 1);
    }

    private ShuffleboardTab buildTab(String tabname) {
        return Shuffleboard.getTab(tabname);
    }
    //#endregion Dashboard

    //#region Convenience
    private Command wait(double seconds) {
      return new WaitCommand(seconds);
    }
    //#endregion
}
